// SimBridge pipelines for this arm side: slice whole-robot joint_states to
// the 7 joints (cached for the action handler) and filter tf_tree by side
// prefix. IMU stubbed until bridge_extension publishes imu_<side>. Stamps
// via peppygen::clock so use_sim_time wins.

use std::sync::Arc;

use peppygen::NodeRunner;
use peppygen::emitted_topics::{joint_states, tf_tree};
use serde::Deserialize;
use sim_bridge_core::{BoxFuture, DaemonState, SimBridge};
use tracing::{info, warn};

use crate::config::ArmId;
use crate::state::{JointStatesLatest, SharedState};

const ROBOT_NAME: &str = "openarm";
const ARM_DOF: usize = 7;

#[derive(Debug, Clone, Deserialize)]
struct JointStatesRaw {
    step: u64,
    joint_names: Vec<String>,
    positions: Vec<f64>,
    velocities: Vec<f64>,
    // Static joint limits from the sim model (optional — older monoliths
    // don't send them). Same order as joint_names.
    #[serde(default)]
    limits_lower: Vec<f64>,
    #[serde(default)]
    limits_upper: Vec<f64>,
}

#[derive(Debug, Clone, Deserialize)]
struct TfFrameRaw {
    name: String,
    parent: String,
    position: [f64; 3],
    orientation: [f64; 4],
}

#[derive(Debug, Clone, Deserialize)]
struct TfTreeRaw {
    step: u64,
    frames: Vec<TfFrameRaw>,
}

pub async fn run(
    runner: Arc<NodeRunner>,
    arm_id: ArmId,
    state: Arc<SharedState>,
    daemon: DaemonState,
) {
    let side = arm_id.side_word();
    info!(
        "telemetry: starting pipelines (arm_id={} side={})",
        arm_id.raw(),
        side
    );

    // sim_node = the publisher node name configured in
    // robot_initializer_mujoco's bridge_extension (defaults to "sim").
    let sim_node: Arc<str> = Arc::from("sim");
    let token = runner.cancellation_token().clone();

    let joint_states_topic: Arc<str> = Arc::from("joint_states");
    let tf_tree_topic: Arc<str> = Arc::from("tf_tree");

    // Filter tf_tree to this arm side. Both arm and gripper frames share the
    // `openarm_<side>_` prefix; explicitly exclude `openarm_<side>_finger*` so
    // gripper-owned frames don't leak into the arm topic.
    let arm_frame_prefix: Arc<str> = Arc::from(format!("openarm_{side}_").as_str());
    let finger_frame_prefix: Arc<str> = Arc::from(format!("openarm_{side}_finger").as_str());

    // Pre-build the 7 canonical arm joint names for this side; emit_joint_states
    // looks each up by exact match so cache ordering doesn't depend on publisher
    // joint order.
    let arm_joints: Arc<[String; ARM_DOF]> = Arc::new(std::array::from_fn(|i| {
        format!("openarm_{side}_joint{}", i + 1)
    }));

    let state_for_js = state.clone();
    let arm_joints_js = arm_joints.clone();

    let bridge = SimBridge::new(runner.clone(), daemon, token, sim_node)
        .sim_to_os(
            joint_states_topic,
            move |runner, msg: JointStatesRaw| -> BoxFuture<std::result::Result<(), String>> {
                let state = state_for_js.clone();
                let names = arm_joints_js.clone();
                Box::pin(async move { emit_joint_states(&runner, &state, &names, &msg).await })
            },
        )
        .sim_to_os(
            tf_tree_topic,
            move |runner, msg: TfTreeRaw| -> BoxFuture<std::result::Result<(), String>> {
                let include = arm_frame_prefix.clone();
                let exclude = finger_frame_prefix.clone();
                Box::pin(async move { emit_tf_tree(&runner, &msg, &include, &exclude).await })
            },
        );

    bridge.run().await;
    info!("telemetry: pipelines exited");
}

fn stamp_now_secs() -> f64 {
    match peppygen::clock::now_ns() {
        Ok(ns) => ns as f64 / 1e9,
        Err(e) => {
            warn!("peppygen::clock::now_ns failed ({e}); stamping with 0.0");
            0.0
        }
    }
}

async fn emit_joint_states(
    runner: &Arc<NodeRunner>,
    state: &Arc<SharedState>,
    expected: &[String; ARM_DOF],
    msg: &JointStatesRaw,
) -> std::result::Result<(), String> {
    // Sanity: monolith should send equal-length vectors.
    let n_names = msg.joint_names.len();
    if msg.positions.len() != n_names || msg.velocities.len() != n_names {
        return Err(format!(
            "joint_states payload length mismatch: names={n_names} \
             positions={} velocities={}",
            msg.positions.len(),
            msg.velocities.len()
        ));
    }

    // Index by name so the output order is deterministic (joint1..joint7) and
    // doesn't drift with publisher reordering.
    let by_name: std::collections::HashMap<&str, usize> = msg
        .joint_names
        .iter()
        .enumerate()
        .map(|(i, n)| (n.as_str(), i))
        .collect();

    let have_limits =
        msg.limits_lower.len() == n_names && msg.limits_upper.len() == n_names;

    let mut joint_names = Vec::with_capacity(ARM_DOF);
    let mut positions = Vec::with_capacity(ARM_DOF);
    let mut velocities = Vec::with_capacity(ARM_DOF);
    let mut limits_lower = Vec::with_capacity(ARM_DOF);
    let mut limits_upper = Vec::with_capacity(ARM_DOF);
    for name in expected {
        let Some(&src) = by_name.get(name.as_str()) else {
            continue;
        };
        joint_names.push(name.clone());
        positions.push(msg.positions[src]);
        velocities.push(msg.velocities[src]);
        if have_limits {
            limits_lower.push(msg.limits_lower[src]);
            limits_upper.push(msg.limits_upper[src]);
        }
    }

    // Limits are static — cache once so move_arm_joints can clamp targets.
    if have_limits && limits_lower.len() == ARM_DOF {
        let mut cached = state.joint_limits.lock().unwrap_or_else(|p| p.into_inner());
        if cached.is_none() {
            *cached = Some(
                limits_lower
                    .iter()
                    .zip(limits_upper.iter())
                    .map(|(&lo, &hi)| (lo, hi))
                    .collect(),
            );
        }
    }

    // Only cache complete 7-DOF samples — a partial payload would corrupt the
    // pose snapshot move_arm_joints + get_joint_positions read on each tick.
    if positions.len() == ARM_DOF {
        let mut latest = state.joint_states.lock().unwrap_or_else(|p| p.into_inner());
        *latest = Some(JointStatesLatest {
            positions: positions.clone(),
        });
    } else {
        warn!(
            got = positions.len(),
            "joint_states: incomplete arm sample (expected {ARM_DOF}); not caching"
        );
    }

    joint_states::emit(
        runner,
        ROBOT_NAME.into(),
        msg.step,
        joint_names,
        positions,
        velocities,
        stamp_now_secs(),
    )
    .await
    .map_err(|e| e.to_string())
}

async fn emit_tf_tree(
    runner: &Arc<NodeRunner>,
    msg: &TfTreeRaw,
    include_prefix: &str,
    exclude_prefix: &str,
) -> std::result::Result<(), String> {
    let frames: Vec<tf_tree::MessageFramesItem> = msg
        .frames
        .iter()
        .filter(|f| f.name.starts_with(include_prefix) && !f.name.starts_with(exclude_prefix))
        .map(|f| tf_tree::MessageFramesItem {
            name: f.name.clone(),
            parent: f.parent.clone(),
            position: f.position,
            orientation: f.orientation,
        })
        .collect();

    tf_tree::emit(
        runner,
        ROBOT_NAME.into(),
        msg.step,
        frames,
        stamp_now_secs(),
    )
    .await
    .map_err(|e| e.to_string())
}
