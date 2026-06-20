// SimBridge pipeline for this arm side: slice the bridge's whole-robot
// joint_states down to this arm's 7 joints, cache the latest pose + static
// limits for move_arm_joints, and re-emit through the shared
// openarm01_joint_state_source interface so sim and real arms publish state
// identically (told apart by arm_id).

use std::sync::Arc;

use peppygen::NodeRunner;
use peppygen::emitted_topics::openarm01_joint_state_source::v1::joint_states;
use peppylib::TopicPublisher;
use serde::Deserialize;
use sim_bridge_core::{BoxFuture, SimBridge};
use tracing::{error, info, warn};

use crate::config::ArmId;
use crate::state::{JointStatesLatest, SharedState};
use crate::transport::PeppylibTransport;

const ARM_DOF: usize = 7;

#[derive(Debug, Clone, Deserialize)]
struct JointStatesRaw {
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

pub async fn run(
    runner: Arc<NodeRunner>,
    arm_id: ArmId,
    state: Arc<SharedState>,
    transport: Arc<PeppylibTransport>,
) {
    let side = arm_id.side_word();
    info!(
        "telemetry: starting joint_states pipeline (arm_id={} side={})",
        arm_id.raw(),
        side
    );

    // sim_node = the publisher node name configured in
    // robot_initializer_mujoco's bridge_extension (defaults to "sim").
    let sim_node: Arc<str> = Arc::from("sim");

    // sim_bridge_core takes a tokio_util token; cancel it from an on_shutdown
    // hook so the bridge tears down cleanly before the runtime drops.
    let token = tokio_util::sync::CancellationToken::new();
    {
        let bridge_token = token.clone();
        runner.on_shutdown(async move { bridge_token.cancel() });
    }

    let joint_states_topic: Arc<str> = Arc::from("joint_states");

    // Pre-build the 7 canonical arm joint names for this side; emit_joint_states
    // looks each up by exact match so output order is joint1..joint7 regardless
    // of the publisher's joint order.
    let arm_joints: Arc<[String; ARM_DOF]> = Arc::new(std::array::from_fn(|i| {
        format!("openarm_{side}_joint{}", i + 1)
    }));

    // Declare the publisher once; the per-message closure clones the lock-free
    // handle and publishes on it.
    let js_pub = match joint_states::declare_publisher(&runner).await {
        Ok(publisher) => publisher,
        Err(e) => {
            error!("declare joint_states publisher: {e}");
            return;
        }
    };

    let arm_id_raw = arm_id.raw();
    let bridge = SimBridge::new(runner.clone(), transport, token, sim_node).sim_to_os(
        joint_states_topic,
        move |_runner, msg: JointStatesRaw| -> BoxFuture<std::result::Result<(), String>> {
            let state = state.clone();
            let names = arm_joints.clone();
            let publisher = js_pub.clone();
            Box::pin(async move {
                emit_joint_states(&publisher, arm_id_raw, &state, &names, &msg).await
            })
        },
    );

    bridge.run().await;
    info!("telemetry: pipeline exited");
}

async fn emit_joint_states(
    publisher: &TopicPublisher,
    arm_id: u8,
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

    let mut positions = Vec::with_capacity(ARM_DOF);
    let mut velocities = Vec::with_capacity(ARM_DOF);
    let mut limits_lower = Vec::with_capacity(ARM_DOF);
    let mut limits_upper = Vec::with_capacity(ARM_DOF);
    for name in expected {
        let Some(&src) = by_name.get(name.as_str()) else {
            continue;
        };
        positions.push(msg.positions[src]);
        velocities.push(msg.velocities[src]);
        if have_limits {
            limits_lower.push(msg.limits_lower[src]);
            limits_upper.push(msg.limits_upper[src]);
        }
    }

    // The interface is fixed at 7 DOF: a partial sample can't be published and
    // would corrupt the pose cache move_arm_joints anchors on, so drop it.
    let Ok(positions): std::result::Result<[f64; ARM_DOF], _> =
        positions.as_slice().try_into()
    else {
        warn!(
            got = positions.len(),
            "joint_states: incomplete arm sample (expected {ARM_DOF}); skipping"
        );
        return Ok(());
    };
    let velocities: [f64; ARM_DOF] = velocities
        .as_slice()
        .try_into()
        .expect("velocities length matches positions");

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

    // Cache the latest pose for move_arm_joints' trajectory anchor.
    {
        let mut latest = state.joint_states.lock().unwrap_or_else(|p| p.into_inner());
        *latest = Some(JointStatesLatest {
            positions: positions.to_vec(),
        });
    }

    let payload = joint_states::build_message(arm_id, positions, velocities)
        .map_err(|e| e.to_string())?;
    publisher.publish(payload).await.map_err(|e| e.to_string())
}
