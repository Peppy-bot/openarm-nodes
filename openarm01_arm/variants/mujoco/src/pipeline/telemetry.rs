// Telemetry pipelines: subscribe to raw peppylib telemetry from
// robot_initializer:mujoco's in-process bridge extension, re-emit as typed
// peppygen on the arm's contract topics, and update the shared state that
// the move_arm_joints action handler + get_joint_positions service read.
//
// One SimBridge per arm instance (left or right). Each builds sim_to_os
// pipelines:
//
//   raw joint_states_<side>   →  typed joint_states  (+ shared cache)
//   raw tf_tree (whole world) →  typed tf_tree       (filtered to side)
//
// IMU pipelines are stubbed but disabled: the monolith's sim_bridge.json5
// publishers for `imu_<side>` are commented out pending MJCF/USD bodies
// (see openarm01_robot_initializer/.../config/sim_bridge.json5:38-39).
// When the monolith ships IMU bodies, uncomment the .sim_to_os call below
// — no other change needed.

use std::sync::Arc;

use peppygen::NodeRunner;
use peppygen::emitted_topics::{joint_states, tf_tree};
use serde::Deserialize;
use sim_bridge_core::{BoxFuture, DaemonState, SimBridge};
use tracing::{error, info};

use crate::config::ArmId;
use crate::state::{JointStatesLatest, SharedState};

const ROBOT_NAME: &str = "openarm";

// ---------------------------------------------------------------------------
// Raw peppylib message shapes — mirror what robot_initializer:mujoco emits.
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Deserialize)]
struct JointStatesRaw {
    #[allow(dead_code)]
    robot: String,
    step: u64,
    positions: Vec<f64>,
    velocities: Vec<f64>,
    stamp: f64,
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
    #[allow(dead_code)]
    robot: String,
    step: u64,
    frames: Vec<TfFrameRaw>,
    stamp: f64,
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

pub async fn run(runner: Arc<NodeRunner>, arm_id: ArmId, state: Arc<SharedState>) {
    let side = arm_id.side_word();
    info!("telemetry: starting pipelines (arm_id={} side={})", arm_id.0, side);

    let daemon = match peppylib::info(&runner, None).await {
        Ok(info) => DaemonState {
            core_node_name: info.core_node_name,
            messaging_port: info.messaging_port,
        },
        Err(e) => {
            error!("telemetry: peppylib::info failed: {e}");
            return;
        }
    };

    // sim_node = the publisher node name configured in
    // robot_initializer:mujoco's bridge_extension (defaults to "sim").
    let sim_node: Arc<str> = Arc::from("sim");
    let token = runner.cancellation_token().clone();

    let joint_states_topic: Arc<str> = Arc::from(format!("joint_states_{side}"));
    let tf_tree_topic: Arc<str> = Arc::from("tf_tree");

    // Frame-name prefix used to filter the whole-world tf_tree down to
    // frames that belong to this arm side. Matches the openarm MJCF naming
    // convention: `openarm_<side>_link*`, `openarm_<side>_joint*`, etc.
    let frame_prefix: Arc<str> = Arc::from(format!("openarm_{side}_").as_str());

    let state_for_js = state.clone();

    let bridge = SimBridge::new(runner.clone(), daemon, token, sim_node)
        .sim_to_os(joint_states_topic, move |runner, msg: JointStatesRaw|
            -> BoxFuture<std::result::Result<(), String>>
        {
            let state = state_for_js.clone();
            Box::pin(async move {
                // Cache for the action handler's feedback loop + the
                // get_joint_positions service.
                {
                    let mut latest = state.joint_states.lock().await;
                    *latest = Some(JointStatesLatest {
                        step: msg.step,
                        positions: msg.positions.clone(),
                        velocities: msg.velocities.clone(),
                        stamp: msg.stamp,
                    });
                }
                emit_joint_states(&runner, &msg).await
            })
        })
        .sim_to_os(tf_tree_topic, move |runner, msg: TfTreeRaw|
            -> BoxFuture<std::result::Result<(), String>>
        {
            let prefix = frame_prefix.clone();
            Box::pin(async move { emit_tf_tree(&runner, &msg, &prefix).await })
        });
        // .sim_to_os(imu_left_topic, ...) — TODO: enable when robot_initializer's
        //   bridge_extension publishes imu_<side> (sim_bridge.json5:38-39).
        // .sim_to_os(imu_right_topic, ...)

    bridge.run().await;
    info!("telemetry: pipelines exited");
}

// ---------------------------------------------------------------------------
// Per-topic emit helpers — keep main builder readable.
// ---------------------------------------------------------------------------

async fn emit_joint_states(
    runner: &Arc<NodeRunner>,
    msg: &JointStatesRaw,
) -> std::result::Result<(), String> {
    joint_states::emit(
        runner,
        ROBOT_NAME.into(),
        msg.step,
        msg.positions.clone(),
        msg.velocities.clone(),
        msg.stamp,
    )
    .await
    .map_err(|e| e.to_string())
}

async fn emit_tf_tree(
    runner: &Arc<NodeRunner>,
    msg: &TfTreeRaw,
    frame_prefix: &str,
) -> std::result::Result<(), String> {
    let frames: Vec<tf_tree::MessageFramesItem> = msg
        .frames
        .iter()
        .filter(|f| f.name.starts_with(frame_prefix))
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
        msg.stamp,
    )
    .await
    .map_err(|e| e.to_string())
}
