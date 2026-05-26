// Shared state between the telemetry pipeline (writer) and the action
// handlers (readers).
//
// telemetry::run writes the latest per-side joint state on every incoming
// raw `joint_states_<side>` message. move_arm_joints reads it on each
// feedback tick to compute convergence + stall; get_joint_positions reads
// it for one-shot service responses.

use std::sync::Arc;

use tokio::sync::Mutex;

#[derive(Debug, Clone)]
#[allow(dead_code)] // step/velocities/stamp are populated for observability + future services
pub struct JointStatesLatest {
    pub step: u64,
    pub positions: Vec<f64>,
    pub velocities: Vec<f64>,
    pub stamp: f64,
}

#[derive(Debug, Default)]
pub struct SharedState {
    pub joint_states: Mutex<Option<JointStatesLatest>>,
}

pub fn new_shared() -> Arc<SharedState> {
    Arc::new(SharedState::default())
}
