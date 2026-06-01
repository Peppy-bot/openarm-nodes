// Shared state between the telemetry pipeline (writer) and the action
// handlers (readers). The telemetry pipeline updates the latest joint
// positions on every incoming raw joint_states; move_arm_joints reads it on
// each feedback tick for convergence + stall detection, and
// get_joint_positions reads it for one-shot service responses.

use std::sync::Arc;

use tokio::sync::Mutex;

#[derive(Debug, Clone)]
pub struct JointStatesLatest {
    pub positions: Vec<f64>,
}

#[derive(Debug, Default)]
pub struct SharedState {
    pub joint_states: Mutex<Option<JointStatesLatest>>,
}

pub fn new_shared() -> Arc<SharedState> {
    Arc::new(SharedState::default())
}
