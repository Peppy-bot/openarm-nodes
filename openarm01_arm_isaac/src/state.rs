// Shared state between the telemetry pipeline (writer) and the action handlers
// + get_joint_positions service (readers). move_arm_joints reads this on each
// feedback tick for convergence + stall detection.

use std::sync::Arc;

use std::sync::Mutex;

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
