// Shared state between the telemetry pipeline (writer) and the move_gripper
// action handler (reader). The telemetry pipeline updates the latest gripper
// state on every incoming raw gripper_state_<side> message; the action handler
// reads it on each feedback tick to compute convergence + stall.

use std::sync::Arc;

use tokio::sync::Mutex;

#[derive(Debug, Clone)]
pub struct GripperStateLatest {
    pub step: u64,
    pub positions: Vec<f64>,
    pub stamp: f64,
}

#[derive(Debug, Default)]
pub struct SharedState {
    pub gripper_state: Mutex<Option<GripperStateLatest>>,
}

pub fn new_shared() -> Arc<SharedState> {
    Arc::new(SharedState::default())
}
