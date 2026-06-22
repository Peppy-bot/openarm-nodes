// Shared state between the telemetry pipeline (writer) and the move_gripper
// action handler (reader). The action handler reads this on each feedback
// tick to compute convergence + stall.

use std::sync::{Arc, Mutex};

#[derive(Debug, Clone)]
pub struct GripperStateLatest {
    pub positions: Vec<f64>,
}

#[derive(Debug, Default)]
pub struct SharedState {
    pub gripper_state: Mutex<Option<GripperStateLatest>>,
}

pub fn new_shared() -> Arc<SharedState> {
    Arc::new(SharedState::default())
}
