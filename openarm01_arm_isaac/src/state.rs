// Shared state between the joint_states consumer (writer) and move_arm_joints
// (reader): the latest measured pose anchors each trajectory and the follow
// loop's chase. Joint limits are enforced by the sim engine, not here.

use std::sync::Arc;

use std::sync::Mutex;

use crate::trajectory::JointVec;

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

// Latest measured pose, or None until telemetry arrives. Anchors a move's
// trajectory and the follow loop's chase at where the arm actually is.
pub fn snapshot_positions(state: &Arc<SharedState>) -> Option<JointVec> {
    let guard = state.joint_states.lock().unwrap_or_else(|p| p.into_inner());
    guard
        .as_ref()
        .and_then(|s| s.positions.as_slice().try_into().ok())
}
