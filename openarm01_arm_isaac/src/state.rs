// Shared state between the joint_states consumer (writer) and move_arm_joints
// (reader): the latest measured pose anchors each trajectory and the follow
// loop's chase. Joint limits are enforced by the sim engine, not here.

use std::sync::Arc;

use std::sync::Mutex;
use std::time::{Duration, Instant};

use crate::trajectory::JointVec;

// Telemetry older than this counts as no telemetry: the sim streams joint_states
// continuously, so a gap this long means the stream has stopped (paused/dead
// sim), and a move must not anchor or complete against a frozen pose.
const STALE_TELEMETRY: Duration = Duration::from_millis(500);

#[derive(Debug, Clone)]
pub struct JointStatesLatest {
    pub positions: Vec<f64>,
    pub recv_at: Instant,
}

#[derive(Debug, Default)]
pub struct SharedState {
    pub joint_states: Mutex<Option<JointStatesLatest>>,
}

pub fn new_shared() -> Arc<SharedState> {
    Arc::new(SharedState::default())
}

// Latest measured pose, or None until fresh telemetry arrives. Anchors a move's
// trajectory and the follow loop's chase at where the arm actually is; a stale
// sample is treated as absent so neither drives on a frozen pose.
pub fn snapshot_positions(state: &Arc<SharedState>) -> Option<JointVec> {
    let guard = state.joint_states.lock().unwrap_or_else(|p| p.into_inner());
    guard
        .as_ref()
        .filter(|s| s.recv_at.elapsed() <= STALE_TELEMETRY)
        .and_then(|s| s.positions.as_slice().try_into().ok())
}
