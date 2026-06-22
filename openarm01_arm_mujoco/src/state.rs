// Shared state between the telemetry pipeline (writer) and move_arm_joints
// (reader): the latest measured pose anchors each trajectory, and the cached
// per-joint limits clamp goal targets.

use std::sync::Arc;

use std::sync::Mutex;

use crate::trajectory::{ARM_DOF as DOF, JointVec};

#[derive(Debug, Clone)]
pub struct JointStatesLatest {
    pub positions: Vec<f64>,
}

#[derive(Debug, Default)]
pub struct SharedState {
    pub joint_states: Mutex<Option<JointStatesLatest>>,
    // Per-joint (lower, upper) limits from the sim model, sliced to this arm's
    // 7 joints. move_arm_joints and the follow loop clamp targets into this range.
    pub joint_limits: Mutex<Option<Vec<(f64, f64)>>>,
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

// Clamp a target into the cached per-joint limits, or None if limits are not
// ready yet (so the caller holds rather than commanding an unclamped pose).
pub fn clamp_to_limits(state: &Arc<SharedState>, target: JointVec) -> Option<JointVec> {
    let limits = state
        .joint_limits
        .lock()
        .unwrap_or_else(|p| p.into_inner())
        .clone()
        .filter(|l| l.len() == DOF)?;
    let mut clamped = target;
    for (q, &(lo, hi)) in clamped.iter_mut().zip(limits.iter()) {
        // Skip a malformed tuple (NaN, inf, or lo > hi) rather than let
        // f64::clamp panic; a bad model range degrades to no clamp, not a crash.
        if lo.is_finite() && hi.is_finite() && lo <= hi {
            *q = q.clamp(lo, hi);
        }
    }
    Some(clamped)
}
