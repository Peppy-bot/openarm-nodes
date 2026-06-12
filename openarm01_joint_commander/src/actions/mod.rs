use std::time::Duration;

use crate::state::SharedState;

pub mod move_arm_joints;
pub mod move_gripper;

/// Cap `original` by the time remaining to the shutdown deadline, once the
/// shutdown hook has set one. Outside shutdown this is `original` unchanged,
/// so user-initiated preempts keep the full cancel/result windows.
fn bounded_by_shutdown(state: &SharedState, original: Duration) -> Duration {
    let deadline = state
        .lock()
        .unwrap_or_else(|p| p.into_inner())
        .shutdown_deadline;
    match deadline {
        Some(deadline) => {
            original.min(deadline.saturating_duration_since(tokio::time::Instant::now()))
        }
        None => original,
    }
}
