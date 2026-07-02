// Shared state between the gripper_states consumer (writer) and the move_gripper
// action handler (reader). The action handler reads this on each feedback tick to
// compute convergence + stall against the measured opening.

use std::sync::{Arc, Mutex};
use std::time::Instant;

#[derive(Debug, Clone, Copy)]
pub struct GripperStateLatest {
    // Measured aperture (m): 0.0 closed, ~0.044 fully open. The sum of the two
    // finger positions, as emitted on gripper_states.
    pub opening: f64,
    // When this sample was received, so the move action can ignore stale
    // telemetry rather than report a false reached/stall from a frozen value.
    pub recv_at: Instant,
}

#[derive(Debug, Default)]
pub struct SharedState {
    pub gripper_state: Mutex<Option<GripperStateLatest>>,
}

pub fn new_shared() -> Arc<SharedState> {
    Arc::new(SharedState::default())
}
