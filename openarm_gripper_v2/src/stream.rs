// Always-on gripper_states publisher: emits the measured opening, effort, and
// effort ceiling at state_rate_hz regardless of mode to the paired backbone on
// the pairing's `gripper_states` topic (a legal no-op while unpaired; any
// monitor observes the pairing). Reads the motor's cached state (no CAN
// traffic of its own), so it never contends with the follow loop for the bus;
// the follow loop refreshes that cache every tick, so the reading is always
// current.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use openarm_can::{GripperCan, PosForce};
use peppygen::NodeRunner;
use peppygen::paired_topics::backbone;
use peppylib::runtime::CancellationToken;
use tracing::{error, warn};

use crate::geometry::Geometry;
use crate::health::{LatchedWarn, capture_stamp};

pub async fn run(
    runner: Arc<NodeRunner>,
    period: Duration,
    geometry: Geometry,
    effort_ceiling_nm: f64,
    gripper: Arc<Mutex<GripperCan<PosForce>>>,
    token: CancellationToken,
) {
    let peer_pub = match backbone::gripper_states::declare_publisher(&runner).await {
        Ok(p) => p,
        Err(e) => return error!("declare paired gripper_states publisher: {e}"),
    };
    let mut peer_warn = LatchedWarn::new("paired gripper_states publish");
    loop {
        tokio::select! {
            _ = token.cancelled() => return,
            _ = tokio::time::sleep(period) => {}
        }
        let state = gripper
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .get_state();
        let opening = geometry.motor_rad_to_fraction(state.position);
        // Motor current-derived torque estimate (N*m at the motor shaft)
        // in the opening frame (positive toward open on either side),
        // carried as the pairing effort.
        let effort = geometry.motor_torque_to_effort(state.torque);
        // Skip a poisoned sample rather than publishing NaN/Inf to consumers,
        // matching the finiteness guards on the command paths.
        if !opening.is_finite() || !effort.is_finite() {
            warn!("gripper_states: skipping non-finite motor sample");
            continue;
        }
        let peer_result = match capture_stamp().and_then(|stamp| {
            backbone::gripper_states::build_message(stamp, opening, effort, effort_ceiling_nm)
                .map_err(|e| e.to_string())
        }) {
            Ok(msg) => peer_pub.publish(msg).await.map_err(|e| e.to_string()),
            Err(e) => Err(e),
        };
        match peer_result {
            Ok(()) => peer_warn.success(),
            Err(e) => peer_warn.failure(&e),
        }
    }
}
