// Always-on gripper_states publisher: emits the measured opening at
// state_rate_hz regardless of mode to the paired backbone on the pairing's
// `gripper_states` topic (a legal no-op while unpaired; any monitor observes
// the pairing). Reads the motor's cached state (no CAN traffic of its own), so
// it never contends with the follow loop for the bus; the follow loop
// refreshes that cache every tick, so the reading is always current.

use std::sync::{Arc, Mutex};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use openarm_can::GripperCan;
use peppygen::NodeRunner;
use peppygen::paired_topics::backbone;
use peppylib::runtime::CancellationToken;
use tracing::{error, warn};

use crate::geometry;

/// Pairing stamp from the daemon-resolved clock (sim time under a simulated
/// clock), so consumers age samples on the same timeline they read. Errors
/// until the clock delivers its first tick.
fn pairing_stamp() -> Result<SystemTime, String> {
    let ns = peppygen::clock::now_ns().map_err(|e| format!("clock not ready: {e}"))?;
    Ok(UNIX_EPOCH + Duration::from_nanos(ns))
}

pub async fn run(
    runner: Arc<NodeRunner>,
    state_rate_hz: u32,
    gripper: Arc<Mutex<GripperCan>>,
    token: CancellationToken,
) {
    let peer_pub = match backbone::gripper_states::declare_publisher(&runner).await {
        Ok(p) => p,
        Err(e) => return error!("declare paired gripper_states publisher: {e}"),
    };
    let period = Duration::from_micros(1_000_000 / state_rate_hz as u64);
    let mut peer_failing = false;
    loop {
        tokio::select! {
            _ = token.cancelled() => return,
            _ = tokio::time::sleep(period) => {}
        }
        let motor_rad = gripper
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .get_state()
            .position;
        let opening = geometry::motor_rad_to_fraction(motor_rad);
        // Skip a poisoned sample rather than publishing NaN/Inf to consumers,
        // matching the finiteness guards on the command paths.
        if !opening.is_finite() {
            warn!("gripper_states: skipping non-finite motor sample");
            continue;
        }
        // The v1.0 prismatic gripper neither senses grip force nor caps it
        // (MIT mode has no per-command force limit): report 0 effort and a
        // 0 ceiling (no effort control).
        let effort = 0.0;
        let max_effort = 0.0;
        let peer_result = match pairing_stamp().and_then(|stamp| {
            backbone::gripper_states::build_message(stamp, opening, effort, max_effort)
                .map_err(|e| e.to_string())
        }) {
            Ok(msg) => peer_pub.publish(msg).await.map_err(|e| e.to_string()),
            Err(e) => Err(e),
        };
        match peer_result {
            Ok(()) => peer_failing = false,
            Err(e) if !peer_failing => {
                peer_failing = true;
                warn!("paired gripper_states publish failing, suppressing repeats: {e}");
            }
            Err(_) => {}
        }
    }
}
