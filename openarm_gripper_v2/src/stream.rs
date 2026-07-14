// Always-on gripper_states publisher: emits the measured opening at
// state_rate_hz regardless of mode: to the paired backbone on the pairing's
// `gripper_states` topic (a legal no-op while unpaired) and to observers on
// the broadcast stream (tagged with `gripper_id`). Reads the motor's cached
// state (no CAN traffic of its own), so it never contends with the follow loop
// for the bus; the follow loop refreshes that cache every tick, so the reading
// is always current.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use openarm_can::GripperCan;
use peppygen::NodeRunner;
use peppygen::emitted_topics::openarm_gripper_states::v1::gripper_states;
use peppygen::pairings::backbone;
use peppylib::runtime::CancellationToken;
use tracing::{error, warn};

use crate::geometry::Geometry;

pub async fn run(
    runner: Arc<NodeRunner>,
    gripper_id: u8,
    state_rate_hz: u32,
    geometry: Geometry,
    gripper: Arc<Mutex<GripperCan>>,
    token: CancellationToken,
) {
    let publisher = match gripper_states::declare_publisher(&runner).await {
        Ok(p) => p,
        Err(e) => return error!("declare gripper_states publisher: {e}"),
    };
    let peer_pub = match backbone::gripper_states::declare_publisher(&runner).await {
        Ok(p) => p,
        Err(e) => return error!("declare paired gripper_states publisher: {e}"),
    };
    let period = Duration::from_micros(1_000_000 / state_rate_hz as u64);
    let mut broadcast_failing = false;
    let mut peer_failing = false;
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
        let force = state.torque;
        // Skip a poisoned sample rather than publishing NaN/Inf to consumers,
        // matching the finiteness guards on the command paths.
        if !opening.is_finite() || !force.is_finite() {
            warn!("gripper_states: skipping non-finite motor sample");
            continue;
        }
        // The broadcast and paired publishes serve unrelated consumers, so
        // each runs and reports independently: one failing must not starve
        // the other.
        let broadcast_result = match gripper_states::build_message(gripper_id, opening, force) {
            Ok(msg) => publisher.publish(msg).await.map_err(|e| e.to_string()),
            Err(e) => Err(e.to_string()),
        };
        match broadcast_result {
            Ok(()) => broadcast_failing = false,
            Err(e) if !broadcast_failing => {
                broadcast_failing = true;
                warn!("gripper_states publish failing, suppressing repeats: {e}");
            }
            Err(_) => {}
        }
        let peer_result = match backbone::gripper_states::build_message(opening) {
            Ok(msg) => peer_pub.publish(msg).await.map_err(|e| e.to_string()),
            Err(e) => Err(e.to_string()),
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
