// Measured-opening publisher, running at state_rate_hz regardless of mode. It
// serves three unrelated consumers, each independently so one failing never
// starves the others:
//   - the paired backbone, on the pairing's `gripper_states` (the control loop's
//     state input; a legal no-op while unpaired);
//   - any observer such as a recorder, on the generic joint_states contract (the
//     measured opening as positions[0], with the sensed grip force as effort[0]);
//   - the same observers, on the generic joint_commands contract (the opening
//     setpoint this gripper is tracking, held-last, so a recorder captures the
//     action aligned with the measured opening).
// The joint_commands stream lives here, not on the backbone that computes the
// setpoint: pairing traffic is unobservable by third parties, and the backbone
// is one node driving both grippers and cannot emit a per-gripper stream.
//
// Reads the motor's cached state (no CAN traffic of its own), so it never
// contends with the follow loop for the bus; the follow loop refreshes that
// cache every tick, so the reading is always current.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use openarm_can::GripperCan;
use peppygen::NodeRunner;
use peppygen::emitted_topics::joint_commands::v1::joint_commands;
use peppygen::emitted_topics::joint_states::v1::joint_states;
use peppygen::pairings::backbone;
use peppylib::runtime::CancellationToken;
use tokio::sync::watch;
use tracing::{error, warn};

use crate::command_stream::GripperCommand;
use crate::geometry::Geometry;

pub async fn run(
    runner: Arc<NodeRunner>,
    state_rate_hz: u32,
    geometry: Geometry,
    gripper: Arc<Mutex<GripperCan>>,
    tracked: watch::Receiver<Option<GripperCommand>>,
    token: CancellationToken,
) {
    let observer_pub = match joint_states::declare_publisher(&runner).await {
        Ok(p) => p,
        Err(e) => return error!("declare joint_states publisher: {e}"),
    };
    let command_pub = match joint_commands::declare_publisher(&runner).await {
        Ok(p) => p,
        Err(e) => return error!("declare joint_commands publisher: {e}"),
    };
    let peer_pub = match backbone::gripper_states::declare_publisher(&runner).await {
        Ok(p) => p,
        Err(e) => return error!("declare paired gripper_states publisher: {e}"),
    };
    let period = Duration::from_micros(1_000_000 / state_rate_hz as u64);
    let mut observer_failing = false;
    let mut command_failing = false;
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
        // Measured opening on the generic observer contract as a 1-DOF joint
        // (positions[0]); the v2 gripper senses grip force, reported as effort[0].
        let observer = match joint_states::build_message(vec![opening], Vec::new(), vec![force]) {
            Ok(m) => observer_pub.publish(m).await.map_err(|e| e.to_string()),
            Err(e) => Err(e.to_string()),
        };
        match observer {
            Ok(()) => observer_failing = false,
            Err(e) if !observer_failing => {
                observer_failing = true;
                warn!("joint_states publish failing, suppressing repeats: {e}");
            }
            Err(_) => {}
        }
        // The opening setpoint commanded to this gripper, held-last.
        let latest_command = *tracked.borrow();
        if let Some(command) = latest_command {
            let result = match joint_commands::build_message(vec![command.opening]) {
                Ok(m) => command_pub.publish(m).await.map_err(|e| e.to_string()),
                Err(e) => Err(e.to_string()),
            };
            match result {
                Ok(()) => command_failing = false,
                Err(e) if !command_failing => {
                    command_failing = true;
                    warn!("joint_commands publish failing, suppressing repeats: {e}");
                }
                Err(_) => {}
            }
        }
        // Measured opening to the paired backbone (the control loop's input).
        let peer = match backbone::gripper_states::build_message(opening) {
            Ok(m) => peer_pub.publish(m).await.map_err(|e| e.to_string()),
            Err(e) => Err(e.to_string()),
        };
        match peer {
            Ok(()) => peer_failing = false,
            Err(e) if !peer_failing => {
                peer_failing = true;
                warn!("paired gripper_states publish failing, suppressing repeats: {e}");
            }
            Err(_) => {}
        }
    }
}
