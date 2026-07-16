// Measured-opening publisher, running at state_rate_hz regardless of mode. It
// serves three unrelated consumers, each independently so one failing never
// starves the others:
//   - the paired backbone, on the pairing's `gripper_states` (the control
//     loop's state input; a legal no-op while unpaired);
//   - any observer such as a recorder, on the generic joint_states contract
//     (the measured opening as positions[0]);
//   - the same observers, on the generic joint_commands contract (the opening
//     setpoint this gripper is currently tracking, so a recorder can capture
//     the action aligned with the measured opening).
// The joint_commands stream lives on the gripper, not the backbone that
// computes the setpoint, for two reasons: pairing traffic (the gripper_link the
// setpoint arrives on) rides a wire discriminator no ordinary subscription
// matches, so no observer can read it off the pairing; and the backbone is one
// node driving both grippers, which cannot emit a per-gripper joint_commands (a
// contract topic has one wire identity per node). The gripper can, because it
// already holds its own side's setpoint: it republishes both directions of its
// pairing here, the opening it sends up and the command it receives down, so a
// recorder's action aligns with this gripper's state.
//
// Reads the motor's cached state (no CAN traffic of its own), so it never
// contends with the follow loop for the bus; the follow loop refreshes that
// cache every tick, so the reading is always current.

use std::sync::{Arc, Mutex};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use openarm_can::GripperCan;
use peppygen::NodeRunner;
use peppygen::emitted_topics::joint_commands::v1::joint_commands;
use peppygen::emitted_topics::joint_states::v1::joint_states;
use peppygen::pairings::backbone;
use peppylib::runtime::CancellationToken;
use tokio::sync::watch;
use tracing::{error, warn};

use crate::command_stream::GripperCommand;
use crate::geometry;

/// The peppy-synchronized clock as a [`SystemTime`] stamp for the generic
/// emissions, or an error while the clock is not ready (before
/// `peppygen::clock::init`, or in sim mode before the first tick).
fn stamp_now() -> Result<SystemTime, String> {
    let ns = peppygen::clock::now_ns().map_err(|e| e.to_string())?;
    Ok(UNIX_EPOCH + Duration::from_nanos(ns))
}

pub async fn run(
    runner: Arc<NodeRunner>,
    state_rate_hz: u32,
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
    let mut clock_failing = false;
    let mut observer_failing = false;
    let mut command_failing = false;
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
        // One synchronized stamp per tick, shared by joint_states and
        // joint_commands so a recorder sees the state/action pair at one time.
        // While the clock is not ready the stamped publishes are skipped (the
        // pairing publish below does not carry a stamp and continues).
        let stamp = match stamp_now() {
            Ok(stamp) => {
                clock_failing = false;
                Some(stamp)
            }
            Err(e) => {
                if !clock_failing {
                    clock_failing = true;
                    warn!("clock not ready, skipping stamped publishes: {e}");
                }
                None
            }
        };
        // Measured opening on the generic observer contract as a 1-DOF joint
        // (positions[0]). The v1.0 prismatic gripper senses neither velocity
        // nor grip force, so both are left empty rather than reported as zero.
        if let Some(stamp) = stamp {
            let observer_result =
                match joint_states::build_message(stamp, vec![opening], Vec::new(), Vec::new()) {
                    Ok(msg) => observer_pub.publish(msg).await.map_err(|e| e.to_string()),
                    Err(e) => Err(e.to_string()),
                };
            match observer_result {
                Ok(()) => observer_failing = false,
                Err(e) if !observer_failing => {
                    observer_failing = true;
                    warn!("joint_states publish failing, suppressing repeats: {e}");
                }
                Err(_) => {}
            }
        }
        // The opening setpoint commanded to this gripper, as the action a
        // recorder captures aligned with the measured opening. Held-last;
        // nothing published until the first command arrives.
        let latest_command = *tracked.borrow();
        if let (Some(stamp), Some(command)) = (stamp, latest_command) {
            let command_result = match joint_commands::build_message(stamp, vec![command.opening]) {
                Ok(msg) => command_pub.publish(msg).await.map_err(|e| e.to_string()),
                Err(e) => Err(e.to_string()),
            };
            match command_result {
                Ok(()) => command_failing = false,
                Err(e) if !command_failing => {
                    command_failing = true;
                    warn!("joint_commands publish failing, suppressing repeats: {e}");
                }
                Err(_) => {}
            }
        }
        // Measured opening to the paired backbone (the control loop's state input).
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
