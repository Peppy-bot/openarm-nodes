// Relay the sim's measured gripper opening for this gripper (demuxed from the
// engine's gripper_id-tagged broadcast) to three consumers, each independently
// so one failing never starves the others:
//   - the paired backbone, on the pairing's `gripper_states` (the control loop's
//     state input; a legal no-op while unpaired);
//   - any observer such as a recorder, on the generic joint_states contract (the
//     measured opening as positions[0]);
//   - the same observers, on the generic joint_commands contract (the opening
//     setpoint this gripper is tracking, held-last, so a recorder captures the
//     action aligned with the measured opening).
// The joint_commands stream lives here, not on the backbone that computes the
// setpoint: pairing traffic is unobservable by third parties, and the backbone
// is one node driving both grippers and cannot emit a per-gripper stream. A
// consumer binds this follower exactly like the real gripper.

use std::sync::Arc;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use peppygen::NodeRunner;
use peppygen::consumed_topics::engine_states_gripper_states;
use peppygen::emitted_topics::joint_commands::v1::joint_commands;
use peppygen::emitted_topics::joint_states::v1::joint_states;
use peppygen::pairings::backbone;
use peppylib::runtime::CancellationToken;
use tokio::sync::watch;
use tracing::{error, warn};

use crate::config::GripperId;
use crate::stream::GripperCommand;

/// The peppy-synchronized clock as a [`SystemTime`] stamp for the generic
/// emissions (sim time once the engine ticks the clock), or an error while the
/// clock is not ready.
fn stamp_now() -> Result<SystemTime, String> {
    let ns = peppygen::clock::now_ns().map_err(|e| e.to_string())?;
    Ok(UNIX_EPOCH + Duration::from_nanos(ns))
}

pub async fn run(
    runner: Arc<NodeRunner>,
    gripper_id: GripperId,
    tracked: watch::Receiver<Option<GripperCommand>>,
    token: CancellationToken,
) {
    let mut subscription = match engine_states_gripper_states::subscribe(&runner).await {
        Ok(subscription) => subscription,
        Err(e) => return error!(error = %e, "engine gripper_states subscribe"),
    };
    let observer_pub = match joint_states::declare_publisher(&runner).await {
        Ok(p) => p,
        Err(e) => return error!(error = %e, "declare joint_states publisher"),
    };
    let command_pub = match joint_commands::declare_publisher(&runner).await {
        Ok(p) => p,
        Err(e) => return error!(error = %e, "declare joint_commands publisher"),
    };
    let peer_pub = match backbone::gripper_states::declare_publisher(&runner).await {
        Ok(p) => p,
        Err(e) => return error!(error = %e, "declare paired gripper_states publisher"),
    };
    let mut clock_failing = false;
    let mut observer_failing = false;
    let mut command_failing = false;
    let mut peer_failing = false;
    loop {
        let received = tokio::select! {
            _ = token.cancelled() => return,
            received = subscription.next() => received,
        };
        let (_producer, msg) = match received {
            Ok(Some(pair)) => pair,
            Ok(None) => return,
            Err(e) => {
                error!(error = %e, "engine gripper_states receive");
                continue;
            }
        };
        // Drop samples for the other gripper or with a non-finite opening so no
        // consumer anchors on a bad measurement.
        if msg.gripper_id != gripper_id.as_u8() || !msg.opening.is_finite() {
            continue;
        }
        // One synchronized stamp per relayed sample (sim time), shared by
        // joint_states and joint_commands so a recorder sees the state/action
        // pair at one time. While the clock is not ready (no sim tick yet) the
        // stamped publishes are skipped; the pairing publish below does not
        // carry a stamp and continues.
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
        // (positions[0]); the gripper senses neither velocity nor force.
        if let Some(stamp) = stamp {
            let observer =
                match joint_states::build_message(stamp, vec![msg.opening], Vec::new(), Vec::new())
                {
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
        }
        // The opening setpoint commanded to this gripper, held-last.
        let latest_command = *tracked.borrow();
        if let (Some(stamp), Some(command)) = (stamp, latest_command) {
            let result = match joint_commands::build_message(stamp, vec![command.opening]) {
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
        let peer = match backbone::gripper_states::build_message(msg.opening) {
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
