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
        // Measured opening on the generic observer contract as a 1-DOF joint
        // (positions[0]); the gripper senses neither velocity nor force.
        let observer = match joint_states::build_message(vec![msg.opening], Vec::new(), Vec::new())
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
