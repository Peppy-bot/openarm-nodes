// Consume the sim's measured gripper opening (gripper_states) for this
// gripper, relay it to the paired backbone on the pairing's `gripper_states`
// topic (a legal no-op while unpaired), and re-emit it on this follower's
// per-side gripper_states, so the backbone sees this gripper's aperture
// without any gripper_id demux and monitors bind the follower exactly like
// the real gripper.

use std::sync::Arc;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use peppygen::NodeRunner;
use peppygen::consumed_topics::engine_states::gripper_states as engine_states_gripper_states;
use peppygen::emitted_topics::states::gripper_states;
use peppygen::paired_topics::backbone;
use peppylib::runtime::CancellationToken;
use tracing::error;

use crate::config::GripperId;

/// Pairing stamp from the daemon-resolved clock (sim time under a simulated
/// clock), so consumers age samples on the same timeline they read. Errors
/// until the clock delivers its first tick.
fn pairing_stamp() -> Result<SystemTime, String> {
    let ns = peppygen::clock::now_ns().map_err(|e| format!("clock not ready: {e}"))?;
    Ok(UNIX_EPOCH + Duration::from_nanos(ns))
}

pub async fn run(runner: Arc<NodeRunner>, gripper_id: GripperId, token: CancellationToken) {
    let mut subscription = match engine_states_gripper_states::subscribe(&runner).await {
        Ok(subscription) => subscription,
        Err(e) => {
            error!(error = %e, "gripper_states subscribe");
            return;
        }
    };
    let peer_pub = match backbone::gripper_states::declare_publisher(&runner).await {
        Ok(p) => p,
        Err(e) => {
            error!(error = %e, "declare paired gripper_states publisher");
            return;
        }
    };
    let states_pub = match gripper_states::declare_publisher(&runner).await {
        Ok(p) => p,
        Err(e) => {
            error!(error = %e, "declare gripper_states publisher");
            return;
        }
    };
    loop {
        let received = tokio::select! {
            _ = token.cancelled() => return,
            received = subscription.next() => received,
        };
        let (_producer, msg) = match received {
            Ok(Some(pair)) => pair,
            Ok(None) => return,
            Err(e) => {
                error!(error = %e, "gripper_states receive");
                continue;
            }
        };
        if msg.gripper_id != gripper_id.as_u8()
            || !msg.opening.is_finite()
            || !msg.force.is_finite()
        {
            continue;
        }
        // Re-emit on the per-side broadcast for launch-bound monitors. The sim
        // gripper applies no effort cap, so the ceiling is 0 (no effort control).
        match gripper_states::build_message(msg.gripper_id, msg.opening, msg.force, 0.0) {
            Ok(payload) => {
                if let Err(e) = states_pub.publish(payload).await {
                    error!(error = %e, "gripper_states publish");
                }
            }
            Err(e) => error!(error = %e, "gripper_states build"),
        }
        // Relay to the paired backbone; silently dropped while unpaired. The
        // engine's sim torque rides along as the pairing effort.
        match pairing_stamp().and_then(|stamp| {
            backbone::gripper_states::build_message(stamp, msg.opening, msg.force, 0.0)
                .map_err(|e| e.to_string())
        }) {
            Ok(payload) => {
                if let Err(e) = peer_pub.publish(payload).await {
                    error!(error = %e, "paired gripper_states publish");
                }
            }
            Err(e) => error!(error = %e, "paired gripper_states build"),
        }
    }
}
