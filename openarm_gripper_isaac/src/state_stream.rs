// Consume the sim's measured gripper opening (the engine's gripper_states) for
// this gripper and relay it to the paired backbone on the pairing's
// `gripper_states` topic (a legal no-op while unpaired), so the backbone sees
// this gripper's aperture without any gripper_id demux; monitors observe the
// pairing exactly like the real gripper's.

use std::sync::Arc;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use peppygen::NodeRunner;
use peppygen::consumed_topics::engine_states::gripper_states as engine_states_gripper_states;
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
        // Relay to the paired backbone; silently dropped while unpaired. The
        // engine's sim torque rides along as the pairing effort; the sim
        // applies no effort cap, so the ceiling is 0 (no effort control).
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
