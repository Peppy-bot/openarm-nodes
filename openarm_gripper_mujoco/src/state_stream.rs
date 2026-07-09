// Consume the sim's measured gripper opening (gripper_states) for this
// gripper and relay it to the paired hub on the pairing's
// `gripper_states` topic (a legal no-op while unpaired), so the hub
// sees this gripper's aperture without any gripper_id demux.

use std::sync::Arc;

use peppygen::NodeRunner;
use peppygen::consumed_topics::state_gripper_states;
use peppygen::pairings::hub;
use peppylib::runtime::CancellationToken;
use tracing::error;

use crate::config::GripperId;

pub async fn run(runner: Arc<NodeRunner>, gripper_id: GripperId, token: CancellationToken) {
    let mut subscription = match state_gripper_states::subscribe(&runner).await {
        Ok(subscription) => subscription,
        Err(e) => {
            error!(error = %e, "gripper_states subscribe");
            return;
        }
    };
    let peer_pub = match hub::gripper_states::declare_publisher(&runner).await {
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
        if msg.gripper_id != gripper_id.as_u8() || !msg.position.is_finite() {
            continue;
        }
        // Relay to the paired hub; silently dropped while unpaired.
        match hub::gripper_states::build_message(msg.position) {
            Ok(payload) => {
                if let Err(e) = peer_pub.publish(payload).await {
                    error!(error = %e, "paired gripper_states publish");
                }
            }
            Err(e) => error!(error = %e, "paired gripper_states build"),
        }
    }
}
