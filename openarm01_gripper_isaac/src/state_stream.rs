// Consume the sim's measured gripper opening (gripper_states), cache the
// latest sample for this gripper, and relay it to the paired commander on the
// pairing's `gripper_states` topic (a legal no-op while unpaired). The move
// action reads the cache on each feedback tick to compute convergence +
// stall; the paired commander sees this gripper's aperture without any
// gripper_id demux.

use std::sync::Arc;
use std::time::Instant;

use peppygen::NodeRunner;
use peppygen::consumed_topics::state_gripper_states;
use peppygen::peers::commander::gripper_states as peer_gripper_states;
use peppylib::runtime::CancellationToken;
use tracing::error;

use crate::config::GripperId;
use crate::state::{GripperStateLatest, SharedState};

pub async fn run(
    runner: Arc<NodeRunner>,
    gripper_id: GripperId,
    state: Arc<SharedState>,
    token: CancellationToken,
) {
    let mut subscription = match state_gripper_states::subscribe(&runner).await {
        Ok(subscription) => subscription,
        Err(e) => {
            error!(error = %e, "gripper_states subscribe");
            return;
        }
    };
    let peer_pub = match peer_gripper_states::declare_publisher(&runner).await {
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
        {
            let mut latest = state
                .gripper_state
                .lock()
                .unwrap_or_else(|p| p.into_inner());
            *latest = Some(GripperStateLatest {
                opening: msg.position,
                recv_at: Instant::now(),
            });
        }
        // Relay to the paired commander; silently dropped while unpaired.
        match peer_gripper_states::build_message(msg.position) {
            Ok(payload) => {
                if let Err(e) = peer_pub.publish(payload).await {
                    error!(error = %e, "paired gripper_states publish");
                }
            }
            Err(e) => error!(error = %e, "paired gripper_states build"),
        }
    }
}
