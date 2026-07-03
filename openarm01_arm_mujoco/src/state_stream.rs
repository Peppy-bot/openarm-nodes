// Consume the sim's measured joint state (joint_states), cache the latest
// pose for this arm, and relay it to the paired commander on the pairing's
// `joint_states` topic (a legal no-op while unpaired). move_arm_joints anchors
// each trajectory on the cached pose and the follow loop re-anchors its chase
// on it; the paired commander sees this arm's state without any arm_id demux.

use std::sync::Arc;
use std::time::Instant;

use peppygen::NodeRunner;
use peppygen::consumed_topics::state_joint_states;
use peppygen::peers::commander::joint_states as peer_joint_states;
use peppylib::runtime::CancellationToken;
use tracing::error;

use crate::config::ArmId;
use crate::state::{JointStatesLatest, SharedState};

pub async fn run(
    runner: Arc<NodeRunner>,
    arm_id: ArmId,
    state: Arc<SharedState>,
    token: CancellationToken,
) {
    let mut subscription = match state_joint_states::subscribe(&runner).await {
        Ok(subscription) => subscription,
        Err(e) => {
            error!(error = %e, "joint_states subscribe");
            return;
        }
    };
    let peer_pub = match peer_joint_states::declare_publisher(&runner).await {
        Ok(p) => p,
        Err(e) => {
            error!(error = %e, "declare paired joint_states publisher");
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
                error!(error = %e, "joint_states receive");
                continue;
            }
        };
        if msg.arm_id != arm_id.raw() {
            continue;
        }
        // A non-finite position would corrupt the pose cache move_arm_joints
        // anchors on, so drop the whole sample rather than caching a bad pose.
        if !msg.positions.iter().all(|v| v.is_finite()) {
            continue;
        }
        {
            let mut latest = state.joint_states.lock().unwrap_or_else(|p| p.into_inner());
            *latest = Some(JointStatesLatest {
                positions: msg.positions.to_vec(),
                recv_at: Instant::now(),
            });
        }
        // Relay to the paired commander; silently dropped while unpaired.
        match peer_joint_states::build_message(msg.positions, msg.velocities) {
            Ok(payload) => {
                if let Err(e) = peer_pub.publish(payload).await {
                    error!(error = %e, "paired joint_states publish");
                }
            }
            Err(e) => error!(error = %e, "paired joint_states build"),
        }
    }
}
