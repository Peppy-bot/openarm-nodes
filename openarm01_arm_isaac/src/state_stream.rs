// Consume the sim's measured joint state (joint_states) and cache the latest
// pose for this arm. move_arm_joints anchors each trajectory on this pose and the
// follow loop re-anchors its chase on it. Replaces the old sim-bridge telemetry
// pipeline that re-emitted state; the sim now emits joint_states directly.

use std::sync::Arc;
use std::time::Instant;

use peppygen::NodeRunner;
use peppygen::consumed_topics::state_joint_states;
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
    loop {
        let received = tokio::select! {
            _ = token.cancelled() => return,
            received = state_joint_states::on_next_message_received(&runner) => received,
        };
        let (_producer, msg) = match received {
            Ok(pair) => pair,
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
        let mut latest = state.joint_states.lock().unwrap_or_else(|p| p.into_inner());
        *latest = Some(JointStatesLatest {
            positions: msg.positions.to_vec(),
            recv_at: Instant::now(),
        });
    }
}
