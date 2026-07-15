// Live arm joint state for the UI. Consumes each side's always-on joint_states
// slot and reports the latest measured positions to the owner. The slot binding
// fixes the side, so the producer identity is authoritative and no in-message id
// is read. It runs continuously, so the panel shows live joint state whether or
// not a move is in flight; the owner decides how to fold each measurement into
// the target.

use std::sync::Arc;

use peppygen::NodeRunner;
use peppygen::consumed_topics::{left_arm_states_joint_states, right_arm_states_joint_states};
use peppylib::runtime::CancellationToken;
use tokio::sync::mpsc;
use tracing::{error, warn};

use crate::owner::Feedback;
use crate::state::{ARM_DOF, Side};

/// Pause after a receive error before retrying, so a persistently broken subscription
/// cannot spin a listener at full CPU or flood the log at the stream rate (shared with
/// the gripper-states listener).
pub(crate) const RECEIVE_ERROR_BACKOFF: std::time::Duration = std::time::Duration::from_millis(100);

pub async fn run(
    runner: Arc<NodeRunner>,
    feedback: mpsc::Sender<Feedback>,
    token: CancellationToken,
) {
    let mut left_subscription = match left_arm_states_joint_states::subscribe(&runner).await {
        Ok(subscription) => subscription,
        Err(e) => {
            error!(error = %e, "left_arm_states subscribe");
            return;
        }
    };
    let mut right_subscription = match right_arm_states_joint_states::subscribe(&runner).await {
        Ok(subscription) => subscription,
        Err(e) => {
            error!(error = %e, "right_arm_states subscribe");
            return;
        }
    };
    loop {
        let (slot, side, received) = tokio::select! {
            _ = token.cancelled() => return,
            received = left_subscription.next() => (
                "left_arm_states",
                Side::Left,
                received.map(|pair| pair.map(|(_producer, msg)| msg.positions)),
            ),
            received = right_subscription.next() => (
                "right_arm_states",
                Side::Right,
                received.map(|pair| pair.map(|(_producer, msg)| msg.positions)),
            ),
        };
        let positions = match received {
            Ok(Some(positions)) => positions,
            Ok(None) => {
                error!(
                    slot,
                    "joint_states subscription closed; live joint readouts stopped"
                );
                return;
            }
            Err(e) => {
                error!(error = %e, slot, "joint_states receive");
                tokio::time::sleep(RECEIVE_ERROR_BACKOFF).await;
                continue;
            }
        };
        // The arm producer reports exactly one position per joint; a different
        // length is a misbound slot, so skip it rather than truncate silently.
        let joints: [f64; ARM_DOF] = match positions.try_into() {
            Ok(joints) => joints,
            Err(positions) => {
                warn!(
                    slot,
                    got = positions.len(),
                    "joint_states: expected {ARM_DOF} arm joints; ignoring"
                );
                continue;
            }
        };
        if feedback
            .send(Feedback::ArmMeasured { side, joints })
            .await
            .is_err()
        {
            return; // the owner is gone; nothing left to report to
        }
    }
}
