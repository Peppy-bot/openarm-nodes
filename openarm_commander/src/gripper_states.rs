// Live gripper opening for the UI. Consumes each side's always-on joint_states
// slot and reports the latest measured opening (positions[0]) to the owner. The
// slot binding fixes the side, so the producer identity is authoritative and no
// in-message id is read. Mirrors joint_states.rs for the arm: it runs
// continuously, so the panel shows live aperture whether or not a move is in
// flight.

use std::sync::Arc;

use peppygen::NodeRunner;
use peppygen::consumed_topics::{
    left_gripper_states_joint_states, right_gripper_states_joint_states,
};
use peppylib::runtime::CancellationToken;
use tokio::sync::mpsc;
use tracing::{error, warn};

use crate::joint_states::RECEIVE_ERROR_BACKOFF;
use crate::owner::Feedback;
use crate::state::Side;

pub async fn run(
    runner: Arc<NodeRunner>,
    feedback: mpsc::Sender<Feedback>,
    token: CancellationToken,
) {
    let mut left_subscription = match left_gripper_states_joint_states::subscribe(&runner).await {
        Ok(subscription) => subscription,
        Err(e) => {
            error!(error = %e, "left_gripper_states subscribe");
            return;
        }
    };
    let mut right_subscription = match right_gripper_states_joint_states::subscribe(&runner).await {
        Ok(subscription) => subscription,
        Err(e) => {
            error!(error = %e, "right_gripper_states subscribe");
            return;
        }
    };
    loop {
        let (slot, side, received) = tokio::select! {
            _ = token.cancelled() => return,
            received = left_subscription.next() => (
                "left_gripper_states",
                Side::Left,
                received.map(|pair| pair.map(|(_producer, msg)| msg.positions)),
            ),
            received = right_subscription.next() => (
                "right_gripper_states",
                Side::Right,
                received.map(|pair| pair.map(|(_producer, msg)| msg.positions)),
            ),
        };
        let positions = match received {
            Ok(Some(positions)) => positions,
            Ok(None) => {
                error!(
                    slot,
                    "joint_states subscription closed; live gripper readouts stopped"
                );
                return;
            }
            Err(e) => {
                error!(error = %e, slot, "joint_states receive");
                tokio::time::sleep(RECEIVE_ERROR_BACKOFF).await;
                continue;
            }
        };
        // The gripper reports its opening as a single-joint state (positions[0]);
        // an empty vector is a misbound slot, so skip it.
        let Some(&opening) = positions.first() else {
            warn!(slot, "joint_states: empty positions from gripper; ignoring");
            continue;
        };
        if feedback
            .send(Feedback::GripperMeasured { side, opening })
            .await
            .is_err()
        {
            return;
        }
    }
}
