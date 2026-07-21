// Live gripper opening for the UI. Consumes each side's always-on `gripper_states`
// slot and reports the latest measured opening to the owner. The slot fixes the
// side; a message whose `gripper_id` disagrees with its slot is rejected. Mirrors
// joint_states.rs for the arm: it runs continuously, so the panel shows live
// aperture whether or not a move is in flight.

use std::sync::Arc;

use peppygen::NodeRunner;
use peppygen::consumed_topics::left_gripper_states::gripper_states as left_gripper_states_gripper_states;
use peppygen::consumed_topics::right_gripper_states::gripper_states as right_gripper_states_gripper_states;
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
    let mut left_subscription = match left_gripper_states_gripper_states::subscribe(&runner).await {
        Ok(subscription) => subscription,
        Err(e) => {
            error!(error = %e, "left_gripper_states subscribe");
            return;
        }
    };
    let mut right_subscription = match right_gripper_states_gripper_states::subscribe(&runner).await
    {
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
                received.map(|pair| pair.map(|(_producer, msg)| (msg.gripper_id, msg.opening))),
            ),
            received = right_subscription.next() => (
                "right_gripper_states",
                Side::Right,
                received.map(|pair| pair.map(|(_producer, msg)| (msg.gripper_id, msg.opening))),
            ),
        };
        let (gripper_id, opening) = match received {
            Ok(Some(pair)) => pair,
            Ok(None) => {
                error!(
                    slot,
                    "gripper_states subscription closed; live gripper readouts stopped"
                );
                return;
            }
            Err(e) => {
                error!(error = %e, slot, "gripper_states receive");
                tokio::time::sleep(RECEIVE_ERROR_BACKOFF).await;
                continue;
            }
        };
        if gripper_id != side.gripper_id() {
            warn!(
                gripper_id,
                slot, "gripper_states: gripper_id does not match its slot; ignoring"
            );
            continue;
        }
        if feedback
            .send(Feedback::GripperMeasured { side, opening })
            .await
            .is_err()
        {
            return;
        }
    }
}
