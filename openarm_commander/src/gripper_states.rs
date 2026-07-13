// Live gripper opening for the UI. Consumes the always-on `gripper_states` stream from
// any gripper, demuxes by `gripper_id`, and reports the latest measured opening to the
// owner. Mirrors joint_states.rs for the arm: it runs continuously, so the panel shows
// live aperture whether or not a move is in flight.

use std::sync::Arc;

use peppygen::NodeRunner;
use peppygen::consumed_topics::{
    left_gripper_states_gripper_states, right_gripper_states_gripper_states,
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
        let (slot, received) = tokio::select! {
            _ = token.cancelled() => return,
            received = left_subscription.next() => (
                "left_gripper_states",
                received.map(|pair| pair.map(|(_producer, msg)| (msg.gripper_id, msg.position))),
            ),
            received = right_subscription.next() => (
                "right_gripper_states",
                received.map(|pair| pair.map(|(_producer, msg)| (msg.gripper_id, msg.position))),
            ),
        };
        let (gripper_id, position) = match received {
            Ok(Some(pair)) => pair,
            Ok(None) => return,
            Err(e) => {
                error!(error = %e, slot, "gripper_states receive");
                tokio::time::sleep(RECEIVE_ERROR_BACKOFF).await;
                continue;
            }
        };
        let Some(side) = Side::from_gripper_id(gripper_id) else {
            warn!(gripper_id, "gripper_states: unknown gripper_id; ignoring");
            continue;
        };
        if feedback
            .send(Feedback::GripperMeasured {
                side,
                opening: position,
            })
            .await
            .is_err()
        {
            return;
        }
    }
}
