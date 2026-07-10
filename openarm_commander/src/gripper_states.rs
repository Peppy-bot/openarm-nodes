// Live gripper opening for the UI. Consumes the always-on `gripper_states`
// stream from any gripper, demuxes by `gripper_id`, and writes the latest
// measured opening into UiState. Mirrors joint_states.rs for the arm: it runs
// continuously, so the panel shows live aperture whether or not a move is in
// flight.

use std::sync::Arc;

use peppygen::NodeRunner;
use peppygen::consumed_topics::gripper_states_gripper_states;
use peppylib::runtime::CancellationToken;
use tracing::{error, warn};

use crate::joint_states::RECEIVE_ERROR_BACKOFF;
use crate::state::{SharedState, Side};

pub async fn run(runner: Arc<NodeRunner>, state: SharedState, token: CancellationToken) {
    let mut subscription = match gripper_states_gripper_states::subscribe(&runner).await {
        Ok(subscription) => subscription,
        Err(e) => {
            error!(error = %e, "gripper_states subscribe");
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
                tokio::time::sleep(RECEIVE_ERROR_BACKOFF).await;
                continue;
            }
        };
        let Some(side) = Side::from_gripper_id(msg.gripper_id) else {
            warn!(
                gripper_id = msg.gripper_id,
                "gripper_states: unknown gripper_id; ignoring"
            );
            continue;
        };
        let mut s = state.lock().unwrap_or_else(|p| p.into_inner());
        s.grippers[side].last_feedback = Some(msg.position);
    }
}
