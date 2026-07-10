// Live arm joint state for the UI. Consumes the always-on `arm_states` stream from any
// arm, demuxes by `arm_id`, and reports the latest measured positions to the owner. It
// runs continuously, so the panel shows live joint state whether or not a move is in
// flight; the owner decides how to fold each measurement into the target.

use std::sync::Arc;

use peppygen::NodeRunner;
use peppygen::consumed_topics::arm_states_arm_states;
use peppylib::runtime::CancellationToken;
use tokio::sync::mpsc;
use tracing::{error, warn};

use crate::owner::Feedback;
use crate::state::Side;

/// Pause after a receive error before retrying, so a persistently broken subscription
/// cannot spin a listener at full CPU or flood the log at the stream rate (shared with
/// the gripper-states listener).
pub(crate) const RECEIVE_ERROR_BACKOFF: std::time::Duration = std::time::Duration::from_millis(100);

pub async fn run(
    runner: Arc<NodeRunner>,
    feedback: mpsc::Sender<Feedback>,
    token: CancellationToken,
) {
    let mut subscription = match arm_states_arm_states::subscribe(&runner).await {
        Ok(subscription) => subscription,
        Err(e) => {
            error!(error = %e, "arm_states subscribe");
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
                error!(error = %e, "arm_states receive");
                tokio::time::sleep(RECEIVE_ERROR_BACKOFF).await;
                continue;
            }
        };
        let Some(side) = Side::from_arm_id(msg.arm_id) else {
            warn!(arm_id = msg.arm_id, "arm_states: unknown arm_id; ignoring");
            continue;
        };
        if feedback
            .send(Feedback::ArmMeasured {
                side,
                joints: msg.positions,
            })
            .await
            .is_err()
        {
            return; // the owner is gone; nothing left to report to
        }
    }
}
