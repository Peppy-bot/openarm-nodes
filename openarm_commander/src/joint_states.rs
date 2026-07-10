// Live arm joint state for the UI. Consumes the always-on `arm_states` stream
// from any arm, demuxes by `arm_id`, and writes the latest measured positions
// into UiState. This replaces reading move progress off the backbone's action
// feedback: it runs continuously, so the panel shows live joint state whether or
// not a move is in flight, and the move handler no longer needs the feedback.

use std::sync::Arc;

use peppygen::NodeRunner;
use peppygen::consumed_topics::arm_states_arm_states;
use peppylib::runtime::CancellationToken;
use tracing::{error, warn};

use crate::state::{SharedState, Side};

/// Pause after a receive error before retrying, so a persistently broken
/// subscription cannot spin a listener at full CPU or flood the log at the
/// stream rate (shared with the gripper-states listener).
pub(crate) const RECEIVE_ERROR_BACKOFF: std::time::Duration = std::time::Duration::from_millis(100);

pub async fn run(runner: Arc<NodeRunner>, state: SharedState, token: CancellationToken) {
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
        let mut s = state.lock().unwrap_or_else(|p| p.into_inner());
        s.arm_mut(side).last_feedback = Some(msg.positions);
        // Initialize the streamed target from the first measured pose so the panel
        // starts at the arm's real position, then leave it: only streaming and
        // discrete moves change it thereafter. Tracking measured continuously while
        // disabled re-seeded the gravity-sagged pose every cycle, so each enable
        // ratcheted the arm further down.
        if !s.arm(side).established {
            s.arm_mut(side).joints = msg.positions;
            s.arm_mut(side).established = true;
        }
    }
}
