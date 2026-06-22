// Live arm joint state for the UI. Consumes the always-on `joint_states` stream
// from any arm, demuxes by `arm_id`, and writes the latest measured positions
// into UiState. This replaces reading move progress off the backbone's action
// feedback: it runs continuously, so the panel shows live joint state whether or
// not a move is in flight, and the move handler no longer needs the feedback.

use std::sync::Arc;

use peppygen::NodeRunner;
use peppygen::consumed_topics::arm_states_joint_states;
use peppylib::runtime::CancellationToken;
use tracing::{error, warn};

use crate::state::{SharedState, Side};

pub async fn run(runner: Arc<NodeRunner>, state: SharedState, token: CancellationToken) {
    loop {
        let received = tokio::select! {
            _ = token.cancelled() => return,
            received = arm_states_joint_states::on_next_message_received(&runner) => received,
        };
        let (_producer, msg) = match received {
            Ok(pair) => pair,
            Err(e) => {
                error!(error = %e, "joint_states receive");
                continue;
            }
        };
        let Some(side) = Side::from_arm_id(msg.arm_id) else {
            warn!(arm_id = msg.arm_id, "joint_states: unknown arm_id; ignoring");
            continue;
        };
        let mut s = state.lock().unwrap_or_else(|p| p.into_inner());
        s.arm_mut(side).last_feedback = Some(msg.positions);
        // While disabled, hold the target on the measured pose so the first
        // streamed setpoint on enabling equals where the arm already is.
        if !s.arm(side).enabled {
            s.arm_mut(side).joints = msg.positions;
        }
    }
}
