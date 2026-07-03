// Live arm joint state for the UI. Each side's pairing slot delivers the
// paired arm's `joint_states` — the slot IS the side, so there is no arm_id
// demux — and writes the latest measured positions into UiState. A slot with
// no paired arm just stays silent, so the panel boots with any subset of arms
// and shows live joint state whether or not a move is in flight.

use std::sync::Arc;

use peppygen::NodeRunner;
use peppygen::peers::left_arm::joint_states as left_joint_states;
use peppygen::peers::right_arm::joint_states as right_joint_states;
use peppylib::runtime::CancellationToken;
use tracing::error;

use crate::state::{SharedState, Side};

/// One receive loop per pairing slot. The generated modules are distinct
/// types with identical shapes, so a macro stamps out the per-slot body.
macro_rules! side_loop {
    ($module:ident, $side:expr, $runner:expr, $state:expr, $token:expr) => {{
        let runner = $runner.clone();
        let state = $state.clone();
        let token = $token.clone();
        tokio::spawn(async move {
            let mut subscription = match $module::subscribe(&runner).await {
                Ok(subscription) => subscription,
                Err(e) => {
                    error!(error = %e, side = $side.label(), "joint_states subscribe");
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
                        error!(error = %e, side = $side.label(), "joint_states receive");
                        continue;
                    }
                };
                let mut s = state.lock().unwrap_or_else(|p| p.into_inner());
                s.arm_mut($side).last_feedback = Some(msg.positions);
                // While disabled, hold the target on the measured pose so the
                // first streamed setpoint on enabling equals where the arm
                // already is.
                if !s.enabled($side) {
                    s.arm_mut($side).joints = msg.positions;
                }
            }
        })
    }};
}

pub async fn run(runner: Arc<NodeRunner>, state: SharedState, token: CancellationToken) {
    let left = side_loop!(left_joint_states, Side::Left, runner, state, token);
    let right = side_loop!(right_joint_states, Side::Right, runner, state, token);
    let _ = left.await;
    let _ = right.await;
}
