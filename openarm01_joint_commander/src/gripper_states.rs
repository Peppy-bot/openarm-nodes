// Live gripper opening for the UI. Each side's pairing slot delivers the
// paired gripper's `gripper_states` — the slot IS the side, so there is no
// gripper_id demux — and writes the latest measured opening into UiState.
// Mirrors joint_states.rs for the arm: a slot with no paired gripper stays
// silent, and the panel shows live state whether or not a move is in flight.

use std::sync::Arc;

use peppygen::NodeRunner;
use peppygen::peers::left_gripper::gripper_states as left_gripper_states;
use peppygen::peers::right_gripper::gripper_states as right_gripper_states;
use peppylib::runtime::CancellationToken;
use tracing::error;

use crate::state::{SharedState, Side};

/// One receive loop per pairing slot; a macro stamps out the per-slot body
/// (the generated modules are distinct types with identical shapes).
macro_rules! side_loop {
    ($module:ident, $side:expr, $runner:expr, $state:expr, $token:expr) => {{
        let runner = $runner.clone();
        let state = $state.clone();
        let token = $token.clone();
        tokio::spawn(async move {
            let mut subscription = match $module::subscribe(&runner).await {
                Ok(subscription) => subscription,
                Err(e) => {
                    error!(error = %e, side = $side.label(), "gripper_states subscribe");
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
                        error!(error = %e, side = $side.label(), "gripper_states receive");
                        continue;
                    }
                };
                let mut s = state.lock().unwrap_or_else(|p| p.into_inner());
                s.gripper_mut($side).last_feedback = Some(msg.position);
            }
        })
    }};
}

pub async fn run(runner: Arc<NodeRunner>, state: SharedState, token: CancellationToken) {
    let left = side_loop!(left_gripper_states, Side::Left, runner, state, token);
    let right = side_loop!(right_gripper_states, Side::Right, runner, state, token);
    let _ = left.await;
    let _ = right.await;
}
