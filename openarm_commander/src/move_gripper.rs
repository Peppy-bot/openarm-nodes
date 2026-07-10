// Spawned per fire_gripper command (the gripper card's Execute in Actions mode).
// Fires the hub's move_gripper (a discrete governed open/close) and writes the
// result into the status line. Cancel-aware so a shutdown can't wedge an in-flight
// goal. A second Execute is refused while one is in flight (see fire_gripper), so
// this needs no per-goal preempt the way the longer arm moves do.

use std::sync::Arc;
use std::time::Duration;

use peppygen::NodeRunner;
use peppygen::QoSProfile;
use peppygen::consumed_actions::backbone_move_gripper;
use peppygen::consumed_actions::backbone_move_gripper::ResultOutcome;
use peppylib::runtime::CancellationToken;
use tracing::{info, warn};

use crate::state::{SharedState, Side};

// Goal-accept round-trip to a pinned producer; the gripper move itself is short.
const GOAL_TIMEOUT: Duration = Duration::from_secs(2);
const RESULT_TIMEOUT: Duration = Duration::from_secs(30);

pub fn spawn(
    runner: Arc<NodeRunner>,
    state: SharedState,
    token: CancellationToken,
    side: Side,
    position: f64,
) {
    tokio::spawn(async move {
        run(runner, state, token, side, position).await;
    });
}

async fn run(
    runner: Arc<NodeRunner>,
    state: SharedState,
    token: CancellationToken,
    side: Side,
    position: f64,
) {
    let label = side.label();
    info!(side = label, position, "fire move_gripper");

    let goal = backbone_move_gripper::GoalRequest {
        gripper_id: side.gripper_id(),
        position,
    };

    let downstream = match backbone_move_gripper::ActionHandle::fire_goal(
        &runner,
        GOAL_TIMEOUT,
        goal,
        QoSProfile::SensorData,
    )
    .await
    {
        Ok(handle) if handle.data.accepted => handle,
        Ok(handle) => {
            let reason = handle
                .data
                .error_message
                .unwrap_or_else(|| "no reason given".into());
            finalize(
                &state,
                side,
                false,
                format!("backbone rejected the gripper goal: {reason}"),
            )
            .await;
            return;
        }
        Err(e) => {
            finalize(&state, side, false, format!("fire_goal failed: {e}")).await;
            return;
        }
    };

    // Await the result, honoring shutdown. A rejected concurrent goal cannot happen
    // here: fire_gripper refuses a second Execute while one is in flight, so unlike
    // the arm moves there is no preempt branch and thus no loop.
    let outcome = tokio::select! {
        _ = token.cancelled() => {
            finalize(&state, side, false, "shutting down; result abandoned").await;
            return;
        }
        result = downstream.get_result(RESULT_TIMEOUT) => result,
    };
    let (success, summary) = match outcome {
        Ok(r) => match r.outcome {
            ResultOutcome::Completed(data) => {
                let msg = if data.success {
                    format!(
                        "move_gripper ({}): success in {:.2}s",
                        label, data.action_time
                    )
                } else {
                    format!("move_gripper ({}) failed: {}", label, data.message)
                };
                (data.success, msg)
            }
            ResultOutcome::Cancelled(data) => (
                false,
                format!("move_gripper ({label}) cancelled: {}", data.message),
            ),
            ResultOutcome::Abandoned => (
                false,
                format!("move_gripper ({label}) abandoned by backbone"),
            ),
            ResultOutcome::Expired => (false, format!("move_gripper ({label}) result expired")),
        },
        Err(e) => (false, format!("move_gripper ({label}) result error: {e}")),
    };
    finalize(&state, side, success, summary).await;
}

async fn finalize(state: &SharedState, side: Side, success: bool, summary: impl Into<String>) {
    let summary = summary.into();
    let mut s = state.lock().unwrap_or_else(|p| p.into_inner());
    s.grippers[side].in_flight = false;
    s.set_status(summary.clone());
    if success {
        info!(side = side.label(), %summary, "move_gripper done");
    } else {
        warn!(side = side.label(), %summary, "move_gripper done");
    }
}
