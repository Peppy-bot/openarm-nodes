// Spawned per fire_gripper command (the gripper card's Execute in Actions mode). Fires
// the backbone's move_gripper (a discrete governed open/close), then reports the outcome to
// the owner. Cancel-aware so a shutdown can't wedge an in-flight goal. A second Execute
// is refused while one is in flight (the owner gates it), so this needs no per-goal
// preempt the way the longer arm moves do.

use std::sync::Arc;
use std::time::Duration;

use peppygen::NodeRunner;
use peppygen::QoSProfile;
use peppygen::consumed_actions::backbone::move_gripper as backbone_move_gripper;
use peppygen::consumed_actions::backbone::move_gripper::ResultOutcome;
use peppylib::runtime::CancellationToken;
use tokio::sync::mpsc;
use tracing::{info, warn};

use crate::owner::Feedback;
use crate::result_wait::{RESULT_POLL, RESULT_RETRY_DELAY, result_poll_retryable};
use crate::state::Side;

// Goal-accept round-trip to a pinned producer; answered directly, so this only needs to
// cover the decider, not a discovery probe.
const GOAL_TIMEOUT: Duration = Duration::from_secs(2);

pub fn spawn(
    runner: Arc<NodeRunner>,
    feedback: mpsc::Sender<Feedback>,
    token: CancellationToken,
    side: Side,
    opening: f64,
) {
    tokio::spawn(async move {
        run(runner, feedback, token, side, opening).await;
    });
}

async fn run(
    runner: Arc<NodeRunner>,
    feedback: mpsc::Sender<Feedback>,
    token: CancellationToken,
    side: Side,
    opening: f64,
) {
    let label = side.label();
    info!(side = label, opening, "fire move_gripper");

    let goal = backbone_move_gripper::GoalRequest {
        gripper_id: side.gripper_id(),
        opening,
    };

    let downstream = match backbone_move_gripper::ActionHandle::fire_goal(
        &runner,
        backbone_move_gripper::bound_producer(&runner),
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
                &feedback,
                side,
                false,
                format!("backbone rejected the gripper goal: {reason}"),
            )
            .await;
            return;
        }
        Err(e) => {
            finalize(&feedback, side, false, format!("fire_goal failed: {e}")).await;
            return;
        }
    };

    // Await the result, honoring shutdown. A rejected concurrent goal cannot happen
    // here: the owner refuses a second Execute while one is in flight, so unlike the
    // arm moves there is no preempt branch. The wait re-arms its bounded poll until
    // a terminal outcome (see result_wait); the backbone's own deadline bounds the
    // move itself.
    let outcome = loop {
        let result_fut = downstream.get_result(RESULT_POLL);
        tokio::pin!(result_fut);
        tokio::select! {
            _ = token.cancelled() => {
                // Best-effort cancel so shutdown leaves no unsupervised motion.
                if let Err(e) = downstream.cancel_goal(GOAL_TIMEOUT).await {
                    warn!(side = side.label(), error = %e, "shutdown cancel failed");
                }
                finalize(&feedback, side, false, "shutting down; move cancelled").await;
                return;
            }
            result = &mut result_fut => match result {
                Err(e) if result_poll_retryable(&e) => {
                    tokio::time::sleep(RESULT_RETRY_DELAY).await;
                }
                result => break result,
            }
        }
    };
    // A result error leaves the goal's fate unknown; cancel best-effort so a move
    // somehow still running does not continue unsupervised.
    if outcome.is_err()
        && let Err(e) = downstream.cancel_goal(GOAL_TIMEOUT).await
    {
        warn!(side = side.label(), error = %e, "cancel after result error failed");
    }
    let (success, summary) = match outcome {
        Ok(r) => match r.outcome {
            ResultOutcome::Completed(data) => {
                // The backbone reports the command delivered and where the jaws
                // measured; whether that opening is good (a grasp stops short of
                // a full close on purpose) is this side's call to surface.
                let msg = if data.success {
                    format!(
                        "move_gripper ({}): success in {:.2}s, opening {:.2}",
                        label, data.action_time, data.final_opening
                    )
                } else {
                    format!(
                        "move_gripper ({}) failed at opening {:.2}: {}",
                        label, data.final_opening, data.message
                    )
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
    finalize(&feedback, side, success, summary).await;
}

// Report the move outcome to the owner, which clears the in-flight slot and writes the
// status line; a dropped channel means the owner is gone (shutdown), so ignore it.
async fn finalize(
    feedback: &mpsc::Sender<Feedback>,
    side: Side,
    success: bool,
    summary: impl Into<String>,
) {
    let summary = summary.into();
    if success {
        info!(side = side.label(), %summary, "move_gripper done");
    } else {
        warn!(side = side.label(), %summary, "move_gripper done");
    }
    let _ = feedback
        .send(Feedback::GripperGoalDone { side, summary })
        .await;
}
