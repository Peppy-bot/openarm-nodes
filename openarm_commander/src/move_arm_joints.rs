// Spawned per fire_arm command (the panel's Home/Ready parks, as discrete governed
// moves). Fires move_arm_joints at the backbone, then reports the outcome to the owner. Each
// goal is its own task; cancel-aware so a shutdown can't wedge an in-flight goal, and
// preempt-aware so a new move can cancel it.

use std::sync::Arc;
use std::time::Duration;

use peppygen::NodeRunner;
use peppygen::QoSProfile;
use peppygen::consumed_actions::backbone_move_arm_joints;
use peppygen::consumed_actions::backbone_move_arm_joints::ResultOutcome;
use peppylib::runtime::CancellationToken;
use tokio::sync::mpsc;
use tracing::{info, warn};

use crate::owner::{Feedback, PREEMPT_GRACE};
use crate::pose::REACHED_ANGLE_TOL_RAD;
use crate::state::{ARM_DOF, Side};

// Goal-accept round-trip to a pinned producer; answered directly, so this only needs to
// cover the decider, not a discovery probe.
const GOAL_TIMEOUT: Duration = Duration::from_secs(2);
const RESULT_TIMEOUT: Duration = Duration::from_secs(60);

#[allow(clippy::too_many_arguments)]
pub fn spawn(
    runner: Arc<NodeRunner>,
    feedback: mpsc::Sender<Feedback>,
    token: CancellationToken,
    preempt: tokio_util::sync::CancellationToken,
    side: Side,
    joint_positions: [f64; ARM_DOF],
    duration_s: f64,
    grace: bool,
) {
    tokio::spawn(async move {
        run(
            runner,
            feedback,
            token,
            preempt,
            side,
            joint_positions,
            duration_s,
            grace,
        )
        .await;
    });
}

#[allow(clippy::too_many_arguments)]
async fn run(
    runner: Arc<NodeRunner>,
    feedback: mpsc::Sender<Feedback>,
    token: CancellationToken,
    preempt: tokio_util::sync::CancellationToken,
    side: Side,
    joint_positions: [f64; ARM_DOF],
    duration_s: f64,
    grace: bool,
) {
    // A queued preempt fires only after the backbone releases its single-flight gate.
    if grace {
        tokio::select! {
            _ = token.cancelled() => return finalize(&feedback, side, false, "shutting down; move dropped").await,
            _ = tokio::time::sleep(PREEMPT_GRACE) => {}
        }
    }
    let label = side.label();
    info!(side = label, ?joint_positions, "fire move_arm_joints");

    let goal = backbone_move_arm_joints::GoalRequest {
        arm_id: side.arm_id(),
        joint_positions,
        duration_s,
    };

    // The launcher-pinned, cardinality-one backbone slot provides the explicit
    // target used for this goal and its feedback/cancel/result lifecycle.
    let downstream = match backbone_move_arm_joints::ActionHandle::fire_goal(
        &runner,
        backbone_move_arm_joints::bound_producer(&runner),
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
                format!("backbone rejected the goal: {reason}"),
            )
            .await;
            return;
        }
        Err(e) => {
            finalize(&feedback, side, false, format!("fire_goal failed: {e}")).await;
            return;
        }
    };

    // Await the move result, honoring preempt (a new move cancels this goal) and
    // shutdown. There is no feedback to drain: live progress is shown from the
    // arm_states stream (see joint_states.rs). v0.10 ResultResponse.outcome is a typed
    // enum (Completed/Cancelled/Abandoned/Expired).
    let result_fut = downstream.get_result(RESULT_TIMEOUT);
    tokio::pin!(result_fut);
    let mut preempted = false;
    let outcome = loop {
        tokio::select! {
            _ = token.cancelled() => {
                finalize(&feedback, side, false, "shutting down; result abandoned").await;
                return;
            }
            _ = preempt.cancelled(), if !preempted => {
                preempted = true;
                if let Err(e) = downstream.cancel_goal(GOAL_TIMEOUT).await {
                    warn!(side = side.label(), error = %e, "preempt cancel failed");
                }
            }
            result = &mut result_fut => break result,
        }
    };
    let (success, summary) = match outcome {
        Ok(r) => match r.outcome {
            ResultOutcome::Completed(data) => {
                if !data.success {
                    (
                        false,
                        format!("move_arm_joints ({label}) failed: {}", data.message),
                    )
                } else {
                    // Confirm the arm actually reached the commanded joints, not just
                    // that the trajectory finished (a governor stop finishes it too).
                    let max_err = (0..ARM_DOF)
                        .map(|i| (data.final_joint_positions[i] - joint_positions[i]).abs())
                        .fold(0.0_f64, f64::max);
                    if max_err <= REACHED_ANGLE_TOL_RAD {
                        (
                            true,
                            format!(
                                "move_arm_joints ({label}): success in {:.2}s",
                                data.action_time
                            ),
                        )
                    } else {
                        (
                            false,
                            format!(
                                "move_arm_joints ({label}) ended {:.1} deg off target (blocked?)",
                                max_err.to_degrees()
                            ),
                        )
                    }
                }
            }
            ResultOutcome::Cancelled(data) => (
                false,
                format!("move_arm_joints ({label}) cancelled: {}", data.message),
            ),
            ResultOutcome::Abandoned => (
                false,
                format!("move_arm_joints ({label}) abandoned by backbone"),
            ),
            ResultOutcome::Expired => (false, format!("move_arm_joints ({label}) result expired")),
        },
        Err(e) => (
            false,
            format!("move_arm_joints ({label}) result error: {e}"),
        ),
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
        info!(side = side.label(), %summary, "move_arm_joints done");
    } else {
        warn!(side = side.label(), %summary, "move_arm_joints done");
    }
    let _ = feedback.send(Feedback::ArmGoalDone { side, summary }).await;
}
