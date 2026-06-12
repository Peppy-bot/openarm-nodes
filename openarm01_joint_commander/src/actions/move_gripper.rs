// Spawned per o/c keypress when a gripper is focused. Same shape as
// move_arm_joints — fire at backbone, stream feedback, write result. The
// preempt token only fires from the shutdown hook in main.rs (a second fire
// while in flight is rejected, not preempted): it cancels the goal at
// backbone and the hook awaits the resulting finalize via in_flight.

use std::sync::Arc;
use std::time::Duration;

use peppygen::NodeRunner;
use peppygen::QoSProfile;
use peppygen::consumed_actions::backbone_move_gripper;
use peppygen::consumed_actions::backbone_move_gripper::ResultOutcome;
use tracing::{info, warn};

use crate::state::{SharedState, Side};

const GOAL_TIMEOUT: Duration = Duration::from_secs(5);
const RESULT_TIMEOUT: Duration = Duration::from_secs(60);

pub fn spawn(
    runner: Arc<NodeRunner>,
    state: SharedState,
    preempt: tokio_util::sync::CancellationToken,
    side: Side,
    position_m: f64,
    feedback_hz: u32,
) {
    tokio::spawn(async move {
        run(runner, state, preempt, side, position_m, feedback_hz).await;
    });
}

async fn run(
    runner: Arc<NodeRunner>,
    state: SharedState,
    preempt: tokio_util::sync::CancellationToken,
    side: Side,
    position_m: f64,
    feedback_hz: u32,
) {
    let label = side.label();

    // Shutdown can fire between the UI registering this goal as in-flight and
    // this task's first poll; the hook cancels the freshly-registered preempt,
    // so honour it here rather than firing a goal during shutdown.
    if preempt.is_cancelled() {
        finalize(&state, side, false, "shutdown before goal was fired").await;
        return;
    }

    info!(side = label, position_m, feedback_hz, "fire move_gripper");

    let goal = backbone_move_gripper::GoalRequest {
        gripper_id: side.gripper_id(),
        feedback_frequency: feedback_hz,
        position: position_m,
    };

    // v0.10 peppylib: fire_goal trims to (runner, timeout, request, qos). Instance
    // targeting moved from call-site args to launcher-pinned link_id bindings.
    let mut downstream = match backbone_move_gripper::ActionHandle::fire_goal(
        &runner,
        GOAL_TIMEOUT,
        goal,
        QoSProfile::SensorData,
    )
    .await
    {
        Ok(handle) if handle.data.accepted => handle,
        Ok(_) => {
            finalize(&state, side, false, "backbone rejected the goal").await;
            return;
        }
        Err(e) => {
            finalize(&state, side, false, format!("fire_goal failed: {e}")).await;
            return;
        }
    };

    let mut preempted = false;
    loop {
        tokio::select! {
            _ = preempt.cancelled(), if !preempted => {
                preempted = true;
                // At shutdown the wait is capped by the hook's remaining
                // budget so this task finishes inside the grace window
                // instead of being force-dropped mid-await at teardown.
                let timeout = super::bounded_by_shutdown(&state, GOAL_TIMEOUT);
                if let Err(e) = downstream.cancel_goal(timeout).await {
                    warn!(side = side.label(), error = %e, "preempt cancel failed");
                }
            }
            feedback = downstream.on_next_feedback_message() => match feedback {
                Ok(f) => {
                    let mut s = state.lock().unwrap_or_else(|p| p.into_inner());
                    s.gripper_mut(side).last_feedback = Some(f.joint_positions);
                }
                Err(_) => break,
            }
        }
    }

    // v0.10 ResultResponse.outcome is a typed enum (Completed/Cancelled/
    // Abandoned/Expired). Stay preempt-aware so a shutdown cancel arriving
    // during the up-to-RESULT_TIMEOUT wait still reaches the gripper and the
    // result lands through the normal path.
    let outcome = loop {
        tokio::select! {
            _ = preempt.cancelled(), if !preempted => {
                preempted = true;
                let timeout = super::bounded_by_shutdown(&state, GOAL_TIMEOUT);
                if let Err(e) = downstream.cancel_goal(timeout).await {
                    warn!(side = side.label(), error = %e, "preempt cancel failed");
                }
            }
            // Re-evaluated each pass: before shutdown this is the full
            // RESULT_TIMEOUT; after a shutdown preempt it shrinks to the
            // hook's remaining budget.
            result = downstream.get_result(super::bounded_by_shutdown(&state, RESULT_TIMEOUT)) => break result,
        }
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
    s.gripper_mut(side).in_flight = false;
    s.gripper_mut(side).preempt = None;
    s.set_status(summary.clone());
    if success {
        info!(side = side.label(), %summary, "move_gripper done");
    } else {
        warn!(side = side.label(), %summary, "move_gripper done");
    }
}
