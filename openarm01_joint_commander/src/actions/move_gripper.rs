// Spawned per o/c keypress when a gripper is focused. Same shape as
// move_arm_joints: fire at backbone, await the result, write it to the status
// line. Live progress comes from the gripper_states stream (gripper_states.rs).

use std::sync::Arc;
use std::time::Duration;

use peppygen::NodeRunner;
use peppygen::QoSProfile;
use peppygen::consumed_actions::backbone_move_gripper;
use peppygen::consumed_actions::backbone_move_gripper::ResultOutcome;
use peppylib::runtime::CancellationToken;
use tracing::{info, warn};

use crate::state::{SharedState, Side};

// Goal-accept round-trip to a pinned producer — answered directly, so this
// only needs to cover the decider, not a discovery probe.
const GOAL_TIMEOUT: Duration = Duration::from_secs(2);
const RESULT_TIMEOUT: Duration = Duration::from_secs(60);

pub fn spawn(
    runner: Arc<NodeRunner>,
    state: SharedState,
    token: CancellationToken,
    side: Side,
    position_m: f64,
) {
    tokio::spawn(async move {
        run(runner, state, token, side, position_m).await;
    });
}

async fn run(
    runner: Arc<NodeRunner>,
    state: SharedState,
    token: CancellationToken,
    side: Side,
    position_m: f64,
) {
    let label = side.label();
    info!(side = label, position_m, "fire move_gripper");

    let goal = backbone_move_gripper::GoalRequest {
        gripper_id: side.gripper_id(),
        position: position_m,
    };

    // v0.10 peppylib: fire_goal trims to (runner, timeout, request, qos). Instance
    // targeting moved from call-site args to launcher-pinned link_id bindings.
    let downstream = match backbone_move_gripper::ActionHandle::fire_goal(
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

    // Await the move result, honoring shutdown. There is no feedback to drain:
    // live progress is shown from the gripper_states stream (see gripper_states.rs).
    // v0.10 ResultResponse.outcome is a typed enum (Completed/Cancelled/
    // Abandoned/Expired); the select keeps a shutdown during the up-to-
    // RESULT_TIMEOUT wait from wedging the task.
    let outcome = tokio::select! {
        _ = token.cancelled() => {
            finalize(&state, side, false, "shutting down — result abandoned").await;
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
    s.gripper_mut(side).in_flight = false;
    s.set_status(summary.clone());
    if success {
        info!(side = side.label(), %summary, "move_gripper done");
    } else {
        warn!(side = side.label(), %summary, "move_gripper done");
    }
}
