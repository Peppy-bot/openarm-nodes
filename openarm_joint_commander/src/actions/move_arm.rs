// Spawned per fire_arm_pose command (the panel's Execute-pose in Actions mode).
// Fires the hub's Cartesian move_arm (planned straight-line pose move), streams
// feedback back into the shared UiState, and writes the result into the status
// line. Each goal is its own task; cancel-aware so a shutdown can't wedge an
// in-flight goal.

use std::sync::Arc;
use std::time::Duration;

use peppygen::NodeRunner;
use peppygen::QoSProfile;
use peppygen::consumed_actions::backbone_move_arm;
use peppygen::consumed_actions::backbone_move_arm::ResultOutcome;
use peppylib::runtime::CancellationToken;
use tracing::{info, warn};

use crate::state::{SharedState, Side};

// Goal-accept round-trip to a pinned producer; answered directly, so this
// only needs to cover the decider, not a discovery probe.
const GOAL_TIMEOUT: Duration = Duration::from_secs(2);
const RESULT_TIMEOUT: Duration = Duration::from_secs(60);

#[allow(clippy::too_many_arguments)]
pub fn spawn(
    runner: Arc<NodeRunner>,
    state: SharedState,
    token: CancellationToken,
    preempt: tokio_util::sync::CancellationToken,
    side: Side,
    position: [f64; 3],
    orientation: [f64; 4],
    duration_s: f64,
) {
    tokio::spawn(async move {
        run(
            runner,
            state,
            token,
            preempt,
            side,
            position,
            orientation,
            duration_s,
        )
        .await;
    });
}

#[allow(clippy::too_many_arguments)]
async fn run(
    runner: Arc<NodeRunner>,
    state: SharedState,
    token: CancellationToken,
    preempt: tokio_util::sync::CancellationToken,
    side: Side,
    position: [f64; 3],
    orientation: [f64; 4],
    duration_s: f64,
) {
    let label = side.label();
    info!(side = label, ?position, ?orientation, "fire move_arm");

    let goal = backbone_move_arm::GoalRequest {
        arm_id: side.arm_id(),
        position,
        orientation,
        duration_s,
    };

    let downstream = match backbone_move_arm::ActionHandle::fire_goal(
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
                format!("backbone rejected the pose goal: {reason}"),
            )
            .await;
            return;
        }
        Err(e) => {
            finalize(&state, side, false, format!("fire_goal failed: {e}")).await;
            return;
        }
    };

    // Await the move result, honoring preempt (a new Execute cancels this goal)
    // and shutdown. Live progress is shown from the arm_states stream.
    let result_fut = downstream.get_result(RESULT_TIMEOUT);
    tokio::pin!(result_fut);
    let mut preempted = false;
    let outcome = loop {
        tokio::select! {
            _ = token.cancelled() => {
                finalize(&state, side, false, "shutting down; result abandoned").await;
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
                let msg = if data.success {
                    format!("move_arm ({}): success in {:.2}s", label, data.action_time)
                } else {
                    format!("move_arm ({}) failed: {}", label, data.message)
                };
                (data.success, msg)
            }
            ResultOutcome::Cancelled(data) => (
                false,
                format!("move_arm ({label}) cancelled: {}", data.message),
            ),
            ResultOutcome::Abandoned => {
                (false, format!("move_arm ({label}) abandoned by backbone"))
            }
            ResultOutcome::Expired => (false, format!("move_arm ({label}) result expired")),
        },
        Err(e) => (false, format!("move_arm ({label}) result error: {e}")),
    };
    finalize(&state, side, success, summary).await;
}

async fn finalize(state: &SharedState, side: Side, success: bool, summary: impl Into<String>) {
    let summary = summary.into();
    let mut s = state.lock().unwrap_or_else(|p| p.into_inner());
    s.arm_mut(side).in_flight = false;
    s.arm_mut(side).preempt = None;
    s.set_status(summary.clone());
    if success {
        info!(side = side.label(), %summary, "move_arm done");
    } else {
        warn!(side = side.label(), %summary, "move_arm done");
    }
}
