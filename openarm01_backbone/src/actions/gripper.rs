use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::Duration;

use peppygen::NodeRunner;
use peppygen::QoSProfile;
use peppygen::consumed_actions::{left_gripper_move_gripper, right_gripper_move_gripper};
use peppygen::exposed_actions::move_gripper::{
    ActionHandle, GoalContext, GoalRequest, GoalResponse,
};
use peppylib::runtime::CancellationToken;
use tracing::{info, warn};

// Goal-accept (and cancel) round-trip to a pinned producer, answered directly, so
// this only needs to cover the decider, not a discovery probe.
const GOAL_TIMEOUT: Duration = Duration::from_secs(2);
const RESULT_TIMEOUT: Duration = Duration::from_secs(60);

struct Outcome {
    success: bool,
    message: String,
    final_joint_positions: Vec<f64>,
    action_time: f64,
    is_cancelled: bool,
}

impl Outcome {
    fn failed(message: impl Into<String>) -> Self {
        Self {
            success: false,
            message: message.into(),
            final_joint_positions: Vec::new(),
            action_time: 0.0,
            is_cancelled: false,
        }
    }

    fn cancelled(message: impl Into<String>) -> Self {
        Self {
            success: false,
            message: message.into(),
            final_joint_positions: Vec::new(),
            action_time: 0.0,
            is_cancelled: true,
        }
    }
}

/// Counts one in-flight relay for its lifetime: increments on creation, and
/// decrements + wakes the shutdown waiter on drop. Held by the spawned relay task
/// so even a panic in the relay frees the count instead of wedging teardown.
struct InflightGuard {
    count: Arc<AtomicUsize>,
    idle: Arc<tokio::sync::Notify>,
}

impl InflightGuard {
    fn new(count: Arc<AtomicUsize>, idle: Arc<tokio::sync::Notify>) -> Self {
        count.fetch_add(1, Ordering::AcqRel);
        Self { count, idle }
    }
}

impl Drop for InflightGuard {
    fn drop(&mut self) {
        self.count.fetch_sub(1, Ordering::AcqRel);
        self.idle.notify_waiters();
    }
}

// Expose move_gripper. For each accepted goal, dispatch on gripper_id to the
// left or right consumed action, relay feedback upward, propagate cancel
// downward, and answer the caller with either complete or complete_cancelled.
pub async fn run(runner: Arc<NodeRunner>, token: CancellationToken) -> peppygen::Result<()> {
    let mut handle = ActionHandle::expose(&runner).await?;

    // Track in-flight relays so a shutdown holds teardown until each goal has
    // delivered its terminal result upstream, bounded by the grace window.
    let inflight = Arc::new(AtomicUsize::new(0));
    let idle = Arc::new(tokio::sync::Notify::new());
    {
        let inflight = inflight.clone();
        let idle = idle.clone();
        runner.on_shutdown(async move {
            while inflight.load(Ordering::Acquire) > 0 {
                let notified = idle.notified();
                tokio::pin!(notified);
                notified.as_mut().enable();
                if inflight.load(Ordering::Acquire) == 0 {
                    break;
                }
                notified.await;
            }
        });
    }

    loop {
        let goal_ctx = tokio::select! {
            _ = token.cancelled() => break,
            result = handle.handle_goal_next_request(|req: &GoalRequest| {
                match req.data.gripper_id {
                    0 | 1 => Ok(GoalResponse::accept()),
                    other => Ok(GoalResponse::reject(format!(
                        "invalid gripper_id {other} (expected 0 for left, 1 for right)"
                    ))),
                }
            }) => match result {
                Ok(Some(ctx)) => ctx,
                Ok(None) => break,
                Err(e) => {
                    warn!(error = %e, "move_gripper: goal handling failed");
                    continue;
                }
            },
        };

        // Spawn per goal so the accept loop returns immediately to receive the
        // next goal. Otherwise left+right move_gripper serialise through one
        // task and the second goal can't be received until the first finishes.
        let runner_for_goal = Arc::clone(&runner);
        let token_for_goal = token.clone();
        let inflight_guard = InflightGuard::new(inflight.clone(), idle.clone());
        tokio::spawn(async move {
            let _inflight_guard = inflight_guard; // decrements + notifies on task end (or panic)
            let outcome = forward(&runner_for_goal, &goal_ctx, &token_for_goal).await;

            let reply = if outcome.is_cancelled {
                goal_ctx
                    .complete_cancelled(
                        outcome.success,
                        outcome.message,
                        outcome.final_joint_positions,
                        outcome.action_time,
                    )
                    .await
            } else {
                goal_ctx
                    .complete(
                        outcome.success,
                        outcome.message,
                        outcome.final_joint_positions,
                        outcome.action_time,
                    )
                    .await
            };
            if let Err(e) = reply {
                warn!(error = %e, "move_gripper: complete failed");
            }
        });
    }
    Ok(())
}

async fn forward(
    runner: &NodeRunner,
    goal_ctx: &GoalContext,
    token: &CancellationToken,
) -> Outcome {
    let req_data = goal_ctx.request().data.clone();
    match req_data.gripper_id {
        0 => dispatch_left(runner, &req_data, goal_ctx, token).await,
        1 => dispatch_right(runner, &req_data, goal_ctx, token).await,
        // Unreachable: handle_goal_next_request rejects anything outside [0, 1].
        _ => Outcome::failed("gripper_id out of range"),
    }
}

// The left and right relays are identical but for the generated consumed-action
// module they fire into (distinct types per side). This macro defines one
// dispatch+relay per side over that module, so the relay logic lives once: fire
// the downstream goal, then await its result while propagating cancel/shutdown
// downward. Progress is observed on the gripper_states stream (consumed by the
// commander), so there is no feedback to relay.
macro_rules! gripper_dispatch {
    ($name:ident, $link:path, $side:literal) => {
        async fn $name(
            runner: &NodeRunner,
            req: &peppygen::exposed_actions::move_gripper::GoalRequestData,
            goal_ctx: &GoalContext,
            token: &CancellationToken,
        ) -> Outcome {
            use $link as link;
            let downstream = match link::ActionHandle::fire_goal(
                runner,
                GOAL_TIMEOUT,
                link::GoalRequest { position: req.position },
                QoSProfile::SensorData,
            )
            .await
            {
                Ok(handle) if handle.data.accepted => handle,
                Ok(handle) => {
                    return Outcome::failed(format!(
                        "{} gripper rejected goal: {}",
                        $side,
                        handle.data.error_message.unwrap_or_else(|| "no reason given".into())
                    ));
                }
                Err(e) => return Outcome::failed(format!("fire_goal to {} gripper failed: {e}", $side)),
            };
            info!("move_gripper: forwarded to {} gripper", $side);

            let mut upstream_cancelled = false;
            let result_fut = downstream.get_result(RESULT_TIMEOUT);
            tokio::pin!(result_fut);
            let result = loop {
                tokio::select! {
                    _ = token.cancelled() => {
                        // Propagate shutdown to the in-flight goal so it stops
                        // driving fingers instead of running until timeout.
                        if let Err(e) = downstream.cancel_goal(GOAL_TIMEOUT).await {
                            warn!(error = %e, "move_gripper: {} shutdown cancel propagation failed", $side);
                        }
                        return Outcome::failed("backbone shutting down");
                    }
                    _ = goal_ctx.cancel_signal(), if !upstream_cancelled => {
                        if let Err(e) = downstream.cancel_goal(GOAL_TIMEOUT).await {
                            warn!(error = %e, "move_gripper: {} cancel propagation failed", $side);
                        }
                        upstream_cancelled = true;
                    }
                    r = &mut result_fut => break r,
                }
            };

            match result {
                Ok(result) => match result.outcome {
                    link::ResultOutcome::Completed(data) => Outcome {
                        success: data.success,
                        message: data.message,
                        final_joint_positions: data.final_joint_positions,
                        action_time: data.action_time,
                        is_cancelled: false,
                    },
                    link::ResultOutcome::Cancelled(data) => Outcome {
                        success: data.success,
                        message: data.message,
                        final_joint_positions: data.final_joint_positions,
                        action_time: data.action_time,
                        is_cancelled: true,
                    },
                    link::ResultOutcome::Abandoned => Outcome::failed(format!("{} gripper abandoned", $side)),
                    link::ResultOutcome::Expired => Outcome::failed(format!("{} gripper result expired", $side)),
                },
                Err(e) => {
                    if upstream_cancelled {
                        Outcome::cancelled(format!("{} gripper cancellation, get_result: {e}", $side))
                    } else {
                        Outcome::failed(format!("get_result from {} gripper failed: {e}", $side))
                    }
                }
            }
        }
    };
}

gripper_dispatch!(dispatch_left, left_gripper_move_gripper, "left");
gripper_dispatch!(dispatch_right, right_gripper_move_gripper, "right");
