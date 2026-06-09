use std::sync::Arc;
use std::time::Duration;

use peppygen::NodeRunner;
use peppygen::QoSProfile;
use peppygen::consumed_actions::{left_gripper_move_gripper, right_gripper_move_gripper};
use peppygen::exposed_actions::move_gripper::{ActionHandle, GoalContext, GoalRequest, GoalResponse};
use peppylib::runtime::CancellationToken;
use tracing::{info, warn};

const GOAL_TIMEOUT: Duration = Duration::from_secs(5);
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

// Expose move_gripper. For each accepted goal, dispatch on gripper_id to the
// left or right consumed action, relay feedback upward, propagate cancel
// downward, and answer the caller with either complete or complete_cancelled.
pub async fn run(runner: Arc<NodeRunner>, token: CancellationToken) -> peppygen::Result<()> {
    let mut handle = ActionHandle::expose(&runner).await?;

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

        // Spawn per goal so the accept loop returns immediately to await the
        // next probe. Otherwise left+right move_gripper serialise through one
        // task and the second goal sees the backbone instance as unreachable.
        let runner_for_goal = Arc::clone(&runner);
        let token_for_goal = token.clone();
        tokio::spawn(async move {
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

async fn forward(runner: &NodeRunner, goal_ctx: &GoalContext, token: &CancellationToken) -> Outcome {
    let req_data = goal_ctx.request().data.clone();
    match req_data.gripper_id {
        0 => dispatch_left(runner, &req_data, goal_ctx, token).await,
        1 => dispatch_right(runner, &req_data, goal_ctx, token).await,
        // Unreachable: handle_goal_next_request rejects anything outside [0, 1].
        _ => Outcome::failed("gripper_id out of range"),
    }
}

async fn dispatch_left(
    runner: &NodeRunner,
    req: &peppygen::exposed_actions::move_gripper::GoalRequestData,
    goal_ctx: &GoalContext,
    token: &CancellationToken,
) -> Outcome {
    let mut downstream = match left_gripper_move_gripper::ActionHandle::fire_goal(
        runner,
        GOAL_TIMEOUT,
        left_gripper_move_gripper::GoalRequest {
            feedback_frequency: req.feedback_frequency,
            position: req.position,
        },
        QoSProfile::SensorData,
    )
    .await
    {
        Ok(handle) if handle.data.accepted => handle,
        Ok(handle) => {
            return Outcome::failed(format!(
                "left gripper rejected goal: {}",
                handle.data.error_message.unwrap_or_else(|| "no reason given".into())
            ));
        }
        Err(e) => return Outcome::failed(format!("fire_goal to left gripper failed: {e}")),
    };
    info!("move_gripper: forwarded to left gripper");

    relay_left(&mut downstream, goal_ctx, token).await
}

async fn dispatch_right(
    runner: &NodeRunner,
    req: &peppygen::exposed_actions::move_gripper::GoalRequestData,
    goal_ctx: &GoalContext,
    token: &CancellationToken,
) -> Outcome {
    let mut downstream = match right_gripper_move_gripper::ActionHandle::fire_goal(
        runner,
        GOAL_TIMEOUT,
        right_gripper_move_gripper::GoalRequest {
            feedback_frequency: req.feedback_frequency,
            position: req.position,
        },
        QoSProfile::SensorData,
    )
    .await
    {
        Ok(handle) if handle.data.accepted => handle,
        Ok(handle) => {
            return Outcome::failed(format!(
                "right gripper rejected goal: {}",
                handle.data.error_message.unwrap_or_else(|| "no reason given".into())
            ));
        }
        Err(e) => return Outcome::failed(format!("fire_goal to right gripper failed: {e}")),
    };
    info!("move_gripper: forwarded to right gripper");

    relay_right(&mut downstream, goal_ctx, token).await
}

// The two relay_* helpers below are byte-equivalent except for the consumed-action
// module path. Macros would deduplicate but obscure the call sites; two short
// functions are clearer at the cost of mild repetition.
async fn relay_left(
    downstream: &mut left_gripper_move_gripper::ActionHandle,
    goal_ctx: &GoalContext,
    token: &CancellationToken,
) -> Outcome {
    let mut upstream_cancelled = false;

    loop {
        tokio::select! {
            _ = token.cancelled() => {
                // Propagate shutdown to the in-flight left gripper goal so it
                // stops driving fingers instead of running until timeout.
                if let Err(e) = downstream.cancel_goal(GOAL_TIMEOUT).await {
                    warn!(error = %e, "move_gripper: left shutdown cancel propagation failed");
                }
                return Outcome::failed("backbone shutting down");
            }
            _ = goal_ctx.cancel_signal(), if !upstream_cancelled => {
                if let Err(e) = downstream.cancel_goal(GOAL_TIMEOUT).await {
                    warn!(error = %e, "move_gripper: left cancel propagation failed");
                }
                upstream_cancelled = true;
            }
            feedback = downstream.on_next_feedback_message() => match feedback {
                Ok(f) => {
                    let action_time = f.action_time;
                    if let Err(e) = goal_ctx.publish_feedback(f.joint_positions, action_time).await {
                        warn!(
                            error = %e,
                            action_time,
                            "move_gripper: upstream publish_feedback failed; continuing"
                        );
                    }
                }
                Err(_) => break,
            }
        }
    }

    match downstream.get_result(RESULT_TIMEOUT).await {
        Ok(result) => match result.outcome {
            left_gripper_move_gripper::ResultOutcome::Completed(data) => Outcome {
                success: data.success,
                message: data.message,
                final_joint_positions: data.final_joint_positions,
                action_time: data.action_time,
                is_cancelled: false,
            },
            left_gripper_move_gripper::ResultOutcome::Cancelled(data) => Outcome {
                success: data.success,
                message: data.message,
                final_joint_positions: data.final_joint_positions,
                action_time: data.action_time,
                is_cancelled: true,
            },
            left_gripper_move_gripper::ResultOutcome::Abandoned => Outcome::failed("left gripper abandoned"),
            left_gripper_move_gripper::ResultOutcome::Expired => Outcome::failed("left gripper result expired"),
        },
        Err(e) => {
            if upstream_cancelled {
                Outcome::cancelled(format!("left gripper cancellation, get_result: {e}"))
            } else {
                Outcome::failed(format!("get_result from left gripper failed: {e}"))
            }
        }
    }
}

async fn relay_right(
    downstream: &mut right_gripper_move_gripper::ActionHandle,
    goal_ctx: &GoalContext,
    token: &CancellationToken,
) -> Outcome {
    let mut upstream_cancelled = false;

    loop {
        tokio::select! {
            _ = token.cancelled() => {
                // Propagate shutdown to the in-flight right gripper goal so it
                // stops driving fingers instead of running until timeout.
                if let Err(e) = downstream.cancel_goal(GOAL_TIMEOUT).await {
                    warn!(error = %e, "move_gripper: right shutdown cancel propagation failed");
                }
                return Outcome::failed("backbone shutting down");
            }
            _ = goal_ctx.cancel_signal(), if !upstream_cancelled => {
                if let Err(e) = downstream.cancel_goal(GOAL_TIMEOUT).await {
                    warn!(error = %e, "move_gripper: right cancel propagation failed");
                }
                upstream_cancelled = true;
            }
            feedback = downstream.on_next_feedback_message() => match feedback {
                Ok(f) => {
                    let action_time = f.action_time;
                    if let Err(e) = goal_ctx.publish_feedback(f.joint_positions, action_time).await {
                        warn!(
                            error = %e,
                            action_time,
                            "move_gripper: upstream publish_feedback failed; continuing"
                        );
                    }
                }
                Err(_) => break,
            }
        }
    }

    match downstream.get_result(RESULT_TIMEOUT).await {
        Ok(result) => match result.outcome {
            right_gripper_move_gripper::ResultOutcome::Completed(data) => Outcome {
                success: data.success,
                message: data.message,
                final_joint_positions: data.final_joint_positions,
                action_time: data.action_time,
                is_cancelled: false,
            },
            right_gripper_move_gripper::ResultOutcome::Cancelled(data) => Outcome {
                success: data.success,
                message: data.message,
                final_joint_positions: data.final_joint_positions,
                action_time: data.action_time,
                is_cancelled: true,
            },
            right_gripper_move_gripper::ResultOutcome::Abandoned => Outcome::failed("right gripper abandoned"),
            right_gripper_move_gripper::ResultOutcome::Expired => Outcome::failed("right gripper result expired"),
        },
        Err(e) => {
            if upstream_cancelled {
                Outcome::cancelled(format!("right gripper cancellation, get_result: {e}"))
            } else {
                Outcome::failed(format!("get_result from right gripper failed: {e}"))
            }
        }
    }
}
