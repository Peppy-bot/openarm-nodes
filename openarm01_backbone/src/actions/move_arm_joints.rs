use std::sync::Arc;
use std::time::Duration;

use peppygen::NodeRunner;
use peppygen::QoSProfile;
use peppygen::consumed_actions::{left_arm_move_arm_joints, right_arm_move_arm_joints};
use peppygen::exposed_actions::move_arm_joints::{ActionHandle, GoalContext, GoalRequest, GoalResponse};
use peppylib::runtime::CancellationToken;
use tracing::{info, warn};

const GOAL_TIMEOUT: Duration = Duration::from_secs(5);
const RESULT_TIMEOUT: Duration = Duration::from_secs(60);
const ARM_DOF: usize = 7;

struct Outcome {
    success: bool,
    message: String,
    final_joint_positions: [f64; ARM_DOF],
    action_time: f64,
    is_cancelled: bool,
}

impl Outcome {
    fn failed(message: impl Into<String>) -> Self {
        Self {
            success: false,
            message: message.into(),
            final_joint_positions: [0.0; ARM_DOF],
            action_time: 0.0,
            is_cancelled: false,
        }
    }

    fn cancelled(message: impl Into<String>) -> Self {
        Self {
            success: false,
            message: message.into(),
            final_joint_positions: [0.0; ARM_DOF],
            action_time: 0.0,
            is_cancelled: true,
        }
    }
}

// Expose move_arm_joints. For each accepted goal, dispatch on arm_id to the
// left or right consumed action, relay feedback upward, propagate cancel
// downward, and answer the caller with either complete or complete_cancelled.
pub async fn run(runner: Arc<NodeRunner>, token: CancellationToken) -> peppygen::Result<()> {
    let mut handle = ActionHandle::expose(&runner).await?;

    loop {
        let goal_ctx = tokio::select! {
            _ = token.cancelled() => break,
            result = handle.handle_goal_next_request(|req: &GoalRequest| {
                match req.data.arm_id {
                    0 | 1 => Ok(GoalResponse::accept()),
                    other => Ok(GoalResponse::reject(format!(
                        "invalid arm_id {other} (expected 0 for left, 1 for right)"
                    ))),
                }
            }) => match result {
                Ok(Some(ctx)) => ctx,
                Ok(None) => break,
                Err(e) => {
                    warn!(error = %e, "move_arm_joints: goal handling failed");
                    continue;
                }
            },
        };

        let outcome = forward(&runner, &goal_ctx, &token).await;

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
            warn!(error = %e, "move_arm_joints: complete failed");
        }
    }
    Ok(())
}

async fn forward(runner: &NodeRunner, goal_ctx: &GoalContext, token: &CancellationToken) -> Outcome {
    let req_data = goal_ctx.request().data.clone();
    match req_data.arm_id {
        0 => dispatch_left(runner, &req_data, goal_ctx, token).await,
        1 => dispatch_right(runner, &req_data, goal_ctx, token).await,
        // Unreachable: handle_goal_next_request rejects anything outside [0, 1].
        _ => Outcome::failed("arm_id out of range"),
    }
}

async fn dispatch_left(
    runner: &NodeRunner,
    req: &peppygen::exposed_actions::move_arm_joints::GoalRequestData,
    goal_ctx: &GoalContext,
    token: &CancellationToken,
) -> Outcome {
    let mut downstream = match left_arm_move_arm_joints::ActionHandle::fire_goal(
        runner,
        GOAL_TIMEOUT,
        left_arm_move_arm_joints::GoalRequest {
            feedback_frequency: req.feedback_frequency,
            joint_positions: req.joint_positions,
        },
        QoSProfile::SensorData,
    )
    .await
    {
        Ok(handle) if handle.data.accepted => handle,
        Ok(handle) => {
            return Outcome::failed(format!(
                "left arm rejected goal: {}",
                handle.data.error_message.unwrap_or_else(|| "no reason given".into())
            ));
        }
        Err(e) => return Outcome::failed(format!("fire_goal to left arm failed: {e}")),
    };
    info!("move_arm_joints: forwarded to left arm");

    relay_left(&mut downstream, goal_ctx, token).await
}

async fn dispatch_right(
    runner: &NodeRunner,
    req: &peppygen::exposed_actions::move_arm_joints::GoalRequestData,
    goal_ctx: &GoalContext,
    token: &CancellationToken,
) -> Outcome {
    let mut downstream = match right_arm_move_arm_joints::ActionHandle::fire_goal(
        runner,
        GOAL_TIMEOUT,
        right_arm_move_arm_joints::GoalRequest {
            feedback_frequency: req.feedback_frequency,
            joint_positions: req.joint_positions,
        },
        QoSProfile::SensorData,
    )
    .await
    {
        Ok(handle) if handle.data.accepted => handle,
        Ok(handle) => {
            return Outcome::failed(format!(
                "right arm rejected goal: {}",
                handle.data.error_message.unwrap_or_else(|| "no reason given".into())
            ));
        }
        Err(e) => return Outcome::failed(format!("fire_goal to right arm failed: {e}")),
    };
    info!("move_arm_joints: forwarded to right arm");

    relay_right(&mut downstream, goal_ctx, token).await
}

// The two relay_* helpers below are byte-equivalent except for the consumed-action
// module path. Macros would deduplicate but obscure the call sites; two short
// functions are clearer at the cost of mild repetition.
async fn relay_left(
    downstream: &mut left_arm_move_arm_joints::ActionHandle,
    goal_ctx: &GoalContext,
    token: &CancellationToken,
) -> Outcome {
    let mut upstream_cancelled = false;

    loop {
        tokio::select! {
            _ = token.cancelled() => return Outcome::failed("backbone shutting down"),
            _ = goal_ctx.cancel_signal(), if !upstream_cancelled => {
                if let Err(e) = downstream.cancel_goal(GOAL_TIMEOUT).await {
                    warn!(error = %e, "move_arm_joints: left cancel propagation failed");
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
                            "move_arm_joints: upstream publish_feedback failed; continuing"
                        );
                    }
                }
                Err(_) => break,
            }
        }
    }

    match downstream.get_result(RESULT_TIMEOUT).await {
        Ok(result) => match result.outcome {
            left_arm_move_arm_joints::ResultOutcome::Completed(data) => Outcome {
                success: data.success,
                message: data.message,
                final_joint_positions: data.final_joint_positions,
                action_time: data.action_time,
                is_cancelled: false,
            },
            left_arm_move_arm_joints::ResultOutcome::Cancelled(data) => Outcome {
                success: data.success,
                message: data.message,
                final_joint_positions: data.final_joint_positions,
                action_time: data.action_time,
                is_cancelled: true,
            },
            left_arm_move_arm_joints::ResultOutcome::Abandoned => Outcome::failed("left arm abandoned"),
            left_arm_move_arm_joints::ResultOutcome::Expired => Outcome::failed("left arm result expired"),
        },
        Err(e) => {
            if upstream_cancelled {
                Outcome::cancelled(format!("left arm cancellation, get_result: {e}"))
            } else {
                Outcome::failed(format!("get_result from left arm failed: {e}"))
            }
        }
    }
}

async fn relay_right(
    downstream: &mut right_arm_move_arm_joints::ActionHandle,
    goal_ctx: &GoalContext,
    token: &CancellationToken,
) -> Outcome {
    let mut upstream_cancelled = false;

    loop {
        tokio::select! {
            _ = token.cancelled() => return Outcome::failed("backbone shutting down"),
            _ = goal_ctx.cancel_signal(), if !upstream_cancelled => {
                if let Err(e) = downstream.cancel_goal(GOAL_TIMEOUT).await {
                    warn!(error = %e, "move_arm_joints: right cancel propagation failed");
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
                            "move_arm_joints: upstream publish_feedback failed; continuing"
                        );
                    }
                }
                Err(_) => break,
            }
        }
    }

    match downstream.get_result(RESULT_TIMEOUT).await {
        Ok(result) => match result.outcome {
            right_arm_move_arm_joints::ResultOutcome::Completed(data) => Outcome {
                success: data.success,
                message: data.message,
                final_joint_positions: data.final_joint_positions,
                action_time: data.action_time,
                is_cancelled: false,
            },
            right_arm_move_arm_joints::ResultOutcome::Cancelled(data) => Outcome {
                success: data.success,
                message: data.message,
                final_joint_positions: data.final_joint_positions,
                action_time: data.action_time,
                is_cancelled: true,
            },
            right_arm_move_arm_joints::ResultOutcome::Abandoned => Outcome::failed("right arm abandoned"),
            right_arm_move_arm_joints::ResultOutcome::Expired => Outcome::failed("right arm result expired"),
        },
        Err(e) => {
            if upstream_cancelled {
                Outcome::cancelled(format!("right arm cancellation, get_result: {e}"))
            } else {
                Outcome::failed(format!("get_result from right arm failed: {e}"))
            }
        }
    }
}
