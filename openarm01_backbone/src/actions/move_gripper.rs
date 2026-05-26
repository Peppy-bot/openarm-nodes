use std::sync::{Arc, Mutex};
use std::time::Duration;

use peppygen::NodeRunner;
use peppygen::QoSProfile;
use peppygen::consumed_actions::gripper_move_gripper;
use peppygen::exposed_actions::move_gripper::{
    ActionHandle, GoalRequest, GoalRequestData, GoalResponse, ResultResponse,
};
use peppylib::runtime::CancellationToken;
use tracing::{info, warn};

use crate::startup::Routing;

const GOAL_TIMEOUT: Duration = Duration::from_secs(5);
const RESULT_TIMEOUT: Duration = Duration::from_secs(60);

struct Outcome {
    success: bool,
    message: String,
    final_joint_positions: Vec<f64>,
    action_time: f64,
}

impl Outcome {
    fn failed(message: impl Into<String>) -> Self {
        Self {
            success: false,
            message: message.into(),
            final_joint_positions: Vec::new(),
            action_time: 0.0,
        }
    }
}

// Expose move_gripper, and for each accepted goal forward it to the gripper instance the goal's
// gripper_id resolves to, relaying the gripper's feedback and result back to the caller.
pub async fn run(
    runner: Arc<NodeRunner>,
    routing: Arc<Routing>,
    token: CancellationToken,
) -> peppygen::Result<()> {
    let mut handle = ActionHandle::expose(&runner).await?;
    let pending: Arc<Mutex<Option<GoalRequestData>>> = Arc::new(Mutex::new(None));

    loop {
        // 1. Accept a goal only if its gripper_id maps to a known gripper instance.
        let pending_for_handler = pending.clone();
        let routing_for_handler = routing.clone();
        let goal_handled = tokio::select! {
            _ = token.cancelled() => break,
            result = handle.handle_goal_next_request(move |req: GoalRequest| {
                let gripper_id = req.data.gripper_id;
                if routing_for_handler.gripper_instance(gripper_id).is_none() {
                    warn!(gripper_id, "move_gripper: no gripper instance for gripper_id; rejecting");
                    return Ok(GoalResponse::new(false));
                }
                *pending_for_handler.lock().unwrap() = Some(req.data);
                Ok(GoalResponse::new(true))
            }) => result,
        };
        if let Err(e) = goal_handled {
            warn!(error = %e, "move_gripper: goal handling failed");
            continue;
        }

        // 2. Forward the accepted goal; rejected goals leave the slot empty.
        let goal = match pending.lock().unwrap().take() {
            Some(goal) => goal,
            None => continue,
        };
        let outcome = forward(&runner, &routing, goal, &handle, &token).await;

        // 3. Answer the caller's result request (cancel-aware so shutdown can't wedge here).
        let stash = Arc::new(Mutex::new(Some(outcome)));
        let stash_for_handler = stash.clone();
        tokio::select! {
            _ = token.cancelled() => break,
            result = handle.handle_result_next_request(move |_req| {
                let outcome = stash_for_handler
                    .lock()
                    .unwrap()
                    .take()
                    .unwrap_or_else(|| Outcome::failed("backbone produced no result"));
                Ok(ResultResponse::new(
                    outcome.success,
                    outcome.message,
                    outcome.final_joint_positions,
                    outcome.action_time,
                ))
            }) => {
                if let Err(e) = result {
                    warn!(error = %e, "move_gripper: result handling failed");
                }
            }
        }
    }
    Ok(())
}

async fn forward(
    runner: &NodeRunner,
    routing: &Routing,
    goal: GoalRequestData,
    handle: &ActionHandle,
    token: &CancellationToken,
) -> Outcome {
    // Presence was checked at goal acceptance; re-resolve to own the instance id locally.
    let instance_id = match routing.gripper_instance(goal.gripper_id) {
        Some(instance_id) => instance_id.to_string(),
        None => return Outcome::failed("gripper_id no longer routable"),
    };

    let mut downstream = match gripper_move_gripper::ActionHandle::fire_goal(
        runner,
        GOAL_TIMEOUT,
        None,
        Some(&instance_id),
        gripper_move_gripper::GoalRequest {
            feedback_frequency: goal.feedback_frequency,
            position: goal.position,
        },
        QoSProfile::SensorData,
    )
    .await
    {
        Ok(handle) if handle.data.accepted => handle,
        Ok(_) => return Outcome::failed("gripper rejected the goal"),
        Err(e) => return Outcome::failed(format!("fire_goal to gripper failed: {e}")),
    };

    info!(gripper_id = goal.gripper_id, instance = %instance_id, "move_gripper: forwarded to gripper");

    // Relay the gripper's feedback upward until its stream ends (Err = no more feedback).
    loop {
        tokio::select! {
            _ = token.cancelled() => return Outcome::failed("backbone shutting down"),
            feedback = downstream.on_next_feedback_message() => match feedback {
                Ok(f) => {
                    let _ = handle.emit_feedback(f.joint_positions, f.action_time).await;
                }
                Err(_) => break,
            }
        }
    }

    match downstream.get_result(RESULT_TIMEOUT).await {
        Ok(result) => Outcome {
            success: result.data.success,
            message: result.data.message,
            final_joint_positions: result.data.final_joint_positions,
            action_time: result.data.action_time,
        },
        Err(e) => Outcome::failed(format!("get_result from gripper failed: {e}")),
    }
}
