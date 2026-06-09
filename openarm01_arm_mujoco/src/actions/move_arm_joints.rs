// Joint-space 7-DOF control: validate target, run a feedback loop on the
// shared joint_states cache, and republish set_ctrl_arm_<side> every tick
// to survive best-effort QoS drops. No shutdown zero — would self-collide.

use std::collections::HashMap;
use std::sync::Arc;
use std::time::{Duration, Instant};

use peppygen::NodeRunner;
use peppygen::exposed_actions::openarm01_arm::v1::move_arm_joints;
use peppylib::config::QoSProfile;
use peppylib::messaging::SenderTarget;
use peppylib::runtime::CancellationToken;
use peppylib::{MessengerHandle, Payload, TopicMessenger};
use serde::Serialize;
use sim_bridge_core::DaemonState;
use tracing::{error, warn};

use crate::config::ArmId;
use crate::state::SharedState;

const DOF: usize = 7;

// Per-joint tolerance (radians) for "reached target". ~0.5° at each joint.
const POSITION_TOLERANCE_RAD: f64 = 0.01;
const MOTION_TIMEOUT: Duration = Duration::from_secs(30);

// Per-joint max diff (not sum |q_i|, which aliases when one joint moves +ε
// and another -ε). Below STALL_EPSILON_RAD over a 500ms window → stalled.
const STALL_LOOKBACK_ITERS: u32 = 100;
const STALL_EPSILON_RAD: f64 = 5e-4;
const FEEDBACK_LOOP_TICK: Duration = Duration::from_millis(5);

// Joint angle hard limits — rejected at goal time. MuJoCo clamps internally
// too, but rejecting early gives the caller a clear "out of range" failure
// instead of silently saturating.
const JOINT_MIN_RAD: f64 = -std::f64::consts::PI;
const JOINT_MAX_RAD: f64 = std::f64::consts::PI;

// If set_ctrl publishing fails for this many consecutive ticks (one full
// stall window, ~500ms), the arm is not being commanded — bail rather than
// let stall detection report a false "physical limit".
const MAX_CONSECUTIVE_PUBLISH_FAILURES: u32 = STALL_LOOKBACK_ITERS;

const ARM_NODE_NAME: &str = "openarm01_arm";

// Keys are joint names (openarm_<side>_joint{1..7}). Isaac resolves them via
// USD dof_names; MuJoCo via the joint-name alias. Same payload, both engines.
#[derive(Serialize)]
struct SetCtrlPayload<'a> {
    actuator_values: HashMap<&'a str, f64>,
}

struct AcceptedGoal {
    target_positions: [f64; DOF],
    feedback_period: Duration,
}

struct MotionResult {
    success: bool,
    is_cancelled: bool,
    message: String,
    final_positions: [f64; DOF],
    action_time: f64,
}

fn feedback_period(freq_hz: u32) -> Duration {
    Duration::from_micros(1_000_000 / freq_hz.max(1) as u64)
}

fn validate_positions(positions: &[f64; DOF]) -> Result<(), String> {
    for (i, &q) in positions.iter().enumerate() {
        if !q.is_finite() {
            return Err(format!("joint {} target is not finite: {q}", i + 1));
        }
        if !(JOINT_MIN_RAD..=JOINT_MAX_RAD).contains(&q) {
            return Err(format!(
                "joint {} target {q} out of range [{JOINT_MIN_RAD}, {JOINT_MAX_RAD}]",
                i + 1,
            ));
        }
    }
    Ok(())
}

pub async fn run(
    runner: Arc<NodeRunner>,
    arm_id: ArmId,
    state: Arc<SharedState>,
    token: CancellationToken,
    handle: Arc<MessengerHandle>,
    daemon: DaemonState,
) {
    let side = arm_id.side_word();
    let actuator_names: [String; DOF] =
        std::array::from_fn(|i| format!("openarm_{side}_joint{}", i + 1));
    let set_ctrl_topic = format!("set_ctrl_arm_{side}");
    // Unique instance_id per arm side so concurrent left+right arms don't
    // collide on the peppylib publisher registry.
    let instance_id = format!("openarm01_arm_{side}_setctrl_pub");

    let mut action_handle = move_arm_joints::ActionHandle::expose(&runner)
        .await
        .expect("expose move_arm_joints");

    loop {
        // v0.10 GoalContext model: the decider closure validates and returns
        // accept/reject. On accept, handle_goal_next_request yields Some(ctx).
        let goal_request =
            action_handle.handle_goal_next_request(|req: &move_arm_joints::GoalRequest| {
                let positions = req.data.joint_positions;
                if let Err(why) = validate_positions(&positions) {
                    return Ok(move_arm_joints::GoalResponse::reject(why));
                }
                Ok(move_arm_joints::GoalResponse::accept())
            });

        let goal_ctx = tokio::select! {
            _ = token.cancelled() => break,
            result = goal_request => {
                match result {
                    Ok(Some(ctx)) => ctx,
                    Ok(None) => break, // action exposed but shutting down
                    Err(e) => {
                        error!("move_arm_joints goal: {e}");
                        continue;
                    }
                }
            }
        };

        let goal = AcceptedGoal {
            target_positions: goal_ctx.request().data.joint_positions,
            feedback_period: feedback_period(goal_ctx.request().data.feedback_frequency),
        };

        let result = run_control_loop(
            &handle,
            &daemon,
            &state,
            &set_ctrl_topic,
            &instance_id,
            &actuator_names,
            &goal_ctx,
            &token,
            goal,
        )
        .await;

        let dispatch = if result.is_cancelled {
            goal_ctx
                .complete_cancelled(
                    result.success,
                    result.message,
                    result.final_positions,
                    result.action_time,
                )
                .await
        } else {
            goal_ctx
                .complete(
                    result.success,
                    result.message,
                    result.final_positions,
                    result.action_time,
                )
                .await
        };
        if let Err(e) = dispatch {
            error!("move_arm_joints complete: {e}");
        }
    }
}

#[allow(clippy::too_many_arguments)]
async fn run_control_loop(
    handle: &MessengerHandle,
    daemon: &DaemonState,
    state: &Arc<SharedState>,
    set_ctrl_topic: &str,
    instance_id: &str,
    actuator_names: &[String; DOF],
    goal_ctx: &move_arm_joints::GoalContext,
    token: &CancellationToken,
    goal: AcceptedGoal,
) -> MotionResult {
    let start = Instant::now();
    let mut last_feedback = Instant::now();
    let mut window_anchor: Option<[f64; DOF]> = None;
    let mut iter: u32 = 0;
    let mut consecutive_publish_failures: u32 = 0;

    loop {
        // Republish every tick: peppylib Standard is best-effort, so this is
        // the self-healing path. Idempotent. If it keeps failing the arm isn't
        // commanded — bail before stall lies.
        match publish_set_ctrl(
            handle,
            daemon,
            set_ctrl_topic,
            instance_id,
            actuator_names,
            &goal.target_positions,
        )
        .await
        {
            Ok(()) => consecutive_publish_failures = 0,
            Err(e) => {
                consecutive_publish_failures += 1;
                warn!("set_ctrl publish failed ({consecutive_publish_failures}): {e}");
                if consecutive_publish_failures >= MAX_CONSECUTIVE_PUBLISH_FAILURES {
                    return MotionResult {
                        success: false,
                        is_cancelled: false,
                        message: "set_ctrl publish failing — arm not commandable".into(),
                        final_positions: [0.0; DOF],
                        action_time: start.elapsed().as_secs_f64(),
                    };
                }
            }
        }

        let elapsed = start.elapsed();
        let elapsed_secs = elapsed.as_secs_f64();

        let latest = { state.joint_states.lock().await.clone() };
        let snap = match latest {
            Some(s) if s.positions.len() == DOF => s,
            Some(s) => {
                warn!(
                    "move_arm_joints: cache has {} positions, expected {DOF} — waiting",
                    s.positions.len()
                );
                if elapsed > MOTION_TIMEOUT {
                    return MotionResult {
                        success: false,
                        is_cancelled: false,
                        message: format!(
                            "telemetry DOF mismatch: got {}, expected {DOF}",
                            s.positions.len()
                        ),
                        final_positions: [0.0; DOF],
                        action_time: elapsed_secs,
                    };
                }
                tokio::select! {
                    _ = token.cancelled() => return cancelled(elapsed_secs, [0.0; DOF]),
                    _ = goal_ctx.cancel_signal() => return cancelled(elapsed_secs, [0.0; DOF]),
                    _ = tokio::time::sleep(FEEDBACK_LOOP_TICK) => continue,
                }
            }
            None => {
                if elapsed > MOTION_TIMEOUT {
                    return MotionResult {
                        success: false,
                        is_cancelled: false,
                        message: "no telemetry from robot_initializer".into(),
                        final_positions: [0.0; DOF],
                        action_time: elapsed_secs,
                    };
                }
                tokio::select! {
                    _ = token.cancelled() => return cancelled(elapsed_secs, [0.0; DOF]),
                    _ = goal_ctx.cancel_signal() => return cancelled(elapsed_secs, [0.0; DOF]),
                    _ = tokio::time::sleep(FEEDBACK_LOOP_TICK) => continue,
                }
            }
        };

        let mut current: [f64; DOF] = [0.0; DOF];
        current.copy_from_slice(&snap.positions);

        let worst_err = current
            .iter()
            .zip(goal.target_positions.iter())
            .map(|(&q, &target)| (q - target).abs())
            .fold(0.0_f64, f64::max);
        let within_tolerance = worst_err < POSITION_TOLERANCE_RAD;

        iter += 1;
        let stalled = if iter % STALL_LOOKBACK_ITERS == 0 {
            let was_stalled = window_anchor
                .map(|prev| {
                    current
                        .iter()
                        .zip(prev.iter())
                        .map(|(&q, &p)| (q - p).abs())
                        .fold(0.0_f64, f64::max)
                        < STALL_EPSILON_RAD
                })
                .unwrap_or(false);
            window_anchor = Some(current);
            was_stalled
        } else {
            false
        };

        if last_feedback.elapsed() >= goal.feedback_period {
            if let Err(e) = goal_ctx.publish_feedback(current, elapsed_secs).await {
                warn!("feedback: {e}");
            }
            last_feedback = Instant::now();
        }

        if within_tolerance {
            return MotionResult {
                success: true,
                is_cancelled: false,
                message: "reached".into(),
                final_positions: current,
                action_time: elapsed_secs,
            };
        }
        if stalled {
            // Stall on the arm is NOT success the way it is on the gripper
            // (a gripper at a physical limit has done its job). Arm stall
            // means the goal couldn't be reached — fail with the final pose.
            return MotionResult {
                success: false,
                is_cancelled: false,
                message: "stalled before reaching target".into(),
                final_positions: current,
                action_time: elapsed_secs,
            };
        }
        if elapsed > MOTION_TIMEOUT {
            return MotionResult {
                success: false,
                is_cancelled: false,
                message: "timeout".into(),
                final_positions: current,
                action_time: elapsed_secs,
            };
        }

        tokio::select! {
            _ = token.cancelled() => return cancelled(elapsed_secs, current),
            _ = goal_ctx.cancel_signal() => return cancelled(elapsed_secs, current),
            _ = tokio::time::sleep(FEEDBACK_LOOP_TICK) => {}
        }
    }
}

fn cancelled(action_time: f64, final_positions: [f64; DOF]) -> MotionResult {
    MotionResult {
        success: false,
        is_cancelled: true,
        message: "cancelled".into(),
        final_positions,
        action_time,
    }
}

async fn publish_set_ctrl(
    handle: &MessengerHandle,
    daemon: &DaemonState,
    topic: &str,
    instance_id: &str,
    actuator_names: &[String; DOF],
    targets: &[f64; DOF],
) -> std::result::Result<(), String> {
    let mut values: HashMap<&str, f64> = HashMap::with_capacity(DOF);
    for (name, &target) in actuator_names.iter().zip(targets.iter()) {
        values.insert(name.as_str(), target);
    }
    let payload = SetCtrlPayload {
        actuator_values: values,
    };
    let bytes = serde_json::to_vec(&payload).map_err(|e| e.to_string())?;

    // Publish as openarm01_arm:v1 so real + sim instances look identical to
    // consumers on the bus.
    let target =
        SenderTarget::node(ARM_NODE_NAME, "v1").map_err(|e| format!("invalid as_target: {e}"))?;

    TopicMessenger::emit(
        handle,
        &daemon.core_node_name,
        instance_id,
        target,
        topic,
        QoSProfile::Standard,
        Payload::from(bytes),
    )
    .await
    .map_err(|e| e.to_string())
}
