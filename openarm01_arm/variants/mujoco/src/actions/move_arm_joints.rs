// move_arm_joints action handler — peppylib-mediated joint-space control.
//
// Each goal:
//   1. Validates the 7-DOF target (each joint angle within the v10 arm's
//      [-pi, pi] limits — wider checks live in the backbone's IK + CD pass).
//   2. Enters a feedback loop reading the latest joint_states from the
//      shared cache (populated by pipeline::telemetry).
//   3. Re-publishes raw set_ctrl_arm_<side> on every tick. peppylib
//      QoSProfile::Standard is best-effort; a single dropped message would
//      otherwise stall the arm. The publish is idempotent.
//   4. Streams typed feedback at the requested rate; detects convergence
//      (worst per-joint error < POSITION_TOLERANCE_RAD) or stall
//      (sum-of-motion across a ~500ms window < STALL_EPSILON_RAD).
//   5. Returns result.
//
// Mirrors the convergence + stall pattern from move_gripper but operates
// in 7-D joint space instead of 2-finger displacement. The set_ctrl
// payload carries 7 actuator values; the MuJoCo bridge_extension's
// `actuator_ctrl` subscriber writes them to MjData.ctrl[] before each
// mj_step.

use std::collections::HashMap;
use std::sync::{Arc, Mutex as StdMutex};
use std::time::{Duration, Instant};

use peppygen::NodeRunner;
use peppygen::exposed_actions::move_arm_joints;
use peppylib::config::QoSProfile;
use peppylib::runtime::CancellationToken;
use peppylib::{MessengerHandle, Payload, TopicMessenger};
use serde::Serialize;
use sim_bridge_core::DaemonState;
use tracing::{error, info, warn};

use crate::config::ArmId;
use crate::state::SharedState;

const DOF: usize = 7;

// Per-joint tolerance (radians) for "reached target". ~0.5° at each joint.
// Loose enough that minor sim integrator noise doesn't keep the goal open;
// tight enough that downstream consumers see the requested pose materially.
const POSITION_TOLERANCE_RAD: f64 = 0.01;
const MOTION_TIMEOUT: Duration = Duration::from_secs(30);

// Stall detection: when the arm can't reach the requested target (jammed by
// collision, hitting a joint limit, lost actuation, etc.), qpos stops
// changing. We compare current sum of |positions| against the sum from
// ~500ms ago. STALL_LOOKBACK_ITERS × FEEDBACK_LOOP_TICK = 500ms window;
// 0.5 mrad-over-500ms = 1 mrad/s — below that the arm is treated as stalled.
const STALL_LOOKBACK_ITERS: u32 = 100;
const STALL_EPSILON_RAD: f64 = 5e-4;
const FEEDBACK_LOOP_TICK: Duration = Duration::from_millis(5);

// Joint angle hard limits — anything outside this is rejected at goal time.
// MuJoCo joint definitions clamp internally too, but rejecting early gives
// the caller a clear "out of range" failure instead of silently saturating.
const JOINT_MIN_RAD: f64 = -std::f64::consts::PI;
const JOINT_MAX_RAD: f64 =  std::f64::consts::PI;

const ARM_NODE_NAME: &str = "openarm01_arm";

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
    message: String,
    final_positions: [f64; DOF],
    action_time: f64,
}

impl Default for MotionResult {
    fn default() -> Self {
        Self {
            success: false,
            message: "no result".into(),
            final_positions: [0.0; DOF],
            action_time: 0.0,
        }
    }
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
    let actuator_names: [String; DOF] = std::array::from_fn(|i| {
        format!("{side}_joint{}_ctrl", i + 1)
    });
    let set_ctrl_topic = format!("set_ctrl_arm_{side}");
    // Unique instance_id per arm side so concurrent left+right arms don't
    // collide on the peppylib publisher registry. peppylib scopes by
    // (node_name, instance_id, topic); if it ever changes to (node_name, topic)
    // the second instance's emit would silently shadow the first.
    let instance_id = format!("openarm01_arm_{side}_setctrl_pub");

    let mut action_handle = move_arm_joints::ActionHandle::expose(&runner)
        .await
        .expect("expose move_arm_joints");

    let pending: Arc<StdMutex<Option<AcceptedGoal>>> = Arc::new(StdMutex::new(None));

    loop {
        let pending_for_handler = pending.clone();
        let goal_request = action_handle.handle_goal_next_request(move |req| {
            let positions = req.data.joint_positions;
            if let Err(why) = validate_positions(&positions) {
                warn!("move_arm_joints: rejecting goal — {why}");
                return Ok(move_arm_joints::GoalResponse::new(false));
            }
            let mut slot = pending_for_handler.lock().unwrap();
            if slot.is_some() {
                warn!("move_arm_joints: rejecting goal — another goal already in flight");
                return Ok(move_arm_joints::GoalResponse::new(false));
            }
            *slot = Some(AcceptedGoal {
                target_positions: positions,
                feedback_period: feedback_period(req.data.feedback_frequency),
            });
            info!(
                "move_arm_joints: accepted goal target={positions:?} feedback_hz={}",
                req.data.feedback_frequency
            );
            Ok(move_arm_joints::GoalResponse::new(true))
        });
        tokio::select! {
            _ = token.cancelled() => break,
            result = goal_request => {
                if let Err(e) = result {
                    error!("move_arm_joints goal: {e}");
                    continue;
                }
            }
        }

        let goal = pending.lock().unwrap().take();
        let result = match goal {
            Some(g) => {
                run_control_loop(
                    &handle, &daemon, &state,
                    &set_ctrl_topic, &instance_id, &actuator_names,
                    &action_handle, &token, g,
                ).await
            }
            None => continue,
        };

        let stash: Arc<StdMutex<Option<MotionResult>>> = Arc::new(StdMutex::new(Some(result)));
        let stash_for_handler = stash.clone();
        let result_request = action_handle.handle_result_next_request(move |_req| {
            let r = stash_for_handler.lock().unwrap().take().unwrap_or_default();
            Ok(move_arm_joints::ResultResponse::new(
                r.success,
                r.message,
                r.final_positions,
                r.action_time,
            ))
        });
        tokio::select! {
            _ = token.cancelled() => break,
            result = result_request => {
                if let Err(e) = result {
                    error!("move_arm_joints result: {e}");
                }
            }
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
    action_handle: &move_arm_joints::ActionHandle,
    token: &CancellationToken,
    goal: AcceptedGoal,
) -> MotionResult {
    let start = Instant::now();
    let mut last_feedback = Instant::now();
    let mut window_anchor: Option<f64> = None;
    let mut iter: u32 = 0;

    loop {
        // Re-publish ctrl every tick (5ms = 200 Hz per arm side during active
        // motion). peppylib Standard QoS is best-effort, so a single dropped
        // message would otherwise stall the arm; republishing makes the path
        // self-healing. Idempotent.
        if let Err(e) = publish_set_ctrl(
            handle, daemon, set_ctrl_topic, instance_id, actuator_names,
            &goal.target_positions,
        ).await {
            warn!("set_ctrl publish: {e}");
        }

        let elapsed = start.elapsed();
        let elapsed_secs = elapsed.as_secs_f64();

        let latest = { state.joint_states.lock().await.clone() };
        let snap = match latest {
            Some(s) if s.positions.len() == DOF => s,
            Some(s) => {
                // Side-mismatched cache or unexpected DOF — log and wait.
                warn!(
                    "move_arm_joints: cache has {} positions, expected {DOF} — waiting",
                    s.positions.len()
                );
                if elapsed > MOTION_TIMEOUT {
                    return MotionResult {
                        success: false,
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
                    _ = tokio::time::sleep(FEEDBACK_LOOP_TICK) => continue,
                }
            }
            None => {
                // No telemetry yet — wait, but honour timeout so we don't
                // hang indefinitely if robot_initializer never publishes.
                if elapsed > MOTION_TIMEOUT {
                    return MotionResult {
                        success: false,
                        message: "no telemetry from robot_initializer".into(),
                        final_positions: [0.0; DOF],
                        action_time: elapsed_secs,
                    };
                }
                tokio::select! {
                    _ = token.cancelled() => return cancelled(elapsed_secs, [0.0; DOF]),
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
        let motion_metric: f64 = current.iter().map(|q| q.abs()).sum();

        iter += 1;
        let stalled = if iter % STALL_LOOKBACK_ITERS == 0 {
            let was_stalled = window_anchor
                .map(|prev| (motion_metric - prev).abs() < STALL_EPSILON_RAD)
                .unwrap_or(false);
            window_anchor = Some(motion_metric);
            was_stalled
        } else {
            false
        };

        if last_feedback.elapsed() >= goal.feedback_period {
            if let Err(e) = action_handle.emit_feedback(current, elapsed_secs).await {
                warn!("feedback: {e}");
            }
            last_feedback = Instant::now();
        }

        if within_tolerance {
            return MotionResult {
                success: true,
                message: "reached".into(),
                final_positions: current,
                action_time: elapsed_secs,
            };
        }
        if stalled {
            // Stall on the arm is NOT success the way it is on the gripper
            // (a gripper at a physical limit has done its job). For the arm
            // a stall means the goal couldn't be reached; report failure
            // with the final pose so the caller can decide what to do.
            return MotionResult {
                success: false,
                message: "stalled before reaching target".into(),
                final_positions: current,
                action_time: elapsed_secs,
            };
        }
        if elapsed > MOTION_TIMEOUT {
            return MotionResult {
                success: false,
                message: "timeout".into(),
                final_positions: current,
                action_time: elapsed_secs,
            };
        }

        tokio::select! {
            _ = token.cancelled() => return cancelled(elapsed_secs, current),
            _ = tokio::time::sleep(FEEDBACK_LOOP_TICK) => {}
        }
    }
}

fn cancelled(action_time: f64, final_positions: [f64; DOF]) -> MotionResult {
    MotionResult {
        success: false,
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
    let payload = SetCtrlPayload { actuator_values: values };
    let bytes = serde_json::to_vec(&payload).map_err(|e| e.to_string())?;

    TopicMessenger::emit(
        handle,
        &daemon.core_node_name,
        instance_id,
        ARM_NODE_NAME,
        topic,
        QoSProfile::Standard,
        Payload::from(bytes),
    )
    .await
    .map_err(|e| e.to_string())
}
