// Per-finger control: run a feedback loop on the shared gripper_state cache
// and republish set_ctrl_gripper_<side> every tick to survive best-effort
// QoS drops. Convergence on worst-finger error; stall on per-window motion.

use std::collections::HashMap;
use std::sync::Arc;
use std::time::{Duration, Instant};

use peppygen::NodeRunner;
use peppygen::exposed_actions::openarm01_gripper::v1::move_gripper;
use peppylib::config::QoSProfile;
use peppylib::messaging::SenderTarget;
use peppylib::runtime::CancellationToken;
use peppylib::{MessengerHandle, Payload, TopicMessenger};
use serde::Serialize;
use sim_bridge_core::DaemonState;
use tokio::signal::unix::{SignalKind, signal};
use tracing::{error, info, warn};

use crate::config::GripperId;
use crate::state::SharedState;

// Per-finger tolerance (meters) for "reached target". `position` is the
// per-finger displacement (0 closed → ~0.044 fully open in the openarm MJCF);
// each finger is independently driven to that value.
const POSITION_TOLERANCE_M: f64 = 0.002;
const MOTION_TIMEOUT: Duration = Duration::from_secs(30);

// Sum-of-positions diff over a 500ms window; below STALL_EPSILON_M → stalled.
// Sum is fine for the gripper: both fingers move in the same direction.
const STALL_LOOKBACK_ITERS: u32 = 100;
const STALL_EPSILON_M: f64 = 5e-4;
const FEEDBACK_LOOP_TICK: Duration = Duration::from_millis(5);

// If set_ctrl publishing fails for this many consecutive ticks (one full stall
// window, ~500ms), the gripper isn't being commanded at all — fail the motion
// instead of letting the stall detector report a false "physical limit".
const MAX_CONSECUTIVE_PUBLISH_FAILURES: u32 = STALL_LOOKBACK_ITERS;

const GRIPPER_NODE_NAME: &str = "openarm01_gripper";

#[derive(Serialize)]
struct SetCtrlPayload<'a> {
    actuator_values: HashMap<&'a str, f64>,
}

struct AcceptedGoal {
    target_position_m: f64,
    feedback_period: Duration,
}

struct MotionResult {
    success: bool,
    is_cancelled: bool,
    message: String,
    final_positions: Vec<f64>,
    action_time: f64,
}

fn feedback_period(freq_hz: u32) -> Duration {
    Duration::from_micros(1_000_000 / freq_hz.max(1) as u64)
}

pub async fn run(
    runner: Arc<NodeRunner>,
    gripper_id: GripperId,
    state: Arc<SharedState>,
    token: CancellationToken,
    handle: Arc<MessengerHandle>,
    daemon: DaemonState,
) {
    let side = gripper_id.side_word();
    let actuator_names = [
        format!("openarm_{side}_finger_joint1"),
        format!("openarm_{side}_finger_joint2"),
    ];
    let set_ctrl_topic = format!("set_ctrl_gripper_{side}");
    // Unique instance_id per gripper side so concurrent left+right grippers
    // don't collide on the peppylib publisher registry.
    let instance_id = format!("openarm01_gripper_{side}_setctrl_pub");

    let mut action_handle = move_gripper::ActionHandle::expose(&runner)
        .await
        .expect("expose move_gripper");

    loop {
        // Single-flight: the next handle_goal_next_request only resumes after
        // the previous goal's GoalContext completes.
        let goal_request =
            action_handle.handle_goal_next_request(|req: &move_gripper::GoalRequest| {
                let pos_m = req.data.position;
                if !(0.0..=0.044).contains(&pos_m) {
                    return Ok(move_gripper::GoalResponse::reject(
                        "position out of range [0.0, 0.044]",
                    ));
                }
                Ok(move_gripper::GoalResponse::accept())
            });

        let goal_ctx = tokio::select! {
            _ = token.cancelled() => break,
            result = goal_request => {
                match result {
                    Ok(Some(ctx)) => ctx,
                    Ok(None) => break, // action exposed but shutting down
                    Err(e) => {
                        error!("move_gripper goal: {e}");
                        continue;
                    }
                }
            }
        };

        let goal = AcceptedGoal {
            target_position_m: goal_ctx.request().data.position,
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

        // complete() and complete_cancelled() take identical field sets; the
        // difference is the client-visible status (Completed vs Cancelled).
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
            error!("move_gripper complete: {e}");
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
    actuator_names: &[String; 2],
    goal_ctx: &move_gripper::GoalContext,
    token: &CancellationToken,
    goal: AcceptedGoal,
) -> MotionResult {
    // goal.target_position_m is total aperture; each finger holds half.
    let per_finger = goal.target_position_m / 2.0;

    let start = Instant::now();
    let mut last_feedback = Instant::now();
    let mut window_anchor: Option<f64> = None;
    let mut iter: u32 = 0;
    let mut consecutive_publish_failures: u32 = 0;

    loop {
        // Republish every tick: peppylib Standard is best-effort, so this is
        // the self-healing path. Idempotent. If it keeps failing, bail before
        // convergence/stall reports false success.
        match publish_set_ctrl(
            handle,
            daemon,
            set_ctrl_topic,
            instance_id,
            actuator_names,
            per_finger,
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
                        message: "set_ctrl publish failing — gripper not commandable".into(),
                        final_positions: vec![],
                        action_time: start.elapsed().as_secs_f64(),
                    };
                }
            }
        }

        let elapsed = start.elapsed();
        let elapsed_secs = elapsed.as_secs_f64();

        let latest = { state.gripper_state.lock().await.clone() };
        let snap = match latest {
            Some(s) if !s.positions.is_empty() => s,
            _ => {
                // No usable telemetry yet (no message, or one with empty joint
                // positions) — wait, but honour timeout so we don't hang
                // indefinitely. Empty positions must not reach the convergence
                // math below, where worst_err would fold to 0.0 and falsely
                // report "reached".
                if elapsed > MOTION_TIMEOUT {
                    return MotionResult {
                        success: false,
                        is_cancelled: false,
                        message: "no usable telemetry from robot_initializer".into(),
                        final_positions: vec![],
                        action_time: elapsed_secs,
                    };
                }
                tokio::select! {
                    _ = token.cancelled() => return cancelled(elapsed_secs, vec![]),
                    _ = goal_ctx.cancel_signal() => return cancelled(elapsed_secs, vec![]),
                    _ = tokio::time::sleep(FEEDBACK_LOOP_TICK) => continue,
                }
            }
        };

        let worst_err = snap
            .positions
            .iter()
            .map(|&q| (q - per_finger).abs())
            .fold(0.0_f64, f64::max);
        let within_tolerance = worst_err < POSITION_TOLERANCE_M;
        let motion_metric: f64 = snap.positions.iter().sum();

        iter += 1;
        let stalled = if iter % STALL_LOOKBACK_ITERS == 0 {
            let was_stalled = window_anchor
                .map(|prev| (motion_metric - prev).abs() < STALL_EPSILON_M)
                .unwrap_or(false);
            window_anchor = Some(motion_metric);
            was_stalled
        } else {
            false
        };

        if last_feedback.elapsed() >= goal.feedback_period {
            if let Err(e) = goal_ctx
                .publish_feedback(snap.positions.clone(), elapsed_secs)
                .await
            {
                warn!("feedback: {e}");
            }
            last_feedback = Instant::now();
        }

        if within_tolerance {
            return MotionResult {
                success: true,
                is_cancelled: false,
                message: "reached".into(),
                final_positions: snap.positions,
                action_time: elapsed_secs,
            };
        }
        if stalled {
            return MotionResult {
                success: true,
                is_cancelled: false,
                message: "stalled at physical limit".into(),
                final_positions: snap.positions,
                action_time: elapsed_secs,
            };
        }
        if elapsed > MOTION_TIMEOUT {
            return MotionResult {
                success: false,
                is_cancelled: false,
                message: "timeout".into(),
                final_positions: snap.positions,
                action_time: elapsed_secs,
            };
        }

        tokio::select! {
            _ = token.cancelled() => return cancelled(elapsed_secs, snap.positions),
            _ = goal_ctx.cancel_signal() => return cancelled(elapsed_secs, snap.positions),
            _ = tokio::time::sleep(FEEDBACK_LOOP_TICK) => {}
        }
    }
}

fn cancelled(action_time: f64, final_positions: Vec<f64>) -> MotionResult {
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
    actuator_names: &[String; 2],
    value: f64,
) -> std::result::Result<(), String> {
    let mut values: HashMap<&str, f64> = HashMap::new();
    values.insert(actuator_names[0].as_str(), value);
    values.insert(actuator_names[1].as_str(), value);
    let payload = SetCtrlPayload {
        actuator_values: values,
    };
    let bytes = serde_json::to_vec(&payload).map_err(|e| e.to_string())?;

    // Publish as openarm01_gripper:v1 so real + sim instances look identical
    // to consumers on the bus.
    let target = SenderTarget::node(GRIPPER_NODE_NAME, "v1")
        .map_err(|e| format!("invalid as_target: {e}"))?;

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

/// SIGINT/SIGTERM handler — cancels the control loop, publishes ctrl=0.0 over a
/// short grace window, then exits. Without the cancel, the still-running action
/// loop could overwrite the zero with the last per_finger command between our
/// publish and process exit; without the repeat publishes, a single best-effort
/// drop would leave the bridge holding the last non-zero command indefinitely.
pub async fn shutdown_handler(
    handle: Arc<MessengerHandle>,
    daemon: DaemonState,
    gripper_id: GripperId,
    token: CancellationToken,
) {
    let side = gripper_id.side_word();
    let actuator_names = [
        format!("openarm_{side}_finger_joint1"),
        format!("openarm_{side}_finger_joint2"),
    ];
    let set_ctrl_topic = format!("set_ctrl_gripper_{side}");
    let instance_id = format!("openarm01_gripper_{side}_shutdown_pub");

    let mut sigint = signal(SignalKind::interrupt()).expect("sigint");
    let mut sigterm = signal(SignalKind::terminate()).expect("sigterm");
    tokio::select! {
        _ = sigint.recv() => {},
        _ = sigterm.recv() => {},
    }
    info!(
        "shutdown: cancelling action loop, zeroing ctrl for gripper_id={}",
        gripper_id.as_u8()
    );
    token.cancel();

    const GRACE_TICK: Duration = Duration::from_millis(10);
    const GRACE_REPEATS: u32 = 5;
    for _ in 0..GRACE_REPEATS {
        if let Err(e) = publish_set_ctrl(
            &handle,
            &daemon,
            &set_ctrl_topic,
            &instance_id,
            &actuator_names,
            0.0,
        )
        .await
        {
            warn!("shutdown publish: {e}");
        }
        tokio::time::sleep(GRACE_TICK).await;
    }
    std::process::exit(0);
}
