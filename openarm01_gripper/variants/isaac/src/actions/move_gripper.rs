// move_gripper action handler — peppylib-mediated control direction.
//
// Each goal:
//   1. Publishes raw set_ctrl_gripper_<side> once via peppylib (the bridge
//      extension on robot_initializer:mujoco subscribes and applies before
//      the next mj_step). Both fingers driven to the same target.
//   2. Enters a feedback loop reading the latest gripper_state from the
//      shared cache (populated by pipeline::telemetry).
//   3. Streams typed feedback at the requested rate; detects convergence
//      (worst-finger error < POSITION_TOLERANCE_M) or stall (sum-of-motion
//      across a ~500ms window < STALL_EPSILON_M).
//   4. Returns result.
//
// Convergence + stall logic carries from the bus-era implementation
// unchanged; only the data path (snapshot from shared cache, publish raw
// peppylib) differs.

use std::collections::HashMap;
use std::sync::{Arc, Mutex as StdMutex};
use std::time::{Duration, Instant};

use peppygen::NodeRunner;
use peppygen::exposed_actions::move_gripper;
use peppylib::config::QoSProfile;
use peppylib::{MessengerHandle, Payload, TopicMessenger};
use serde::Serialize;
use sim_bridge_core::DaemonState;
use tracing::{error, warn};

use crate::config::GripperId;
use crate::state::SharedState;

// Per-finger tolerance (meters) for "reached target". `position` is the
// per-finger displacement (0 closed → ~0.044 fully open in the openarm MJCF);
// each finger is independently driven to that value.
const POSITION_TOLERANCE_M: f64 = 0.002;
const MOTION_TIMEOUT: Duration = Duration::from_secs(30);

// Stall detection: when the fingers can't reach the requested target (e.g.,
// pressed against each other at full close, jammed by an object, motor at
// its limit), qpos stops changing. We compare current sum against the sum
// from ~500ms ago. STALL_LOOKBACK_ITERS × FEEDBACK_LOOP_TICK = 500ms window;
// 0.5mm-over-500ms = 1mm/s — below that is treated as stalled.
const STALL_LOOKBACK_ITERS: u32 = 100;
const STALL_EPSILON_M: f64 = 5e-4;
const FEEDBACK_LOOP_TICK: Duration = Duration::from_millis(5);

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
    message: String,
    final_positions: Vec<f64>,
    action_time: f64,
}

impl Default for MotionResult {
    fn default() -> Self {
        Self {
            success: false,
            message: "no result".into(),
            final_positions: vec![],
            action_time: 0.0,
        }
    }
}

fn feedback_period(freq_hz: u32) -> Duration {
    Duration::from_micros(1_000_000 / freq_hz.max(1) as u64)
}

pub async fn run(
    runner: Arc<NodeRunner>,
    gripper_id: GripperId,
    state: Arc<SharedState>,
) {
    let side = gripper_id.side_word();
    let actuator_names = [
        format!("{side}_finger1_ctrl"),
        format!("{side}_finger2_ctrl"),
    ];
    let set_ctrl_topic = format!("set_ctrl_gripper_{side}");
    // Unique instance_id per gripper side so concurrent left+right grippers
    // don't collide on the peppylib publisher registry.
    let instance_id = format!("openarm01_gripper_{side}_setctrl_pub");

    let daemon = match peppylib::info(&runner, None).await {
        Ok(info) => DaemonState {
            core_node_name: info.core_node_name,
            messaging_port: info.messaging_port,
        },
        Err(e) => {
            error!("move_gripper: peppylib::info failed: {e}");
            return;
        }
    };

    let handle = match MessengerHandle::from_host_port("localhost", daemon.messaging_port).await {
        Ok(h) => h,
        Err(e) => {
            error!("move_gripper: peppylib connect failed: {e}");
            return;
        }
    };

    let mut action_handle = move_gripper::ActionHandle::expose(&runner)
        .await
        .expect("expose move_gripper");

    let pending: Arc<StdMutex<Option<AcceptedGoal>>> = Arc::new(StdMutex::new(None));

    loop {
        let pending_for_handler = pending.clone();
        if let Err(e) = action_handle
            .handle_goal_next_request(move |req| {
                let pos_m = req.data.position;
                if !(0.0..=0.044).contains(&pos_m) {
                    return Ok(move_gripper::GoalResponse::new(false));
                }
                let mut slot = pending_for_handler.lock().unwrap();
                if slot.is_some() {
                    return Ok(move_gripper::GoalResponse::new(false));
                }
                *slot = Some(AcceptedGoal {
                    target_position_m: pos_m,
                    feedback_period: feedback_period(req.data.feedback_frequency),
                });
                Ok(move_gripper::GoalResponse::new(true))
            })
            .await
        {
            error!("move_gripper goal: {e}");
            continue;
        }

        let goal = pending.lock().unwrap().take();
        let result = match goal {
            Some(g) => {
                run_control_loop(
                    &handle, &daemon, &state,
                    &set_ctrl_topic, &instance_id, &actuator_names,
                    &action_handle, g,
                ).await
            }
            None => continue,
        };

        let stash: Arc<StdMutex<Option<MotionResult>>> = Arc::new(StdMutex::new(Some(result)));
        let stash_for_handler = stash.clone();
        if let Err(e) = action_handle
            .handle_result_next_request(move |_req| {
                let r = stash_for_handler.lock().unwrap().take().unwrap_or_default();
                Ok(move_gripper::ResultResponse {
                    success: r.success,
                    message: r.message,
                    final_joint_positions: r.final_positions,
                    action_time: r.action_time,
                })
            })
            .await
        {
            error!("move_gripper result: {e}");
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
    action_handle: &move_gripper::ActionHandle,
    goal: AcceptedGoal,
) -> MotionResult {
    // Publish the ctrl target once. The bridge extension's
    // ActuatorCtrlBridge holds the latest value and applies it on every
    // mj_step until the next publish — matches the actuator-latch behaviour
    // of the bus-era write_ctrl.
    if let Err(e) = publish_set_ctrl(
        handle, daemon, set_ctrl_topic, instance_id, actuator_names, goal.target_position_m,
    ).await {
        warn!("set_ctrl publish: {e}");
    }

    let start = Instant::now();
    let mut last_feedback = Instant::now();
    let mut window_anchor: Option<f64> = None;
    let mut iter: u32 = 0;

    loop {
        let elapsed = start.elapsed();
        let elapsed_secs = elapsed.as_secs_f64();

        let latest = { state.gripper_state.lock().await.clone() };
        let snap = match latest {
            Some(s) => s,
            None => {
                // No telemetry yet — wait, but honour timeout so we don't
                // hang indefinitely if robot_initializer never publishes.
                if elapsed > MOTION_TIMEOUT {
                    return MotionResult {
                        success: false,
                        message: "no telemetry from robot_initializer".into(),
                        final_positions: vec![],
                        action_time: elapsed_secs,
                    };
                }
                tokio::time::sleep(FEEDBACK_LOOP_TICK).await;
                continue;
            }
        };

        // goal.target_position_m is total aperture; each finger holds half.
        // Compare per-finger qpos against the per-finger target.
        let per_finger_target = goal.target_position_m / 2.0;
        let worst_err = snap.positions.iter()
            .map(|&q| (q - per_finger_target).abs())
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
            if let Err(e) = action_handle
                .emit_feedback(snap.positions.clone(), elapsed_secs).await
            {
                warn!("feedback: {e}");
            }
            last_feedback = Instant::now();
        }

        if within_tolerance {
            return MotionResult {
                success: true,
                message: "reached".into(),
                final_positions: snap.positions,
                action_time: elapsed_secs,
            };
        }
        if stalled {
            return MotionResult {
                success: true,
                message: "stalled at physical limit".into(),
                final_positions: snap.positions,
                action_time: elapsed_secs,
            };
        }
        if elapsed > MOTION_TIMEOUT {
            return MotionResult {
                success: false,
                message: "timeout".into(),
                final_positions: snap.positions,
                action_time: elapsed_secs,
            };
        }

        tokio::time::sleep(FEEDBACK_LOOP_TICK).await;
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
    // `value` is the total aperture target per the V10 contract (0.0 closed,
    // 0.044 fully open). Each finger contributes half — drive both to value/2.
    let per_finger = value / 2.0;
    let mut values: HashMap<&str, f64> = HashMap::new();
    values.insert(actuator_names[0].as_str(), per_finger);
    values.insert(actuator_names[1].as_str(), per_finger);
    let payload = SetCtrlPayload { actuator_values: values };
    let bytes = serde_json::to_vec(&payload).map_err(|e| e.to_string())?;

    TopicMessenger::emit(
        handle,
        &daemon.core_node_name,
        instance_id,
        GRIPPER_NODE_NAME,
        topic,
        QoSProfile::Standard,
        Payload::from(bytes),
    )
    .await
    .map_err(|e| e.to_string())
}
