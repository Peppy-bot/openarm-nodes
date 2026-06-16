// Joint-space 7-DOF control mirroring the real driver: reject out-of-limit
// targets, anchor a quintic minimum-jerk trajectory at the current pose, and
// stream (q_des, dq_des) setpoints at 100 Hz. The sim-side actuator plugin
// applies the same MIT gains the real motors run, so motion timing, gravity
// sag, and completion semantics match hardware. Completion is time-based
// (trajectory elapsed), exactly like the real driver — no convergence check.

use std::collections::HashMap;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use peppygen::NodeRunner;
use peppygen::exposed_actions::openarm01_arm::v1::move_arm_joints;
use peppylib::config::QoSProfile;
use peppylib::messaging::SenderTarget;
use peppylib::runtime::CancellationToken;
use peppylib::{MessengerHandle, Payload, TopicMessenger, TopicPublisher};
use serde::Serialize;
use sim_bridge_core::DaemonState;
use tracing::{error, info, warn};

use crate::config::ArmId;
use crate::state::SharedState;
use crate::trajectory::{ARM_DOF as DOF, JointVec, Trajectory};

// Real-driver defaults (openarm01_arm peppy.json5): 100 Hz control cycle,
// 30 s motion timeout, per-joint peak velocities (OpenArm V1.0 URDF joint
// velocity limits) sizing the quintic duration.
const CYCLE_PERIOD: Duration = Duration::from_millis(10);
const MOTION_TIMEOUT: Duration = Duration::from_secs(30);
const MAX_JOINT_VELOCITY_RAD_S: JointVec = [
    16.754666, 16.754666, 5.445426, 5.445426, 20.943946, 20.943946, 20.943946,
];

// ~500 ms of dropped publishes at 100 Hz → the arm isn't being commanded;
// bail instead of playing the trajectory into the void.
const MAX_CONSECUTIVE_PUBLISH_FAILURES: u32 = 50;

const ARM_NODE_NAME: &str = "openarm01_arm";

// Keys are joint names (openarm_<side>_joint{1..7}). The sim-side actuator
// plugin owns the MIT gains and combines q_des/dq_des into motor torque.
#[derive(Serialize)]
struct SetCtrlPayload<'a> {
    actuator_values: HashMap<&'a str, f64>,
    velocity_values: HashMap<&'a str, f64>,
}

struct MotionResult {
    success: bool,
    is_cancelled: bool,
    message: String,
    final_positions: JointVec,
    action_time: f64,
}

fn feedback_period(freq_hz: u32) -> Duration {
    Duration::from_micros(1_000_000 / freq_hz.max(1) as u64)
}

// Mirrors the real driver's target_in_limits: every joint inside the model's
// range, non-finite rejected. Limits come from telemetry (MJCF / USD ranges).
fn target_in_limits(target: &JointVec, limits: &[(f64, f64)]) -> bool {
    target
        .iter()
        .zip(limits.iter())
        .all(|(&q, &(lo, hi))| q.is_finite() && q >= lo && q <= hi)
}

fn snapshot_positions(state: &Arc<SharedState>) -> Option<JointVec> {
    let guard = state.joint_states.lock().unwrap_or_else(|p| p.into_inner());
    guard.as_ref().and_then(|s| s.positions.as_slice().try_into().ok())
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
    let actuator_names: Arc<[String; DOF]> = Arc::new(std::array::from_fn(|i| {
        format!("openarm_{side}_joint{}", i + 1)
    }));
    let set_ctrl_topic: Arc<str> = Arc::from(format!("set_ctrl_arm_{side}").as_str());
    // Unique instance_id per arm side so concurrent left+right arms don't
    // collide on the peppylib publisher registry.
    let instance_id: Arc<str> = Arc::from(format!("openarm01_arm_{side}_setctrl_pub").as_str());

    // Declare the set_ctrl publisher once; the per-tick control loop reuses it.
    let target = match SenderTarget::node(ARM_NODE_NAME, "v1") {
        Ok(target) => target,
        Err(e) => {
            error!("invalid set_ctrl target: {e}");
            return;
        }
    };
    let set_ctrl_pub = match TopicMessenger::declare_publisher(
        &handle,
        &daemon.core_node_name,
        &instance_id,
        target,
        None,
        &set_ctrl_topic,
        QoSProfile::Standard,
    )
    .await
    {
        Ok(publisher) => publisher,
        Err(e) => {
            error!("declare set_ctrl publisher: {e}");
            return;
        }
    };

    let mut action_handle = move_arm_joints::ActionHandle::expose(&runner)
        .await
        .expect("expose move_arm_joints");

    // Single-flight gate, same as the real driver: a goal arriving mid-motion
    // is actively rejected rather than queued to run stale afterwards.
    let busy = Arc::new(AtomicBool::new(false));
    // Notified when a motion clears the gate, so the shutdown hook can hold
    // teardown until an in-flight goal has delivered its terminal result.
    let idle = Arc::new(tokio::sync::Notify::new());

    {
        let busy = busy.clone();
        let idle = idle.clone();
        runner.on_shutdown(async move {
            while busy.load(Ordering::Acquire) {
                let notified = idle.notified();
                tokio::pin!(notified);
                notified.as_mut().enable();
                if !busy.load(Ordering::Acquire) {
                    break;
                }
                notified.await;
            }
        });
    }

    loop {
        let state_for_decider = state.clone();
        let busy_for_decider = busy.clone();
        let goal_request =
            action_handle.handle_goal_next_request(move |req: &move_arm_joints::GoalRequest| {
                let limits = state_for_decider
                    .joint_limits
                    .lock()
                    .unwrap_or_else(|p| p.into_inner())
                    .clone();
                let Some(limits) = limits.filter(|l| l.len() == DOF) else {
                    return Ok(move_arm_joints::GoalResponse::reject(
                        "arm telemetry not ready",
                    ));
                };
                if !target_in_limits(&req.data.joint_positions, &limits) {
                    return Ok(move_arm_joints::GoalResponse::reject(
                        "target joint positions out of range",
                    ));
                }
                if !(req.data.duration_s.is_finite() && req.data.duration_s >= 0.0) {
                    return Ok(move_arm_joints::GoalResponse::reject(
                        "duration_s must be finite and >= 0",
                    ));
                }
                if busy_for_decider
                    .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
                    .is_err()
                {
                    return Ok(move_arm_joints::GoalResponse::reject(
                        "arm is already executing a motion",
                    ));
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

        // Spawn the motion so the accept loop keeps listening (and rejecting)
        // during execution — mirrors the real driver's structure.
        let set_ctrl_pub = set_ctrl_pub.clone();
        let state = state.clone();
        let token = token.clone();
        let busy = busy.clone();
        let idle = idle.clone();
        let actuator_names = actuator_names.clone();
        tokio::spawn(async move {
            let result = run_control_loop(
                &set_ctrl_pub,
                &state,
                &actuator_names,
                &goal_ctx,
                &token,
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
            busy.store(false, Ordering::Release);
            idle.notify_waiters();
        });
    }
}

async fn run_control_loop(
    set_ctrl_pub: &TopicPublisher,
    state: &Arc<SharedState>,
    actuator_names: &[String; DOF],
    goal_ctx: &move_arm_joints::GoalContext,
    token: &CancellationToken,
) -> MotionResult {
    let target = goal_ctx.request().data.joint_positions;
    let feedback_period = feedback_period(goal_ctx.request().data.feedback_frequency);
    // Trajectory::new floors the requested duration at the per-joint
    // velocity-limit duration — a too-fast request (or 0 = no preference) is
    // slowed to the fastest safe move rather than rejected (interface contract).
    let duration_s = goal_ctx.request().data.duration_s;

    // Anchor the trajectory at the current pose, like the real driver anchors
    // at the measured CAN state. The decider guaranteed telemetry exists.
    let Some(q_start) = snapshot_positions(state) else {
        return MotionResult {
            success: false,
            is_cancelled: false,
            message: "telemetry lost before motion start".into(),
            final_positions: [0.0; DOF],
            action_time: 0.0,
        };
    };

    info!(
        "move_arm_joints: start={q_start:.3?} target={target:.3?}",
    );
    let trajectory = Trajectory::new(q_start, target, MAX_JOINT_VELOCITY_RAD_S, duration_s);
    let start = trajectory.motion_start;
    let mut last_feedback = Instant::now();
    let mut consecutive_publish_failures: u32 = 0;

    loop {
        let cycle_start = Instant::now();
        let (q_des, dq_des) = trajectory.sample(cycle_start);

        match publish_set_ctrl(set_ctrl_pub, actuator_names, &q_des, &dq_des).await
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
                        final_positions: snapshot_positions(state).unwrap_or([0.0; DOF]),
                        action_time: start.elapsed().as_secs_f64(),
                    };
                }
            }
        }

        let elapsed = start.elapsed();
        let elapsed_secs = elapsed.as_secs_f64();
        let positions = snapshot_positions(state).unwrap_or(q_start);

        if last_feedback.elapsed() >= feedback_period {
            if let Err(e) = goal_ctx.publish_feedback(positions, elapsed_secs).await {
                warn!("feedback: {e}");
            }
            last_feedback = Instant::now();
        }

        // Time-based completion, exactly like the real driver: the trajectory
        // has played out and the servo holds the final setpoint. No
        // convergence check — gravity sag is real behavior, not failure.
        if trajectory.is_complete(cycle_start) {
            return MotionResult {
                success: true,
                is_cancelled: false,
                message: "trajectory complete".into(),
                final_positions: positions,
                action_time: elapsed_secs,
            };
        }
        if elapsed > MOTION_TIMEOUT {
            return MotionResult {
                success: false,
                is_cancelled: false,
                message: "timeout".into(),
                final_positions: positions,
                action_time: elapsed_secs,
            };
        }

        let cycle_budget = CYCLE_PERIOD.saturating_sub(cycle_start.elapsed());
        tokio::select! {
            _ = token.cancelled() => return cancelled(elapsed_secs, positions),
            _ = goal_ctx.cancel_signal() => return cancelled(elapsed_secs, positions),
            _ = tokio::time::sleep(cycle_budget) => {}
        }
    }
}

fn cancelled(action_time: f64, final_positions: JointVec) -> MotionResult {
    MotionResult {
        success: false,
        is_cancelled: true,
        message: "cancelled".into(),
        final_positions,
        action_time,
    }
}

async fn publish_set_ctrl(
    publisher: &TopicPublisher,
    actuator_names: &[String; DOF],
    q_des: &JointVec,
    dq_des: &JointVec,
) -> std::result::Result<(), String> {
    let mut positions: HashMap<&str, f64> = HashMap::with_capacity(DOF);
    let mut velocities: HashMap<&str, f64> = HashMap::with_capacity(DOF);
    for i in 0..DOF {
        positions.insert(actuator_names[i].as_str(), q_des[i]);
        velocities.insert(actuator_names[i].as_str(), dq_des[i]);
    }
    let payload = SetCtrlPayload {
        actuator_values: positions,
        velocity_values: velocities,
    };
    let bytes = serde_json::to_vec(&payload).map_err(|e| e.to_string())?;
    publisher
        .publish(Payload::from(bytes))
        .await
        .map_err(|e| e.to_string())
}
