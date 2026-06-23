// Joint-space 7-DOF control mirroring the real driver: anchor a quintic
// minimum-jerk trajectory at the current pose and stream (q_des, dq_des)
// setpoints at the control rate. Joint limits are enforced by the sim engine, so
// an out-of-range target just settles against the model's stop. The sim-side actuator
// plugin applies the same MIT gains the real motors run, so motion timing,
// gravity sag, and completion semantics match hardware. Completion is time-based
// (trajectory elapsed), exactly like the real driver — no convergence check.
//
// The passthrough publisher and the single-flight busy gate are owned by main and
// shared with the follow loop, so a move and the streamed-command follower never
// drive the sim at once.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Instant;

use peppygen::NodeRunner;
use peppygen::exposed_actions::openarm01_arm::v1::move_arm_joints;
use peppylib::TopicPublisher;
use peppylib::runtime::CancellationToken;
use tracing::{error, info, warn};

use crate::config::ControlParams;
use crate::passthrough;
use crate::state::{self, SharedState};
use crate::trajectory::{ARM_DOF as DOF, JointVec, Trajectory};

// ~500 ms of dropped publishes at 100 Hz → the arm isn't being commanded; bail
// instead of playing the trajectory into the void.
const MAX_CONSECUTIVE_PUBLISH_FAILURES: u32 = 50;
const MOTION_TIMEOUT: std::time::Duration = std::time::Duration::from_secs(30);

struct MotionResult {
    success: bool,
    is_cancelled: bool,
    message: String,
    final_positions: JointVec,
    action_time: f64,
}

pub async fn run(
    runner: Arc<NodeRunner>,
    state: Arc<SharedState>,
    token: CancellationToken,
    passthrough_pub: TopicPublisher,
    arm_id: u8,
    busy: Arc<AtomicBool>,
    params: ControlParams,
) {
    let mut action_handle = move_arm_joints::ActionHandle::expose(&runner)
        .await
        .expect("expose move_arm_joints");

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
                // Readiness: the trajectory anchors on the measured pose, so a
                // goal is only acceptable once joint_states telemetry has arrived.
                if state::snapshot_positions(&state_for_decider).is_none() {
                    return Ok(move_arm_joints::GoalResponse::reject(
                        "arm telemetry not ready",
                    ));
                }
                // The sim enforces joint limits; reject only a non-finite target
                // (which would corrupt the trajectory) and a bad duration.
                if !req.data.joint_positions.iter().all(|q| q.is_finite()) {
                    return Ok(move_arm_joints::GoalResponse::reject(
                        "target joint positions must be finite",
                    ));
                }
                if !(req.data.duration_s.is_finite() && req.data.duration_s >= 0.0) {
                    return Ok(move_arm_joints::GoalResponse::reject(
                        "duration_s must be finite and >= 0",
                    ));
                }
                // Latch the single-flight gate: a goal arriving mid-motion (or
                // while the follow loop is driving) is rejected, not queued.
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
        let passthrough_pub = passthrough_pub.clone();
        let state = state.clone();
        let token = token.clone();
        let busy = busy.clone();
        let idle = idle.clone();
        tokio::spawn(async move {
            let result =
                run_control_loop(&passthrough_pub, arm_id, &state, &goal_ctx, &token, &params)
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
    passthrough_pub: &TopicPublisher,
    arm_id: u8,
    state: &Arc<SharedState>,
    goal_ctx: &move_arm_joints::GoalContext,
    token: &CancellationToken,
    params: &ControlParams,
) -> MotionResult {
    let target = goal_ctx.request().data.joint_positions;
    // Trajectory::new floors the requested duration at the per-joint
    // velocity-limit duration — a too-fast request (or 0 = no preference) is
    // slowed to the fastest safe move rather than rejected (interface contract).
    let duration_s = goal_ctx.request().data.duration_s;

    // Anchor the trajectory at the current pose, like the real driver anchors
    // at the measured CAN state. The decider guaranteed telemetry exists.
    let Some(q_start) = state::snapshot_positions(state) else {
        return MotionResult {
            success: false,
            is_cancelled: false,
            message: "telemetry lost before motion start".into(),
            final_positions: [0.0; DOF],
            action_time: 0.0,
        };
    };

    info!("move_arm_joints: start={q_start:.3?} target={target:.3?}");
    let trajectory = Trajectory::new(q_start, target, params.max_joint_velocity, duration_s);
    let start = trajectory.motion_start;
    let mut consecutive_publish_failures: u32 = 0;

    loop {
        let cycle_start = Instant::now();
        let (q_des, dq_des) = trajectory.sample(cycle_start);

        match passthrough::publish(passthrough_pub, arm_id, &q_des, &dq_des).await {
            Ok(()) => consecutive_publish_failures = 0,
            Err(e) => {
                consecutive_publish_failures += 1;
                warn!("passthrough publish failed ({consecutive_publish_failures}): {e}");
                if consecutive_publish_failures >= MAX_CONSECUTIVE_PUBLISH_FAILURES {
                    return MotionResult {
                        success: false,
                        is_cancelled: false,
                        message: "passthrough publish failing: arm not commandable".into(),
                        final_positions: state::snapshot_positions(state).unwrap_or([0.0; DOF]),
                        action_time: start.elapsed().as_secs_f64(),
                    };
                }
            }
        }

        let elapsed = start.elapsed();
        let elapsed_secs = elapsed.as_secs_f64();
        let positions = state::snapshot_positions(state).unwrap_or(q_start);

        // Time-based completion, exactly like the real driver: the trajectory
        // has played out and the servo holds the final setpoint. No convergence
        // check — gravity sag is real behavior, not failure.
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

        let cycle_budget = params.control_period.saturating_sub(cycle_start.elapsed());
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
