use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use peppygen::NodeRunner;
use peppygen::exposed_actions::openarm01_arm::v1::move_arm_joints;
use tracing::{error, info};

use openarm_can::{ArmCan, v10};
use crate::trajectory::Trajectory;

struct AcceptedGoal {
    target: v10::JointVec,
    feedback_period: Duration,
}

#[derive(Clone)]
pub struct ControlConfig {
    pub kp: v10::JointVec,
    pub kd: v10::JointVec,
    pub cycle_period: Duration,
    pub recv_timeout_us: i32,
    pub motion_timeout: Duration,
    pub max_joint_velocity_rad_s: v10::JointVec,
    pub min_motion_time_s: f64,
}

struct MotionResult {
    success: bool,
    message: String,
    final_joint_positions: v10::JointVec,
    action_time: f64,
}

impl MotionResult {
    fn completed(positions: v10::JointVec, action_time: f64) -> Self {
        Self {
            success: true,
            message: "trajectory complete".into(),
            final_joint_positions: positions,
            action_time,
        }
    }

    fn timed_out(positions: v10::JointVec, action_time: f64) -> Self {
        Self {
            success: false,
            message: "timeout".into(),
            final_joint_positions: positions,
            action_time,
        }
    }
}

/// Spawn-and-loop entry for the move_arm_joints action: accept a goal, run the
/// trajectory-tracking control loop, complete the goal with its result. Re-enters
/// the goal loop afterwards. A goal arriving mid-motion is rejected (single-flight);
/// the file lock in main.rs guarantees single-instance.
pub async fn run_move_arm_joints(
    runner: Arc<NodeRunner>,
    arm: Arc<Mutex<ArmCan>>,
    cfg: ControlConfig,
) {
    let mut handle = move_arm_joints::ActionHandle::expose(&runner)
        .await
        .expect("expose move_arm_joints");

    // Single-flight gate: reject a new goal while one is still executing. The
    // motion runs in a spawned task so the loop keeps listening (and rejecting)
    // rather than silently queueing a goal that would run stale once the
    // current one finishes.
    let busy = Arc::new(AtomicBool::new(false));

    loop {
        let ctx = match handle
            .handle_goal_next_request(|req| {
                // Reject targets outside the arm's joint limits (also rejects
                // NaN/inf, which Limit::contains treats as out of range).
                if !target_in_limits(&req.data.joint_positions) {
                    return Ok(move_arm_joints::GoalResponse::reject(
                        "target joint positions out of range",
                    ));
                }
                // Atomically claim the slot; reject if a motion already holds it.
                if busy
                    .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
                    .is_err()
                {
                    Ok(move_arm_joints::GoalResponse::reject(
                        "arm is already executing a motion",
                    ))
                } else {
                    Ok(move_arm_joints::GoalResponse::accept())
                }
            })
            .await
        {
            Ok(Some(ctx)) => ctx,
            Ok(None) => break, // action closed (node shutting down)
            Err(e) => {
                error!("move_arm_joints goal: {e}");
                continue;
            }
        };

        let goal = AcceptedGoal {
            target: ctx.request().data.joint_positions,
            feedback_period: feedback_period(ctx.request().data.feedback_frequency),
        };

        let arm = Arc::clone(&arm);
        let cfg = cfg.clone();
        let busy = Arc::clone(&busy);
        tokio::spawn(async move {
            let result = run_control_loop(&arm, &ctx, &cfg, goal).await;
            if let Err(e) = ctx
                .complete(
                    result.success,
                    result.message,
                    result.final_joint_positions,
                    result.action_time,
                )
                .await
            {
                error!("move_arm_joints complete: {e}");
            }
            busy.store(false, Ordering::Release);
        });
    }
}

async fn run_control_loop(
    arm: &Arc<Mutex<ArmCan>>,
    ctx: &move_arm_joints::GoalContext,
    cfg: &ControlConfig,
    goal: AcceptedGoal,
) -> MotionResult {
    // Anchor the trajectory at the current joint positions.
    let q_start = {
        let mut a = arm.lock().unwrap_or_else(|e| e.into_inner());
        a.refresh_all();
        a.recv_all(cfg.recv_timeout_us);
        a.get_state().positions
    };

    info!(
        "move_arm_joints: start={} target={}",
        fmt_joints(&q_start),
        fmt_joints(&goal.target),
    );
    let trajectory = Trajectory::new(q_start, goal.target, cfg.max_joint_velocity_rad_s, cfg.min_motion_time_s);
    let start = trajectory.motion_start;
    let mut last_feedback = Instant::now();
    let mut feedback_failures: u32 = 0;
    // tau (feedforward torque) is zero. The default ROS2 control path also leaves
    // tau_commands_ at zero — JointTrajectoryController populates pos/vel command
    // interfaces only, and v10_simple_hardware just forwards what's there. Adding
    // gravity compensation would require an inverse-dynamics model (URDF + a solver
    // like pinocchio or KDL) and would populate this slot with g(q).
    let zero_tau = [0.0f64; v10::ARM_DOF];

    loop {
        let cycle_start = Instant::now();
        let (q_des, dq_des) = trajectory.sample(cycle_start);

        let positions = {
            let mut a = arm.lock().unwrap_or_else(|e| e.into_inner());
            a.mit_control(&cfg.kp, &cfg.kd, &q_des, &dq_des, &zero_tau);
            a.refresh_all();
            a.recv_all(cfg.recv_timeout_us);
            a.get_state().positions
        };

        let elapsed = start.elapsed();
        let elapsed_secs = elapsed.as_secs_f64();

        if last_feedback.elapsed() >= goal.feedback_period {
            // Warn once per motion if it starts failing, then stay quiet.
            if let Err(e) = ctx.publish_feedback(positions, elapsed_secs).await {
                feedback_failures += 1;
                if feedback_failures == 1 {
                    tracing::warn!("move_arm_joints feedback publish failing, suppressing repeats: {e}");
                }
            }
            last_feedback = Instant::now();
        }

        if trajectory.is_complete(cycle_start) {
            return MotionResult::completed(positions, elapsed_secs);
        }
        if elapsed > cfg.motion_timeout {
            return MotionResult::timed_out(positions, elapsed_secs);
        }

        let cycle_elapsed = cycle_start.elapsed();
        if cycle_elapsed < cfg.cycle_period {
            tokio::time::sleep(cfg.cycle_period - cycle_elapsed).await;
        } else if cycle_elapsed > cfg.cycle_period.mul_f64(1.2) {
            tracing::warn!(
                "control loop overrun: {:.1}ms (budget {:.1}ms)",
                cycle_elapsed.as_secs_f64() * 1000.0,
                cfg.cycle_period.as_secs_f64() * 1000.0,
            );
        }
    }
}

/// True if every joint target lies within its V10 position limit. Non-finite
/// values (NaN/inf) fall outside any range, so they are rejected too.
fn target_in_limits(target: &v10::JointVec) -> bool {
    v10::ARM_JOINT_LIMITS
        .iter()
        .zip(target)
        .all(|(limit, &q)| limit.contains(q))
}

/// Convert a feedback frequency in Hz to a Duration. Floors at 1 Hz to avoid divide-by-zero.
fn feedback_period(freq_hz: u32) -> Duration {
    Duration::from_micros(1_000_000 / freq_hz.max(1) as u64)
}

fn fmt_joints(v: &v10::JointVec) -> String {
    let parts: Vec<String> = v.iter().map(|x| format!("{:.3}", x)).collect();
    format!("[{}]", parts.join(", "))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn feedback_period_floors_zero_freq() {
        assert_eq!(feedback_period(0), Duration::from_secs(1));
    }

    #[test]
    fn target_in_limits_accepts_home_and_rejects_out_of_range() {
        // Home pose (all zeros) is inside every joint limit.
        assert!(target_in_limits(&[0.0; v10::ARM_DOF]));

        // A single joint past its upper bound fails the whole target.
        let mut over = [0.0; v10::ARM_DOF];
        over[3] = v10::ARM_JOINT_LIMITS[3].upper + 0.1;
        assert!(!target_in_limits(&over));

        // Non-finite values are rejected.
        let mut nan = [0.0; v10::ARM_DOF];
        nan[0] = f64::NAN;
        assert!(!target_in_limits(&nan));
        let mut inf = [0.0; v10::ARM_DOF];
        inf[0] = f64::INFINITY;
        assert!(!target_in_limits(&inf));
    }
}
