use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use peppygen::NodeRunner;
use peppygen::exposed_actions::move_arm_joints;
use tracing::{error, info};

use openarm_can::{ArmCan, v10};
use crate::trajectory::Trajectory;

struct AcceptedGoal {
    target: v10::JointVec,
    feedback_frequency: u32,
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

impl Default for MotionResult {
    fn default() -> Self {
        Self {
            success: false,
            message: "no result".into(),
            final_joint_positions: [0.0; v10::ARM_DOF],
            action_time: 0.0,
        }
    }
}

/// Spawn-and-loop entry for the move_arm_joints action: accept a goal, run the
/// trajectory-tracking control loop, return the result. Re-enters the goal loop
/// after each completed (or rejected) goal.
pub async fn run_move_arm_joints(
    runner: Arc<NodeRunner>,
    arm: Arc<Mutex<ArmCan>>,
    cfg: ControlConfig,
) {
    let mut handle = move_arm_joints::ActionHandle::expose(&runner)
        .await
        .expect("expose move_arm_joints");

    // In-process only; relies on the file lock in main.rs for single-instance.
    // Revisit once PEPPYOS-75 replaces the file lock.
    let pending: Arc<Mutex<Option<AcceptedGoal>>> = Arc::new(Mutex::new(None));

    loop {
        // 1. Wait for a goal request.
        let pending_for_handler = pending.clone();
        if let Err(e) = handle
            .handle_goal_next_request(move |req| {
                if req.data.joint_positions.len() != v10::ARM_DOF {
                    return Ok(move_arm_joints::GoalResponse::new(false));
                }
                // Reject if a motion is already pending pickup.
                let mut slot = pending_for_handler.lock().unwrap();
                if slot.is_some() {
                    return Ok(move_arm_joints::GoalResponse::new(false));
                }
                // Already validated the slice is of the right length.
                let target = req.data.joint_positions.as_slice().try_into().unwrap();
                *slot = Some(AcceptedGoal {
                    target,
                    feedback_frequency: req.data.feedback_frequency.clamp(1, 1000),
                });
                Ok(move_arm_joints::GoalResponse::new(true))
            })
            .await
        {
            error!("move_arm_joints goal: {e}");
            continue;
        }

        // 2. If accepted, run the control loop here.
        let goal = pending.lock().unwrap().take();
        let result = match goal {
            Some(g) => run_control_loop(&arm, &handle, &cfg, g).await,
            None => continue, // goal was rejected
        };

        // 3. Wait for the matching result request and return the result.
        let stash: Arc<Mutex<Option<MotionResult>>> = Arc::new(Mutex::new(Some(result)));
        let stash_for_handler = stash.clone();
        if let Err(e) = handle
            .handle_result_next_request(move |_req| {
                let r = stash_for_handler.lock().unwrap().take().unwrap_or_default();
                Ok(move_arm_joints::ResultResponse {
                    success: r.success,
                    message: r.message,
                    final_joint_positions: r.final_joint_positions,
                    action_time: r.action_time,
                })
            })
            .await
        {
            error!("move_arm_joints result: {e}");
        }
    }
}

fn fmt_joints(v: &v10::JointVec) -> String {
    let parts: Vec<String> = v.iter().map(|x| format!("{:.3}", x)).collect();
    format!("[{}]", parts.join(", "))
}

async fn run_control_loop(
    arm: &Arc<Mutex<ArmCan>>,
    handle: &move_arm_joints::ActionHandle,
    cfg: &ControlConfig,
    goal: AcceptedGoal,
) -> MotionResult {
    // Anchor the trajectory at the current joint positions.
    let q_start = {
        let mut a = arm.lock().unwrap();
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
    let feedback_period = Duration::from_micros((1_000_000 / goal.feedback_frequency as u64).max(1));
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
            let mut a = arm.lock().unwrap();
            a.mit_control(&cfg.kp, &cfg.kd, &q_des, &dq_des, &zero_tau);
            a.refresh_all();
            a.recv_all(cfg.recv_timeout_us);
            a.get_state().positions
        };

        let elapsed = start.elapsed();
        let elapsed_secs = elapsed.as_secs_f64();

        if last_feedback.elapsed() >= feedback_period {
            let _ = handle.emit_feedback(positions, elapsed_secs).await;
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
