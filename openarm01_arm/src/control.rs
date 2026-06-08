use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use peppygen::NodeRunner;
use peppygen::exposed_actions::openarm01_arm::v1::move_arm_joints;
use tokio::sync::mpsc;
use tracing::{error, info, warn};

use crate::friction;
use crate::trajectory::Trajectory;
use crate::{ARM_DOF, JointVec};
use srs_model::Limit;
use openarm_can::ArmCan;

/// All-zero joint vector, the zero desired velocity sent alongside a held or
/// commanded position.
const ZERO: JointVec = [0.0; ARM_DOF];

#[derive(Clone)]
pub struct ControlConfig {
    pub kp: JointVec,
    pub kd: JointVec,
    pub cycle_period: Duration,
    pub recv_timeout_us: i32,
    pub motion_timeout: Duration,
    pub max_joint_velocity_rad_s: JointVec,
    pub min_motion_time_s: f64,
    /// This arm's joint position limits, parsed from the URDF (per side, via the
    /// `base_link`). Used to reject out-of-range move targets.
    pub limits: [Limit; ARM_DOF],
}

/// An accepted move goal, handed from the action handler to the single control task.
struct Goal {
    target: JointVec,
    feedback_period: Duration,
    ctx: move_arm_joints::GoalContext,
}

/// What the single control task is doing this tick. Because exactly one task ever
/// writes to the motors, the mode merely selects the command; there is no writer
/// arbitration.
// TODO: add a `Stream` mode for external setpoint streaming at the control rate
// (e.g. an openarm-teleop follower tracking a continuous target stream), alongside
// `Hold` and `Trajectory`.
enum Mode {
    /// Holding a fixed setpoint with gravity/Coriolis/friction feedforward plus PD
    /// (kp/kd): the default at startup (holds the power-on pose) and after every
    /// motion (holds that move's commanded goal). The PD term holds position, so the
    /// arm stays put rather than sagging at rest.
    Hold { setpoint: JointVec },
    /// Tracking a trajectory for an accepted `move_arm_joints` goal.
    Trajectory(ActiveMotion),
}

struct ActiveMotion {
    trajectory: Trajectory,
    ctx: move_arm_joints::GoalContext,
    feedback_period: Duration,
    last_feedback: Instant,
    feedback_failures: u32,
}

struct MotionResult {
    success: bool,
    message: String,
    final_joint_positions: JointVec,
    action_time: f64,
}

impl MotionResult {
    fn completed(positions: JointVec, action_time: f64) -> Self {
        Self {
            success: true,
            message: "trajectory complete".into(),
            final_joint_positions: positions,
            action_time,
        }
    }

    fn timed_out(positions: JointVec, action_time: f64) -> Self {
        Self {
            success: false,
            message: "timeout".into(),
            final_joint_positions: positions,
            action_time,
        }
    }
}

/// Spawn the arm's control: a single task that owns the motors (the only motor
/// writer, so hold and trajectory can never command concurrently) plus the
/// action handler that admits `move_arm_joints` goals. The control task owns the
/// srs_model `Arm` and computes the gravity and Coriolis feedforward in-process
/// every tick (no model node, no per-tick messaging).
pub fn spawn(runner: Arc<NodeRunner>, arm: Arc<Mutex<ArmCan>>, cfg: ControlConfig, model: srs_model::Arm) {
    let busy = Arc::new(AtomicBool::new(false));
    let (goal_tx, goal_rx) = mpsc::channel::<Goal>(1);
    tokio::spawn(run_control(arm, cfg.clone(), goal_rx, busy.clone(), model));
    tokio::spawn(run_action(runner, cfg.limits, goal_tx, busy));
}

/// The single motor-owning control loop. Runs forever at `cfg.cycle_period`: reads
/// state, computes the gravity/Coriolis/friction feedforward in-process from the
/// rigid-body model, and commands either a PD position hold or trajectory tracking.
async fn run_control(
    arm: Arc<Mutex<ArmCan>>,
    cfg: ControlConfig,
    mut goals: mpsc::Receiver<Goal>,
    busy: Arc<AtomicBool>,
    mut model: srs_model::Arm,
) {
    // Hold the power-on pose until the first move (never lunge to zero on boot);
    // after each move the loop holds that move's commanded goal instead.
    let (q0, _) = read_state(&arm, cfg.recv_timeout_us);
    let mut mode = Mode::Hold { setpoint: q0 };
    // Absolute timeline the loop paces against (advances by exactly one
    // `cycle_period` per tick), so per-cycle sleep overshoot doesn't accumulate.
    let mut next_tick = tokio::time::Instant::now();

    info!("control loop started (in-process gravity compensation)");
    loop {
        let cycle_start = Instant::now();
        let (q, qdot) = read_state(&arm, cfg.recv_timeout_us);

        // Feedforward from the rigid-body model: gravity and Coriolis from the
        // posed chain (which carries the distal gripper payload) plus locally
        // computed friction, all at full weight, so the PD term only corrects
        // residual error. Full inverse-dynamics compensation; the teleop follower
        // omits Coriolis, we include it (correct, and zero at rest). `posed`
        // borrows `model`; the terms are copied out before the borrow ends.
        let (gravity, coriolis) = {
            let posed = model.at(&q);
            (posed.gravity_torques(), posed.coriolis_torques(&qdot))
        };
        let friction = friction::torques(&friction::V1, &qdot);
        let ff_tau: JointVec = std::array::from_fn(|i| gravity[i] + coriolis[i] + friction[i]);

        // Admit a new goal while holding (single-flight gated by `busy`).
        if matches!(mode, Mode::Hold { .. }) {
            if let Ok(goal) = goals.try_recv() {
                info!(
                    "move_arm_joints: start={} target={}",
                    fmt_joints(&q),
                    fmt_joints(&goal.target),
                );
                mode = Mode::Trajectory(ActiveMotion {
                    trajectory: Trajectory::new(
                        q,
                        goal.target,
                        cfg.max_joint_velocity_rad_s,
                        cfg.min_motion_time_s,
                    ),
                    ctx: goal.ctx,
                    feedback_period: goal.feedback_period,
                    last_feedback: Instant::now(),
                    feedback_failures: 0,
                });
            }
        }

        let mut finished: Option<MotionResult> = None;

        match &mut mode {
            // Hold the latched setpoint: gravity/Coriolis/friction feedforward + PD.
            Mode::Hold { setpoint } => {
                let mut a = arm.lock().unwrap_or_else(|e| e.into_inner());
                a.mit_control(&cfg.kp, &cfg.kd, setpoint, &ZERO, &ff_tau);
            }
            Mode::Trajectory(m) => {
                let (q_des, dq_des) = m.trajectory.sample(cycle_start);
                {
                    let mut a = arm.lock().unwrap_or_else(|e| e.into_inner());
                    a.mit_control(&cfg.kp, &cfg.kd, &q_des, &dq_des, &ff_tau);
                }
                let elapsed = m.trajectory.motion_start.elapsed().as_secs_f64();
                if m.last_feedback.elapsed() >= m.feedback_period {
                    // Warn once per motion if feedback starts failing, then stay quiet.
                    if let Err(e) = m.ctx.publish_feedback(q, elapsed).await {
                        m.feedback_failures += 1;
                        if m.feedback_failures == 1 {
                            warn!("move_arm_joints feedback publish failing, suppressing repeats: {e}");
                        }
                    }
                    m.last_feedback = Instant::now();
                }
                if m.trajectory.is_complete(cycle_start) {
                    finished = Some(MotionResult::completed(q, elapsed));
                } else if elapsed > cfg.motion_timeout.as_secs_f64() {
                    finished = Some(MotionResult::timed_out(q, elapsed));
                }
            }
        }

        // Completing a goal latches its commanded target as the hold setpoint (hold
        // the goal, not wherever the arm drifted) and frees admission.
        if let Some(result) = finished {
            let hold = match &mode {
                Mode::Trajectory(m) => m.trajectory.target(),
                Mode::Hold { setpoint } => *setpoint,
            };
            if let Mode::Trajectory(m) = std::mem::replace(&mut mode, Mode::Hold { setpoint: hold }) {
                if let Err(e) = m
                    .ctx
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
            }
            busy.store(false, Ordering::Release);
        }

        pace_to_deadline(&mut next_tick, cfg.cycle_period).await;
    }
}

/// Exposes the `move_arm_joints` action: validates the target against this arm's
/// limits, admits one goal at a time (`busy`), and hands the accepted goal to the
/// control task. It never touches the motors.
async fn run_action(
    runner: Arc<NodeRunner>,
    limits: [Limit; ARM_DOF],
    goals: mpsc::Sender<Goal>,
    busy: Arc<AtomicBool>,
) {
    let mut handle = move_arm_joints::ActionHandle::expose(&runner)
        .await
        .expect("expose move_arm_joints");

    loop {
        let ctx = match handle
            .handle_goal_next_request(|req| {
                // Reject targets outside this arm's joint limits (also rejects
                // NaN/inf, which Limit::contains treats as out of range).
                if !target_in_limits(&req.data.joint_positions, &limits) {
                    return Ok(move_arm_joints::GoalResponse::reject(
                        "target joint positions out of range",
                    ));
                }
                // Atomically claim the single-flight slot; the control task clears
                // it when the motion finishes.
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

        let goal = Goal {
            target: ctx.request().data.joint_positions,
            feedback_period: feedback_period(ctx.request().data.feedback_frequency),
            ctx,
        };
        if let Err(err) = goals.send(goal).await {
            // Control task gone (node shutting down): release admission. The goal's
            // `ctx` drops with it, which the engine reports to the client as abandoned.
            error!("control task unavailable, dropping goal: {err}");
            busy.store(false, Ordering::Release);
        }
    }
}

/// Read the measured joint state (positions + velocities) one time.
fn read_state(arm: &Mutex<ArmCan>, recv_timeout_us: i32) -> (JointVec, JointVec) {
    let mut a = arm.lock().unwrap_or_else(|e| e.into_inner());
    a.refresh_all();
    a.recv_all(recv_timeout_us);
    let state = a.get_state();
    (state.positions, state.velocities)
}

/// Pace the loop to an absolute timeline: sleep until `next_tick`, which advances
/// by exactly one `period` each cycle, so the ~1 ms overshoot every
/// `tokio::time::sleep` incurs is corrected on the next cycle instead of
/// accumulating. On an overrun the deadline is already past: re-anchor to now and
/// skip the sleep so the next cycle starts immediately rather than bursting to
/// catch up.
async fn pace_to_deadline(next_tick: &mut tokio::time::Instant, period: Duration) {
    *next_tick += period;
    let now = tokio::time::Instant::now();
    if *next_tick <= now {
        *next_tick = now;
    } else {
        tokio::time::sleep_until(*next_tick).await;
    }
}

/// True if every joint target lies within this arm's position limits. Non-finite
/// values (NaN/inf) fall outside any range, so they are rejected too.
fn target_in_limits(target: &JointVec, limits: &[Limit; ARM_DOF]) -> bool {
    limits.iter().zip(target).all(|(limit, &q)| limit.contains(q))
}

/// Convert a feedback frequency in Hz to a Duration. Floors at 1 Hz to avoid divide-by-zero.
fn feedback_period(freq_hz: u32) -> Duration {
    Duration::from_micros(1_000_000 / freq_hz.max(1) as u64)
}

fn fmt_joints(v: &JointVec) -> String {
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
        // Synthetic window per joint (the real limits come from the URDF at
        // runtime); j4 is one-sided like the elbow, lower bound at 0.
        let mut limits = [Limit { lo: -1.0, hi: 1.0 }; ARM_DOF];
        limits[3] = Limit { lo: 0.0, hi: 2.0 };

        // Home pose (all zeros) is inside every joint limit.
        assert!(target_in_limits(&[0.0; ARM_DOF], &limits));

        // A single joint past its upper bound fails the whole target.
        let mut over = [0.0; ARM_DOF];
        over[3] = limits[3].hi + 0.1;
        assert!(!target_in_limits(&over, &limits));

        // Non-finite values are rejected (Limit::contains is false for NaN/inf).
        let mut nan = [0.0; ARM_DOF];
        nan[0] = f64::NAN;
        assert!(!target_in_limits(&nan, &limits));
        let mut inf = [0.0; ARM_DOF];
        inf[0] = f64::INFINITY;
        assert!(!target_in_limits(&inf, &limits));
    }
}
