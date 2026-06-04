use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use peppygen::NodeRunner;
use peppygen::consumed_services::model_get_compensation as compensation;
use peppygen::exposed_actions::openarm01_arm::v1::move_arm_joints;
use tokio::sync::mpsc;
use tracing::{error, info, warn};

use crate::friction;
use crate::joint_limits::{self, Limit};
use crate::trajectory::Trajectory;
use crate::{ARM_DOF, JointVec};
use openarm_can::ArmCan;

/// kp/kd/dq all zero: pure-torque ("float") command, the arm follows only the
/// feedforward torque and is otherwise compliant.
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
    /// Scale applied to the Coriolis feedforward term (gravity is always full).
    /// The openarm teleop example uses a small factor (~0.1).
    pub coriolis_scale: f64,
    /// Scale applied to the friction feedforward term. The openarm teleop example
    /// uses ~0.3 (its transparency mode); 1.0 is the full physical friction.
    pub friction_scale: f64,
    /// Per-request deadline for the `get_compensation` service call.
    pub compensation_timeout: Duration,
    /// This arm's side-specific joint position limits (selected by `arm_id`).
    pub limits: &'static [Limit; joint_limits::ARM_DOF],
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
enum Mode {
    /// Compliant gravity/Coriolis/friction compensation (kp=kd=0): the default at
    /// startup and after every motion, so the arm is never left uncompensated.
    Float,
    /// Tracking a trajectory for an accepted `move_arm_joints` goal.
    Trajectory(ActiveMotion),
    // TODO: JointAngleStream - track a live stream of joint-angle setpoints (no
    // trajectory planning), e.g. for teleop from a leader arm or pose estimator.
    // It slots in here as another command source for the single writer; the
    // action handler / a topic subscriber would push setpoints and switch the mode.
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

/// Spawn the arm's control: a single task that owns the motors (gravity-comp
/// float by default, trajectory tracking while a goal runs, back to float after),
/// plus the action handler that admits `move_arm_joints` goals and feeds them to
/// it over a channel. The control task is the *only* motor writer, so float and
/// trajectory can never command concurrently.
pub fn spawn(runner: Arc<NodeRunner>, arm: Arc<Mutex<ArmCan>>, cfg: ControlConfig) {
    // Single-flight admission: at most one goal active at a time. This only gates
    // the action handler; the control task is the sole writer, so there is no
    // writer race, just admission. Capacity 1 is enough (a goal is admitted only
    // once the previous one has finished and cleared the flag).
    let busy = Arc::new(AtomicBool::new(false));
    let (goal_tx, goal_rx) = mpsc::channel::<Goal>(1);
    tokio::spawn(run_control(runner.clone(), arm, cfg.clone(), goal_rx, busy.clone()));
    tokio::spawn(run_action(runner, cfg.limits, goal_tx, busy));
}

/// The single motor-owning control loop. Runs forever at `cfg.cycle_period`,
/// reading state and computing one feedforward torque per tick, then commanding
/// either a compliant float hold or trajectory tracking depending on the mode.
async fn run_control(
    runner: Arc<NodeRunner>,
    arm: Arc<Mutex<ArmCan>>,
    cfg: ControlConfig,
    mut goals: mpsc::Receiver<Goal>,
    busy: Arc<AtomicBool>,
) {
    let mut mode = Mode::Float;
    let mut last_model = ZERO;
    let mut comp_failures: u32 = 0;
    let mut poll_stats = PollStats::new();
    // Don't energize the arm until compensation has been obtained at least once:
    // commanding pure feedforward with a missing gravity term would let it sag.
    // `depends_on` starts srs_model first, so this clears on the first tick.
    let mut have_compensation = false;

    info!("control loop started (float gravity compensation)");
    loop {
        let cycle_start = Instant::now();
        let (q, qdot) = read_state(&arm, cfg.recv_timeout_us);

        // Admit a new goal only while floating, and only once compensation has
        // been obtained at least once, so a trajectory never runs with a missing
        // gravity term (same gate as energizing float below). The action handler
        // already gated on `busy`, so a goal is waiting here only when the arm is
        // idle; until then it stays buffered in the channel. Anchor the trajectory
        // at the freshly measured position.
        if matches!(mode, Mode::Float) && have_compensation {
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

        let (tau, ok) =
            feedforward(&runner, &cfg, &q, &qdot, &mut last_model, &mut poll_stats).await;
        note_compensation(ok, &mut comp_failures);
        poll_stats.report_if_due();
        have_compensation |= ok;

        let mut finished: Option<MotionResult> = None;
        match &mut mode {
            Mode::Float => {
                if have_compensation {
                    let mut a = arm.lock().unwrap_or_else(|e| e.into_inner());
                    a.mit_control(&ZERO, &ZERO, &q, &ZERO, &tau);
                }
            }
            Mode::Trajectory(m) => {
                let (q_des, dq_des) = m.trajectory.sample(cycle_start);
                {
                    let mut a = arm.lock().unwrap_or_else(|e| e.into_inner());
                    a.mit_control(&cfg.kp, &cfg.kd, &q_des, &dq_des, &tau);
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

        // Completing a goal returns to float and frees admission. Done after the
        // match so the trajectory's `ctx` can be moved out of `mode`.
        if let Some(result) = finished {
            if let Mode::Trajectory(m) = std::mem::replace(&mut mode, Mode::Float) {
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

        pace_cycle(cycle_start, cfg.cycle_period).await;
    }
}

/// Exposes the `move_arm_joints` action: validates the target against this arm's
/// limits, admits one goal at a time (`busy`), and hands the accepted goal to the
/// control task. It never touches the motors.
async fn run_action(
    runner: Arc<NodeRunner>,
    limits: &'static [Limit; joint_limits::ARM_DOF],
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
                if !target_in_limits(&req.data.joint_positions, limits) {
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

/// Rolling latency of the per-tick `get_compensation` poll, reported once a
/// second. TEMPORARY: the compensation fetch sits on the hot control path, so
/// this confirms it stays in the microsecond range in practice; remove once the
/// timing is trusted (or the fetch is moved off the tick).
struct PollStats {
    window_start: Instant,
    count: u32,
    sum: Duration,
    min: Duration,
    max: Duration,
}

impl PollStats {
    fn new() -> Self {
        Self {
            window_start: Instant::now(),
            count: 0,
            sum: Duration::ZERO,
            min: Duration::MAX,
            max: Duration::ZERO,
        }
    }

    fn record(&mut self, elapsed: Duration) {
        self.count += 1;
        self.sum += elapsed;
        self.min = self.min.min(elapsed);
        self.max = self.max.max(elapsed);
    }

    /// Log avg/min/max once the 1 s window elapses, then reset for the next one.
    fn report_if_due(&mut self) {
        if self.window_start.elapsed() < Duration::from_secs(1) {
            return;
        }
        if self.count > 0 {
            info!(
                "compensation poll latency over {} ticks: avg={}µs min={}µs max={}µs",
                self.count,
                (self.sum / self.count).as_micros(),
                self.min.as_micros(),
                self.max.as_micros(),
            );
        }
        *self = Self::new();
    }
}

/// Feedforward torque for one control tick: gravity + Coriolis from the
/// `gravity_coriolis_compensation` service (one round trip), plus locally computed
/// friction. On a service failure (or a wrong-DOF response) the last good model
/// torque is reused (gravity/Coriolis change slowly), so the loop neither stalls
/// nor drops compensation; friction is always applied. Returns the torque and
/// whether the service answered usefully this tick.
async fn feedforward(
    runner: &NodeRunner,
    cfg: &ControlConfig,
    q: &JointVec,
    qdot: &JointVec,
    last_model: &mut JointVec,
    poll_stats: &mut PollStats,
) -> (JointVec, bool) {
    // The interface is DOF-generic (Vec on the wire); convert at the boundary.
    let request = compensation::Request {
        joint_positions: q.to_vec(),
        joint_velocities: qdot.to_vec(),
    };
    let poll_start = Instant::now();
    let response = compensation::poll(runner, cfg.compensation_timeout, request).await;
    poll_stats.record(poll_start.elapsed());
    let ok = match response {
        // Gravity is applied in full; Coriolis is scaled (so the combined `total`
        // field is unused). Lets the consumer weight each term, matching the
        // openarm teleop example.
        Ok(response) => match (
            JointVec::try_from(response.data.gravity.as_slice()),
            JointVec::try_from(response.data.coriolis.as_slice()),
        ) {
            (Ok(gravity), Ok(coriolis)) => {
                *last_model =
                    std::array::from_fn(|i| gravity[i] + cfg.coriolis_scale * coriolis[i]);
                true
            }
            _ => false, // wrong joint count for this arm; keep last good
        },
        Err(_) => false,
    };
    let fric = friction::torques(&friction::V1, qdot);
    let tau = std::array::from_fn(|i| last_model[i] + cfg.friction_scale * fric[i]);
    (tau, ok)
}

/// Warn once when compensation starts failing, then stay quiet until it recovers,
/// so a persistent outage doesn't spam the log every control tick.
fn note_compensation(ok: bool, consecutive_failures: &mut u32) {
    if ok {
        *consecutive_failures = 0;
        return;
    }
    *consecutive_failures += 1;
    if *consecutive_failures == 1 {
        warn!("get_compensation unavailable; holding last torque (suppressing repeats)");
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

/// Sleep out the remainder of a control cycle, warning on a significant overrun.
async fn pace_cycle(cycle_start: Instant, period: Duration) {
    let elapsed = cycle_start.elapsed();
    if elapsed < period {
        tokio::time::sleep(period - elapsed).await;
    } else if elapsed > period.mul_f64(1.2) {
        warn!(
            "control loop overrun: {:.1}ms (budget {:.1}ms)",
            elapsed.as_secs_f64() * 1000.0,
            period.as_secs_f64() * 1000.0,
        );
    }
}

/// True if every joint target lies within this arm's position limits. Non-finite
/// values (NaN/inf) fall outside any range, so they are rejected too.
fn target_in_limits(target: &JointVec, limits: &[Limit; joint_limits::ARM_DOF]) -> bool {
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
        let limits = joint_limits::for_arm_id(joint_limits::ARM_ID_LEFT);

        // Home pose (all zeros) is inside every joint limit.
        assert!(target_in_limits(&[0.0; ARM_DOF], limits));

        // A single joint past its upper bound fails the whole target.
        let mut over = [0.0; ARM_DOF];
        over[3] = limits[3].upper + 0.1;
        assert!(!target_in_limits(&over, limits));

        // Non-finite values are rejected.
        let mut nan = [0.0; ARM_DOF];
        nan[0] = f64::NAN;
        assert!(!target_in_limits(&nan, limits));
        let mut inf = [0.0; ARM_DOF];
        inf[0] = f64::INFINITY;
        assert!(!target_in_limits(&inf, limits));
    }
}
