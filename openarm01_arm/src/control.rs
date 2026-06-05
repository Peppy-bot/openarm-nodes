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
    /// Scale applied to the gravity feedforward term. 1.0 is full compensation;
    /// lower it for safe bring-up (e.g. 0.3) to confirm each joint pushes against
    /// gravity in the right direction before trusting the full term.
    pub gravity_scale: f64,
    /// Scale applied to the Coriolis feedforward term. The openarm teleop example
    /// uses a small factor (~0.1).
    pub coriolis_scale: f64,
    /// Scale applied to the friction feedforward term. The openarm teleop example
    /// uses ~0.3 (its transparency mode); 1.0 is the full physical friction.
    pub friction_scale: f64,
    /// Per-request deadline for the `get_compensation` service call.
    pub compensation_timeout: Duration,
    /// How often to emit the bring-up diagnostics (loop timing, poll latency,
    /// compensation terms, tracking error). Tunable at runtime so logging can be
    /// sped up for a fast step without rebuilding.
    pub log_period: Duration,
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
    let mut last_gravity = ZERO;
    let mut last_coriolis = ZERO;
    let mut comp_failures: u32 = 0;
    let mut poll_stats = PollStats::new();
    let mut loop_stats = LoopStats::new();
    let mut last_diag = Instant::now();
    let mut prev_q = ZERO;
    // Don't energize the arm until compensation has been obtained at least once:
    // commanding pure feedforward with a missing gravity term would let it sag.
    // `depends_on` starts srs_model first, so this clears on the first tick.
    let mut have_compensation = false;
    // Absolute timeline the loop paces against (advances by exactly one
    // `cycle_period` per tick). Anchoring here rather than to "now + remaining"
    // keeps per-cycle sleep overshoot from accumulating into a slow loop.
    let mut next_tick = tokio::time::Instant::now();

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

        let ff = feedforward(
            &runner,
            &cfg,
            &q,
            &qdot,
            &mut last_gravity,
            &mut last_coriolis,
            &mut poll_stats,
        )
        .await;
        note_compensation(ff.ok, &mut comp_failures);
        let had_compensation = have_compensation;
        have_compensation |= ff.ok;
        if have_compensation && !had_compensation {
            info!("compensation acquired; arm will energize");
        }
        let diag_due = last_diag.elapsed() >= cfg.log_period;

        let mut finished: Option<MotionResult> = None;
        let mut tracking: Option<(JointVec, f64)> = None;
        match &mut mode {
            Mode::Float => {
                if have_compensation {
                    let mut a = arm.lock().unwrap_or_else(|e| e.into_inner());
                    a.mit_control(&ZERO, &ZERO, &q, &ZERO, &ff.tau);
                }
            }
            Mode::Trajectory(m) => {
                let (q_des, dq_des) = m.trajectory.sample(cycle_start);
                {
                    let mut a = arm.lock().unwrap_or_else(|e| e.into_inner());
                    a.mit_control(&cfg.kp, &cfg.kd, &q_des, &dq_des, &ff.tau);
                }
                let elapsed = m.trajectory.motion_start.elapsed().as_secs_f64();
                tracking = Some((q_des, elapsed));
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

        // Bring-up diagnostics on a fixed cadence: loop timing, poll latency,
        // compensation breakdown, and (while moving) tracking error. Work time is
        // measured before the report so the one-per-window log doesn't skew it.
        loop_stats.record(cycle_start.elapsed(), cfg.cycle_period);
        if diag_due {
            let window = last_diag.elapsed();
            loop_stats.report(window, cfg.cycle_period);
            poll_stats.report();
            log_compensation(&cfg, &q, &qdot, &ff, &prev_q);
            if let Some((q_des, elapsed)) = tracking {
                log_tracking(&q_des, &q, elapsed);
            }
            loop_stats = LoopStats::new();
            poll_stats = PollStats::new();
            prev_q = q;
            last_diag = Instant::now();
        }

        pace_to_deadline(&mut next_tick, cfg.cycle_period).await;
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

/// Per-tick `get_compensation` poll latency accumulated over a diagnostic window.
/// The compensation fetch sits on the hot control path, so the `max` is the number
/// that says whether the synchronous poll fits the cycle budget at a given rate.
/// TEMPORARY bring-up instrumentation; remove once timing is trusted.
struct PollStats {
    count: u32,
    sum: Duration,
    min: Duration,
    max: Duration,
}

impl PollStats {
    fn new() -> Self {
        Self { count: 0, sum: Duration::ZERO, min: Duration::MAX, max: Duration::ZERO }
    }

    fn record(&mut self, elapsed: Duration) {
        self.count += 1;
        self.sum += elapsed;
        self.min = self.min.min(elapsed);
        self.max = self.max.max(elapsed);
    }

    fn report(&self) {
        if self.count == 0 {
            info!("poll latency: no samples this window (compensation never polled)");
            return;
        }
        info!(
            "poll latency: avg={}µs min={}µs max={}µs (n={})",
            (self.sum / self.count).as_micros(),
            self.min.as_micros(),
            self.max.as_micros(),
            self.count,
        );
    }
}

/// Control-loop timing over a diagnostic window: how much of each cycle the work
/// (state read + compensation + command) consumes and how often it overran the
/// budget. The headline number for deciding whether the loop can sustain a higher
/// control rate. TEMPORARY bring-up instrumentation.
struct LoopStats {
    count: u32,
    work_sum: Duration,
    work_max: Duration,
    overruns: u32,
}

impl LoopStats {
    fn new() -> Self {
        Self { count: 0, work_sum: Duration::ZERO, work_max: Duration::ZERO, overruns: 0 }
    }

    fn record(&mut self, work: Duration, budget: Duration) {
        self.count += 1;
        self.work_sum += work;
        self.work_max = self.work_max.max(work);
        if work > budget {
            self.overruns += 1;
        }
    }

    fn report(&self, window: Duration, budget: Duration) {
        if self.count == 0 {
            return;
        }
        let secs = window.as_secs_f64().max(f64::MIN_POSITIVE);
        info!(
            "loop: {:.0} Hz (n={}), work avg={:.2}ms max={:.2}ms, overruns={} (budget {:.2}ms)",
            self.count as f64 / secs,
            self.count,
            (self.work_sum.as_secs_f64() / self.count as f64) * 1e3,
            self.work_max.as_secs_f64() * 1e3,
            self.overruns,
            budget.as_secs_f64() * 1e3,
        );
    }
}

/// One tick's feedforward: the commanded `tau` plus the separate terms it was
/// built from. `gravity`/`coriolis` are the last good model values (stale-reused
/// on a poll miss); `friction` is always fresh. The breakdown is what the bring-up
/// diagnostic logs so signs and magnitudes can be judged term by term.
struct Feedforward {
    tau: JointVec,
    /// Whether the compensation service answered usefully this tick.
    ok: bool,
    gravity: JointVec,
    coriolis: JointVec,
    friction: JointVec,
}

/// Feedforward torque for one control tick: gravity + Coriolis from the
/// `gravity_coriolis_compensation` service (one round trip), plus locally computed
/// friction, each weighted by its scale. On a service failure (or a wrong-DOF
/// response) the last good gravity/Coriolis are reused (they change slowly), so the
/// loop neither stalls nor drops compensation; friction is always applied.
async fn feedforward(
    runner: &NodeRunner,
    cfg: &ControlConfig,
    q: &JointVec,
    qdot: &JointVec,
    last_gravity: &mut JointVec,
    last_coriolis: &mut JointVec,
    poll_stats: &mut PollStats,
) -> Feedforward {
    // The interface is DOF-generic (Vec on the wire); convert at the boundary.
    let request = compensation::Request {
        joint_positions: q.to_vec(),
        joint_velocities: qdot.to_vec(),
    };
    let poll_start = Instant::now();
    let response = compensation::poll(runner, cfg.compensation_timeout, request).await;
    poll_stats.record(poll_start.elapsed());
    let ok = match response {
        Ok(response) => match (
            JointVec::try_from(response.data.gravity.as_slice()),
            JointVec::try_from(response.data.coriolis.as_slice()),
        ) {
            (Ok(gravity), Ok(coriolis)) => {
                *last_gravity = gravity;
                *last_coriolis = coriolis;
                true
            }
            _ => false, // wrong joint count for this arm; keep last good
        },
        Err(_) => false,
    };
    // Each term independently scaled, so bring-up can ramp gravity, then friction,
    // then Coriolis (matching the openarm teleop weighting at full).
    let friction = friction::torques(&friction::V1, qdot);
    let tau = std::array::from_fn(|i| {
        cfg.gravity_scale * last_gravity[i]
            + cfg.coriolis_scale * last_coriolis[i]
            + cfg.friction_scale * friction[i]
    });
    Feedforward { tau, ok, gravity: *last_gravity, coriolis: *last_coriolis, friction }
}

/// Bring-up diagnostic: the measured state, each feedforward term, and how far the
/// arm drifted since the last window (≈0 = holding, large = sagging or running), so
/// soundness (signs, magnitudes, stability) can be judged from the logs. TEMPORARY;
/// remove with [`PollStats`]/[`LoopStats`] once compensation is trusted.
fn log_compensation(
    cfg: &ControlConfig,
    q: &JointVec,
    qdot: &JointVec,
    ff: &Feedforward,
    prev_q: &JointVec,
) {
    let drift = q.iter().zip(prev_q).map(|(a, b)| (a - b).abs()).fold(0.0, f64::max);
    info!(
        "comp ok={} scales(g={} c={} f={}) max_drift={:.4}rad\n  q={}\n  qdot={}\n  gravity={}\n  coriolis={}\n  friction={}\n  tau={}",
        ff.ok,
        cfg.gravity_scale,
        cfg.coriolis_scale,
        cfg.friction_scale,
        drift,
        fmt_joints(q),
        fmt_joints(qdot),
        fmt_joints(&ff.gravity),
        fmt_joints(&ff.coriolis),
        fmt_joints(&ff.friction),
        fmt_joints(&ff.tau),
    );
}

/// Bring-up diagnostic for trajectory tracking: commanded vs measured joints and
/// the worst-joint error, so tracking quality can be judged during a move.
fn log_tracking(q_des: &JointVec, q: &JointVec, elapsed: f64) {
    let err: JointVec = std::array::from_fn(|i| q_des[i] - q[i]);
    let max_err = err.iter().map(|e| e.abs()).fold(0.0, f64::max);
    info!(
        "track t={:.2}s max_err={:.4}rad\n  q_des={}\n  q={}\n  err={}",
        elapsed,
        max_err,
        fmt_joints(q_des),
        fmt_joints(q),
        fmt_joints(&err),
    );
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

/// Pace the loop to an absolute timeline: sleep until `next_tick`, which advances
/// by exactly one `period` each cycle. Because the deadline is absolute (not
/// "now + remaining"), the ~1 ms overshoot every `tokio::time::sleep` incurs is
/// corrected on the following cycle instead of accumulating, so the loop holds
/// its target rate. On an overrun the deadline is already in the past: re-anchor
/// to now and skip the sleep so the next cycle starts immediately, rather than
/// firing a burst of zero-length cycles to "catch up" the missed time. Overruns
/// are not warned per-tick (that spams at high rates); `LoopStats` reports the
/// overrun count per window.
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
