use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use peppygen::NodeRunner;
use peppygen::consumed_topics::model_compensation;
use peppygen::emitted_topics::joint_state::v1::joint_state;
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
    /// How long a streamed compensation sample stays usable. Past this age the
    /// model is treated as down: float falls back to a PD position-hold and a
    /// trajectory simply drops its model feedforward (see [`run_control`]).
    pub stale_timeout: Duration,
    /// How often to emit the bring-up diagnostics. Tunable at runtime so logging
    /// can be sped up for a fast step without rebuilding.
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
    /// startup and after every motion. When the compensation stream is stale the
    /// model feedforward is simply dropped, so float goes limp — it never holds a
    /// stale, pose-mismatched gravity term.
    Float,
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

/// One streamed compensation sample plus its arrival time (for staleness).
#[derive(Clone, Copy)]
struct CompSample {
    gravity: JointVec,
    coriolis: JointVec,
    seq: u32,
    arrival: Instant,
}

/// Latest compensation sample: written by the subscriber task, read by the loop.
type CompCache = Arc<Mutex<Option<CompSample>>>;

/// Spawn the arm's control: a single task that owns the motors, the action
/// handler that admits `move_arm_joints` goals, and a background subscriber that
/// caches the latest streamed `compensation`. The control task is the *only*
/// motor writer, so float and trajectory can never command concurrently.
pub fn spawn(runner: Arc<NodeRunner>, arm: Arc<Mutex<ArmCan>>, cfg: ControlConfig) {
    let busy = Arc::new(AtomicBool::new(false));
    let (goal_tx, goal_rx) = mpsc::channel::<Goal>(1);
    let comp: CompCache = Arc::new(Mutex::new(None));
    spawn_comp_subscriber(runner.clone(), comp.clone());
    tokio::spawn(run_control(runner.clone(), arm, cfg.clone(), goal_rx, busy.clone(), comp));
    tokio::spawn(run_action(runner, cfg.limits, goal_tx, busy));
}

/// Background subscriber: caches the latest `compensation` sample (with arrival
/// time) for the control loop to read without blocking. The generated
/// `on_next_message_received` re-subscribes per call, which is fine here: we only
/// ever want the most recent sample, and the control loop never waits on it.
fn spawn_comp_subscriber(runner: Arc<NodeRunner>, comp: CompCache) {
    tokio::spawn(async move {
        let mut warned = false;
        loop {
            match model_compensation::on_next_message_received(&runner, None).await {
                Ok((_instance_id, msg)) => match (to_joints(&msg.gravity), to_joints(&msg.coriolis)) {
                    (Some(gravity), Some(coriolis)) => {
                        *comp.lock().unwrap_or_else(|e| e.into_inner()) = Some(CompSample {
                            gravity,
                            coriolis,
                            seq: msg.seq,
                            arrival: Instant::now(),
                        });
                        warned = false;
                    }
                    _ => {
                        if !warned {
                            warn!(
                                "compensation: expected {ARM_DOF} joints, got {}/{} (suppressing repeats)",
                                msg.gravity.len(),
                                msg.coriolis.len(),
                            );
                            warned = true;
                        }
                    }
                },
                Err(e) => {
                    if !warned {
                        warn!("compensation stream error, suppressing repeats: {e}");
                        warned = true;
                    }
                }
            }
        }
    });
}

/// Convert a wire joint vector (unspecified length on the streaming interface)
/// into the fixed `[f64; ARM_DOF]`, rejecting a wrong-DOF message.
fn to_joints(v: &[f64]) -> Option<JointVec> {
    JointVec::try_from(v).ok()
}


/// The single motor-owning control loop. Runs forever at `cfg.cycle_period`: reads
/// state, publishes it for the model node, reads the latest streamed compensation,
/// and commands either a compliant float, a PD position-hold (stale fallback), or
/// trajectory tracking.
async fn run_control(
    runner: Arc<NodeRunner>,
    arm: Arc<Mutex<ArmCan>>,
    cfg: ControlConfig,
    mut goals: mpsc::Receiver<Goal>,
    busy: Arc<AtomicBool>,
    comp: CompCache,
) {
    let mut mode = Mode::Float;
    let mut prev_fresh = false;
    let mut seq: u32 = 0;
    let mut publish_warned = false;
    let mut loop_stats = LoopStats::new();
    let mut last_diag = Instant::now();
    let mut prev_q = ZERO;
    // Absolute timeline the loop paces against (advances by exactly one
    // `cycle_period` per tick), so per-cycle sleep overshoot doesn't accumulate.
    let mut next_tick = tokio::time::Instant::now();

    info!("control loop started (streamed gravity compensation)");
    loop {
        let cycle_start = Instant::now();
        let (q, qdot) = read_state(&arm, cfg.recv_timeout_us);

        // Push measured state to the model node (fire-and-forget: a one-way
        // publish, no per-tick round trip on the control path).
        seq = seq.wrapping_add(1);
        match joint_state::emit(&runner, seq, q.to_vec(), qdot.to_vec()).await {
            Ok(()) => publish_warned = false,
            Err(e) => {
                if !publish_warned {
                    warn!("joint_state publish failing, suppressing repeats: {e}");
                    publish_warned = true;
                }
            }
        }

        // Latest cached compensation (Copy out under the lock), and whether it's
        // fresh enough to use.
        let sample = *comp.lock().unwrap_or_else(|e| e.into_inner());
        let fresh = sample.is_some_and(|s| s.arrival.elapsed() < cfg.stale_timeout);
        // One-shot log when freshness flips, so a model outage (or recovery) is
        // obvious without scanning the periodic diagnostic.
        if fresh != prev_fresh {
            if fresh {
                info!("compensation stream live (model feedforward active)");
            } else {
                warn!("compensation stale; dropping model feedforward (float goes compliant; a move keeps tracking on kp/kd)");
            }
            prev_fresh = fresh;
        }
        let friction = friction::torques(&friction::V1, &qdot);

        // Admit a new goal while floating (single-flight gated by `busy`).
        if matches!(mode, Mode::Float) {
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

        // The model feedforward (gravity + Coriolis), scaled. Zeroed when stale so
        // a stale, pose-mismatched gravity term is never applied. Friction is
        // local and always fresh.
        let model_tau: JointVec = match sample {
            Some(s) if fresh => std::array::from_fn(|i| {
                cfg.gravity_scale * s.gravity[i] + cfg.coriolis_scale * s.coriolis[i]
            }),
            _ => ZERO,
        };
        let ff_tau: JointVec =
            std::array::from_fn(|i| model_tau[i] + cfg.friction_scale * friction[i]);

        let diag_due = last_diag.elapsed() >= cfg.log_period;
        let mut finished: Option<MotionResult> = None;
        let mut tracking: Option<(JointVec, f64)> = None;

        match &mut mode {
            // Compliant float: pure feedforward, kp=kd=0. When the stream is stale
            // `ff_tau` carries only friction (~0 at rest), so float goes limp.
            Mode::Float => {
                let mut a = arm.lock().unwrap_or_else(|e| e.into_inner());
                a.mit_control(&ZERO, &ZERO, &q, &ZERO, &ff_tau);
            }
            Mode::Trajectory(m) => {
                let (q_des, dq_des) = m.trajectory.sample(cycle_start);
                {
                    let mut a = arm.lock().unwrap_or_else(|e| e.into_inner());
                    a.mit_control(&cfg.kp, &cfg.kd, &q_des, &dq_des, &ff_tau);
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

        // Completing a goal returns to float and frees admission. Float then
        // applies fresh compensation (or goes limp if the stream is stale).
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

        // Bring-up diagnostics on a fixed cadence.
        loop_stats.record(cycle_start.elapsed(), cfg.cycle_period);
        if diag_due {
            let window = last_diag.elapsed();
            loop_stats.report(window, cfg.cycle_period);
            log_compensation(&cfg, state_label(&mode), sample, fresh, &q, &qdot, &friction, &ff_tau, &prev_q);
            if let Some((q_des, elapsed)) = tracking {
                log_tracking(&q_des, &q, elapsed);
            }
            loop_stats = LoopStats::new();
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

/// Control-loop timing over a diagnostic window: how much of each cycle the work
/// (state read + publish + command) consumes and how often it overran the budget.
/// TEMPORARY bring-up instrumentation.
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

/// Short label for the diagnostic line: what the loop is doing this window.
fn state_label(mode: &Mode) -> &'static str {
    match mode {
        Mode::Trajectory(_) => "trajectory",
        Mode::Float => "float",
    }
}

/// Bring-up diagnostic: the loop state, compensation freshness/age, the model
/// terms and friction, and how far the arm drifted since the last window, so
/// soundness (signs, magnitudes, staleness, stability) can be judged from the
/// logs. `tau` is the feedforward actually applied (float: the whole command;
/// trajectory: added on top of the PD tracking term); `gravity`/`coriolis` are
/// zeroed in `tau` when stale. TEMPORARY; remove once compensation is trusted.
#[allow(clippy::too_many_arguments)]
fn log_compensation(
    cfg: &ControlConfig,
    state: &str,
    sample: Option<CompSample>,
    fresh: bool,
    q: &JointVec,
    qdot: &JointVec,
    friction: &JointVec,
    ff_tau: &JointVec,
    prev_q: &JointVec,
) {
    // Raw last-cached model terms (shown even when stale, so you can see the last
    // value and its age); `tau` reflects whether they were actually applied.
    let (gravity, coriolis, seq, age_ms) = match sample {
        Some(s) => (s.gravity, s.coriolis, s.seq, s.arrival.elapsed().as_millis()),
        None => (ZERO, ZERO, 0, 0),
    };
    let drift = q.iter().zip(prev_q).map(|(a, b)| (a - b).abs()).fold(0.0, f64::max);
    info!(
        "comp state={state} fresh={fresh} age={age_ms}ms seq={seq} scales(g={} c={} f={}) max_drift={:.4}rad\n  q={}\n  qdot={}\n  gravity={}\n  coriolis={}\n  friction={}\n  tau={}",
        cfg.gravity_scale,
        cfg.coriolis_scale,
        cfg.friction_scale,
        drift,
        fmt_joints(q),
        fmt_joints(qdot),
        fmt_joints(&gravity),
        fmt_joints(&coriolis),
        fmt_joints(friction),
        fmt_joints(ff_tau),
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
/// catch up. Overruns are not warned per-tick; `LoopStats` reports the count.
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

    #[test]
    fn to_joints_accepts_seven_and_rejects_other_lengths() {
        assert!(to_joints(&[0.0; ARM_DOF]).is_some());
        assert!(to_joints(&[0.0; 6]).is_none());
        assert!(to_joints(&[0.0; 8]).is_none());
    }
}
