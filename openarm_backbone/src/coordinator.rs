//! The bimanual coordination loop. Every tick it advances both arms' planners
//! and both grippers' openings to candidate setpoints, governs the whole step
//! against the self-collision model in one call (arms and openings are one
//! governed configuration), and publishes the governed per-arm setpoints and
//! per-gripper openings. One loop owns the governor (the single collision
//! model), both planners, and the backbone-executed gripper moves, so everything is
//! always governed together against a consistent configuration, and the
//! governed result is fed back so the next tick chases from where each DOF was
//! actually allowed to go.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use peppygen::NodeRunner;
use peppygen::emitted_topics::collision_status;
use peppygen::exposed_actions::move_gripper;
use peppygen::paired_topics::{left_arm_link, left_gripper_link, right_arm_link, right_gripper_link};
use peppylib::runtime::CancellationToken;
use tokio::sync::{mpsc, watch};
use tracing::{error, info, warn};

use control_core::{Pacer, filters::LowPassFilter};

use crate::governor::{GovState, Governor, Guard};
use crate::planner::{BusyGuard, Goal, Planner};
use crate::streams::{GovernorConfig, GripperCommand, GripperOpening, JointCommand, MeasuredState};
use crate::{ARM_DOF, ArmPair, JointVec, MOTION_TIMEOUT_FACTOR, Side, motion_timed_out};

/// How long [`seed`] waits for an arm's first measured state before warning that
/// the backbone is still blocked, so a silent arm is visible in the log instead of an
/// indefinite quiet stall.
const SEED_WAIT_WARN_PERIOD: Duration = Duration::from_secs(2);

/// One arm's inbound channels into the coordinator: the commander's arm command
/// stream, the commander's gripper opening command stream, the measured arm state,
/// the measured gripper opening, the accepted-goal queues, and the single-flight
/// busy flags (one for arm moves, one for gripper moves).
///
/// The two command streams are held as their `watch::Sender`, not a receiver: the
/// coordinator both reads the latest (`borrow`) and clears it (`send_replace`)
/// while a move runs on that side, so a setpoint still in flight when the move was
/// fired cannot re-target the arm (or snap the jaws) when the move ends. The
/// stream listener holds a clone of the same sender and fills it.
pub struct ArmChannels {
    pub command: watch::Sender<Option<JointCommand>>,
    pub gripper_command: watch::Sender<Option<GripperCommand>>,
    pub measured: watch::Receiver<Option<MeasuredState>>,
    pub gripper: watch::Receiver<Option<GripperOpening>>,
    pub goals: mpsc::Receiver<Goal>,
    pub busy: Arc<AtomicBool>,
    pub gripper_goals: mpsc::Receiver<GripperGoal>,
    pub gripper_busy: Arc<AtomicBool>,
}

/// The coordinator's run parameters. A commander that stops streaming simply
/// leaves its last governed setpoint in place (the follower holds it), so
/// there is no freshness deadman to configure.
pub struct RunConfig {
    pub cycle_period: Duration,
    /// Cutoff (Hz) for the low-pass on each published desired velocity. `dq` is a
    /// per-tick position difference scaled by `1/dt`, so it amplifies any setpoint noise
    /// by the control rate; filtering it keeps the arm's Kd term from buzzing on a noisy
    /// stream without touching the desired position.
    pub velocity_filter_cutoff_hz: f64,
}

/// An accepted `move_gripper` goal handed to the coordinator, which executes it
/// through the same per-tick governing as everything else (the gripper analog of
/// [`Goal`] for the arms). The opening is the validated goal fraction.
pub struct GripperGoal {
    pub opening: f64,
    pub ctx: move_gripper::GoalContext,
}

/// A backbone-executed gripper move in flight: the opening chases `target_frac`
/// through the governor until the governed chase lands on the target, the goal
/// is cancelled, or the move overruns its budget (a governed clamp short of the
/// target ends here). Like the arm's trajectory tiers, completion is graded on
/// the commanded motion, not the measured jaws; the result reports the measured
/// opening and the caller judges it. The busy guard releases the side's
/// single-flight slot on any exit.
struct GripperMove {
    target_frac: f64,
    ctx: move_gripper::GoalContext,
    started: Instant,
    /// Nominal chase duration; the runtime aborts once the move runs past
    /// `MOTION_TIMEOUT_FACTOR` times this, exactly as the arm servo does.
    budget_s: f64,
    _busy: BusyGuard,
}

/// Run the coordination loop. Holds the governor and both planners. Runs until
/// the node's cancellation token fires; returns `Err` if a publisher cannot be
/// declared at bringup. Any return takes the node down (the supervisor in `main`
/// treats it as fatal).
pub async fn run(
    runner: Arc<NodeRunner>,
    mut governor: Governor,
    mut planners: ArmPair<Planner>,
    mut channels: ArmPair<ArmChannels>,
    governor_config: watch::Receiver<GovernorConfig>,
    config: RunConfig,
    token: CancellationToken,
) -> peppygen::Result<()> {
    let RunConfig {
        cycle_period,
        velocity_filter_cutoff_hz,
    } = config;
    // One publisher per pairing slot (arms and grippers alike). Publishing while
    // a slot is unpaired is a legal no-op, so the backbone streams governed setpoints
    // regardless and a follower simply starts tracking once its pair is
    // established.
    let left_arm_pub = match left_arm_link::arm_setpoints::declare_publisher(&runner).await {
        Ok(p) => p,
        Err(e) => {
            error!("declare left arm_setpoints publisher: {e}");
            return Err(e);
        }
    };
    let right_arm_pub = match right_arm_link::arm_setpoints::declare_publisher(&runner).await {
        Ok(p) => p,
        Err(e) => {
            error!("declare right arm_setpoints publisher: {e}");
            return Err(e);
        }
    };
    let left_gripper_pub =
        match left_gripper_link::gripper_commands::declare_publisher(&runner).await {
            Ok(p) => p,
            Err(e) => {
                error!("declare left gripper_commands publisher: {e}");
                return Err(e);
            }
        };
    let right_gripper_pub =
        match right_gripper_link::gripper_commands::declare_publisher(&runner).await {
            Ok(p) => p,
            Err(e) => {
                error!("declare right gripper_commands publisher: {e}");
                return Err(e);
            }
        };
    let status_publisher = match collision_status::declare_publisher(&runner).await {
        Ok(p) => p,
        Err(e) => {
            error!("declare collision_status publisher: {e}");
            return Err(e);
        }
    };

    // Hold each arm's real pose, not a neutral zero: wait for the first measured
    // state from both arms and seed the held setpoint there before publishing.
    if seed(&mut channels.left, &mut planners.left, Side::Left)
        .await
        .is_err()
        || seed(&mut channels.right, &mut planners.right, Side::Right)
            .await
            .is_err()
    {
        return Ok(());
    }
    info!("bimanual backbone: both arms reporting; governed streaming begins");

    // A gripper's latest measured opening fraction. `seed` gated on each side's
    // first reading and the watch never reverts to `None`, so the read is
    // infallible from here on.
    let opening = |gripper: &watch::Receiver<Option<GripperOpening>>| {
        gripper
            .borrow()
            .map(|g| g.fraction)
            .expect("seed gated on the first gripper opening")
    };
    // Track the last governed opening fraction per gripper: the governed
    // configuration's `prev`. Anchored on the measured jaws (here and whenever a
    // side idles) so governing always ramps from where the fingers really are;
    // the opening rate is read from the governor (its single owner) rather than
    // carried here.
    let opening_rate = governor.max_opening_rate_frac_s();
    let mut governed_openings = ArmPair::new(
        opening(&channels.left.gripper),
        opening(&channels.right.gripper),
    );
    // In-flight backbone-executed gripper moves, one single-flight slot per side.
    let mut gripper_moves: ArmPair<Option<GripperMove>> = ArmPair::new(None, None);

    let dt = cycle_period.as_secs_f64();
    // One low-pass per joint per arm, smoothing the published desired velocity. `main`
    // validates `0 < cutoff < Nyquist` at bringup, a strict superset of what `from_cutoff`
    // rejects, so construction cannot fail: build one filter and copy it per joint (the
    // same bringup-invariant pattern as `Pacer::new(...).expect(...)`).
    let filter = LowPassFilter::from_cutoff(velocity_filter_cutoff_hz, dt)
        .expect("velocity_filter_cutoff_hz is bringup-validated in (0, Nyquist)");
    let mut dq_filters = ArmPair::new([filter; ARM_DOF], [filter; ARM_DOF]);
    // The proximity readout is for human eyes, so publish it at ~20 Hz rather than
    // the control rate: one extra distance query every `readout_every` ticks.
    let readout_every = (0.05 / dt).round().max(1.0) as u64;
    let mut tick: u64 = 0;
    let mut pacer = Pacer::new(cycle_period).expect("control_rate_hz is asserted > 0 at startup");
    loop {
        // A move in progress consumes its side's streamed command: while the busy
        // flag is set, clear the command watch every tick so a setpoint still in
        // flight when the move was fired (the commander streams at the control
        // rate, so one is almost always queued) is wiped instead of surviving in
        // the watch and re-targeting the arm (or the jaws) when the move ends and
        // Follow resumes. Streaming and discrete moves are mutually exclusive per
        // side (firing a move disables that side's streaming), so this never drops
        // a command the operator still wants.
        for ch in [&channels.left, &channels.right] {
            if ch.busy.load(Ordering::Acquire) {
                ch.command.send_replace(None);
            }
            if ch.gripper_busy.load(Ordering::Acquire) {
                ch.gripper_command.send_replace(None);
            }
        }

        // Apply the latest commander controls (cheap no-ops when unchanged; invalid
        // band/speed values are rejected by the setters, keeping the last good).
        let cfg = *governor_config.borrow();
        governor.set_enabled(cfg.enabled);
        governor.set_band(cfg.d_stop, cfg.d_safe);
        planners.left.set_max_ee_velocity(cfg.max_ee_velocity_m_s);
        planners.right.set_max_ee_velocity(cfg.max_ee_velocity_m_s);
        let now = Instant::now();

        let arm_candidate = ArmPair::new(
            tick_arm(&mut channels.left, &mut planners.left, now).await,
            tick_arm(&mut channels.right, &mut planners.right, now).await,
        );
        let measured_openings = ArmPair::new(
            opening(&channels.left.gripper),
            opening(&channels.right.gripper),
        );

        // Service the backbone-executed gripper moves: admit a queued goal into a free
        // side and complete an in-flight move on the chase landing, cancellation,
        // or a budget overrun. The governed opening passed in is last tick's (this
        // tick's is computed below), which is also the chase base a new goal
        // budgets from.
        service_gripper_move(
            &mut gripper_moves.left,
            &mut channels.left,
            governed_openings.left,
            measured_openings.left,
            opening_rate,
            now,
        )
        .await;
        service_gripper_move(
            &mut gripper_moves.right,
            &mut channels.right,
            governed_openings.right,
            measured_openings.right,
            opening_rate,
            now,
        )
        .await;

        // Resolve each gripper's target for this tick: an in-flight move owns the
        // opening; otherwise the latest commander command drives it; otherwise the
        // side idles (never commanded, or unpaired), silent on the wire with the
        // governed opening anchored to the measured jaws.
        let targets = ArmPair::new(
            gripper_target(&gripper_moves.left, &channels.left),
            gripper_target(&gripper_moves.right, &channels.right),
        );
        if targets.left.is_none() {
            governed_openings.left = measured_openings.left;
        }
        if targets.right.is_none() {
            governed_openings.right = measured_openings.right;
        }

        // One governed configuration: the last published setpoints and openings
        // as `prev`, the velocity-limited chases as the candidate. The openings
        // chase their target at the opening rate exactly as the planner
        // velocity-limits the arm candidates; the governor then throttles, holds,
        // scans, and monitors everything through the same barrier.
        let prev = GovState::new(
            ArmPair::new(planners.left.setpoint(), planners.right.setpoint()),
            governed_openings,
        );
        // The real state for the governor's measured-state monitor. Arms fall
        // back to the held setpoint if a measurement is momentarily absent (only
        // before the first state, which `seed` already gated on), so a gap never
        // reads as a breach.
        let measured = GovState::new(
            ArmPair::new(
                channels
                    .left
                    .measured
                    .borrow()
                    .as_ref()
                    .map_or(prev.arms.left, |m| m.positions),
                channels
                    .right
                    .measured
                    .borrow()
                    .as_ref()
                    .map_or(prev.arms.right, |m| m.positions),
            ),
            measured_openings,
        );
        let chase_opening = |prev_frac: f64, target: Option<f64>| -> f64 {
            let t = target.unwrap_or(prev_frac);
            prev_frac + (t - prev_frac).clamp(-opening_rate * dt, opening_rate * dt)
        };
        let cand = GovState::new(
            arm_candidate,
            ArmPair::new(
                chase_opening(prev.openings.left, targets.left),
                chase_opening(prev.openings.right, targets.right),
            ),
        );
        let governed = governor.govern(&prev, &cand, &measured, dt);
        governed_openings = governed.openings;

        // Publish one governed setpoint per arm on its pairing slot; the slot
        // scopes the stream to its paired arm, so the message carries no arm_id.
        type BuildSetpoint = fn(JointVec, JointVec) -> peppygen::Result<peppylib::Payload>;
        for (side, planner, filters, arm_pub, build, prev_q, governed_q) in [
            (
                Side::Left,
                &mut planners.left,
                &mut dq_filters.left,
                &left_arm_pub,
                left_arm_link::arm_setpoints::build_message as BuildSetpoint,
                prev.arms.left,
                governed.arms.left,
            ),
            (
                Side::Right,
                &mut planners.right,
                &mut dq_filters.right,
                &right_arm_pub,
                right_arm_link::arm_setpoints::build_message as BuildSetpoint,
                prev.arms.right,
                governed.arms.right,
            ),
        ] {
            // Desired velocity is the per-tick position delta; low-pass it per joint so a
            // noisy stream does not drive the arm's Kd term into buzz. The published
            // position (`governed_q`) is untouched, so tracking is unaffected.
            let dq = filtered_velocity(filters, &governed_q, &prev_q, dt);
            planner.commit(governed_q);
            match build(governed_q, dq) {
                Ok(msg) => {
                    if let Err(e) = arm_pub.publish(msg).await {
                        warn!("arm_setpoints publish ({} arm): {e}", side.label());
                    }
                }
                Err(e) => error!("build arm_setpoints ({} arm): {e}", side.label()),
            }
        }

        // Publish each active side's governed opening fraction on its pairing
        // slot (the slot scopes the stream to its paired gripper, so the message
        // carries only the opening); an idle side stays silent and its gripper
        // holds the jaws.
        type BuildOpening = fn(f64) -> peppygen::Result<peppylib::Payload>;
        for (side, gripper_pub, build, opening_frac, active) in [
            (
                Side::Left,
                &left_gripper_pub,
                left_gripper_link::gripper_commands::build_message as BuildOpening,
                governed_openings.left,
                targets.left.is_some(),
            ),
            (
                Side::Right,
                &right_gripper_pub,
                right_gripper_link::gripper_commands::build_message as BuildOpening,
                governed_openings.right,
                targets.right.is_some(),
            ),
        ] {
            if !active {
                continue;
            }
            match build(opening_frac) {
                Ok(msg) => {
                    if let Err(e) = gripper_pub.publish(msg).await {
                        warn!("gripper_commands publish ({} gripper): {e}", side.label());
                    }
                }
                Err(e) => error!("build gripper_commands ({} gripper): {e}", side.label()),
            }
        }

        // Operator proximity readout (rate-limited): the nearest checked pair's
        // signed distance and link names, live regardless of the governor state,
        // plus the governor's current disposition of the commanded motion.
        if tick.is_multiple_of(readout_every)
            && let Some(p) = governor.proximity(&prev)
        {
            let guard = governor.guard();
            match collision_status::build_message(
                p.distance,
                p.link_a,
                p.link_b,
                guard == Guard::Throttling,
                guard == Guard::Stopped,
            ) {
                Ok(msg) => {
                    if let Err(e) = status_publisher.publish(msg).await {
                        warn!("collision_status publish: {e}");
                    }
                }
                Err(e) => error!("build collision_status: {e}"),
            }
        }
        tick += 1;
        tokio::select! {
            _ = token.cancelled() => return Ok(()),
            _ = pacer.pace() => {}
        }
    }
}

/// All senders on the measured-state channel dropped (its only producer is the
/// state listener task), so no measurement will ever arrive: seeding is abandoned.
struct Shutdown;

/// Wait for the arm's first measured state and first gripper opening, then seed
/// the planner's held setpoint from the measured pose (clamped into the joint
/// limits, so a power-up pose past a soft limit does not anchor the backbone
/// off-limit). Gating on both means the backbone never publishes a setpoint before a
/// real arm measurement exists, and never governs on the fully-open finger
/// default while the real jaws might be closed (open placement vacates the
/// between-jaws space a closed finger occupies).
/// Warns periodically while either stays silent so the wait is visible in the
/// log; `Err(Shutdown)` if a channel closes first.
async fn seed(
    channels: &mut ArmChannels,
    planner: &mut Planner,
    side: Side,
) -> Result<(), Shutdown> {
    wait_for_first(&mut channels.measured, side, "arm measured state").await?;
    wait_for_first(&mut channels.gripper, side, "gripper opening").await?;
    let q0 = channels
        .measured
        .borrow()
        .expect("gated on first state")
        .positions;
    planner.seed_from_measured(q0);
    Ok(())
}

/// Block until `latest` holds its first value, warning every
/// [`SEED_WAIT_WARN_PERIOD`] while `what` stays silent; `Err(Shutdown)` if the
/// channel closes first (its listener task died).
async fn wait_for_first<T>(
    latest: &mut watch::Receiver<Option<T>>,
    side: Side,
    what: &str,
) -> Result<(), Shutdown> {
    loop {
        match tokio::time::timeout(SEED_WAIT_WARN_PERIOD, latest.wait_for(Option::is_some)).await {
            Ok(Ok(_)) => return Ok(()),
            Ok(Err(_)) => {
                error!(
                    "{} {what} channel closed before its first value",
                    side.label()
                );
                return Err(Shutdown);
            }
            Err(_) => warn!(
                "{} {what} not reported yet; backbone waiting to stream",
                side.label()
            ),
        }
    }
}

/// Advance one arm's planner to its candidate setpoint for this tick: anchor on the
/// measured pose (or the held setpoint if no measurement yet), feed the latest
/// commander command, and admit any pending move goal.
async fn tick_arm(channels: &mut ArmChannels, planner: &mut Planner, now: Instant) -> JointVec {
    let measured_q = match *channels.measured.borrow() {
        Some(s) => s.positions,
        None => planner.setpoint(),
    };
    let command = channels.command.borrow().clone();
    planner
        .tick(
            measured_q,
            command,
            &mut channels.goals,
            &channels.busy,
            now,
        )
        .await
}

/// Landing threshold for the governed chase, in opening fraction. Purely
/// numerical: the rate-limited chase lands on its target up to IEEE rounding
/// residue, and the governor passes an unthrottled candidate through
/// bit-exact, so anything past this is a real clamp. Nanometer-scale jaw
/// travel, orders of magnitude below actuator resolution; goal satisfaction
/// is the caller's judgment from the reported `final_opening`.
const OPENING_LANDED_FRAC: f64 = 1e-9;

/// Nominal duration (s) of a gripper move admitted with the chase at
/// `governed_frac`: the commanded travel at the opening rate. The gripper
/// analog of the arm servo's plan-time rollout, graded by the same
/// [`motion_timed_out`] rule.
fn gripper_move_budget_s(governed_frac: f64, target_frac: f64, opening_rate_frac_s: f64) -> f64 {
    (target_frac - governed_frac).abs() / opening_rate_frac_s
}

/// Admit a queued gripper goal into a free side and drive an in-flight move to
/// its terminal: the governed chase landing on the target completes it,
/// cancellation ends it, and overrunning the budget sized at admission fails it
/// (a collision-governed clamp short of the target lands here, so the message
/// says so). Every terminal reports the measured jaws as `final_opening`; like
/// the arm's post-move reached check, judging that against the goal belongs to
/// the caller. The busy slot releases with the move on every path.
async fn service_gripper_move(
    mv: &mut Option<GripperMove>,
    channels: &mut ArmChannels,
    governed_frac: f64,
    measured_frac: f64,
    opening_rate_frac_s: f64,
    now: Instant,
) {
    if mv.is_none()
        && let Ok(goal) = channels.gripper_goals.try_recv()
    {
        *mv = Some(GripperMove {
            target_frac: goal.opening,
            ctx: goal.ctx,
            started: now,
            budget_s: gripper_move_budget_s(governed_frac, goal.opening, opening_rate_frac_s),
            _busy: BusyGuard(channels.gripper_busy.clone()),
        });
    }
    let Some(m) = mv.as_ref() else { return };
    let elapsed_s = now.duration_since(m.started).as_secs_f64();
    let landed = (governed_frac - m.target_frac).abs() <= OPENING_LANDED_FRAC;
    let (success, message, cancelled) = if m.ctx.is_cancelled() {
        (false, "goal cancelled".to_string(), true)
    } else if landed {
        (true, "move complete".to_string(), false)
    } else if motion_timed_out(elapsed_s, m.budget_s) {
        (
            false,
            format!(
                "overran {MOTION_TIMEOUT_FACTOR:.0}x its {:.1}s nominal travel, short of the target (a collision-governed clamp ends here)",
                m.budget_s
            ),
            false,
        )
    } else {
        return;
    };
    let m = mv.take().expect("in-flight move checked above");
    let result = if cancelled {
        m.ctx
            .complete_cancelled(success, message, measured_frac, elapsed_s)
            .await
    } else {
        m.ctx
            .complete(success, message, measured_frac, elapsed_s)
            .await
    };
    if let Err(e) = result {
        error!("move_gripper complete: {e}");
    }
}

/// The side's target opening fraction for this tick: an in-flight backbone-executed
/// move owns it; otherwise the commander's streamed command drives it;
/// otherwise `None` (idle: before any command, or on an unpaired side).
fn gripper_target(mv: &Option<GripperMove>, channels: &ArmChannels) -> Option<f64> {
    if let Some(m) = mv {
        return Some(m.target_frac);
    }
    follow_gripper_target(&channels.gripper_command.borrow().clone())
}

/// Resolve the streamed opening target: the latest commander command clamped
/// into `[0, 1]`, or `None` when none has arrived. The stream is paired to one
/// producer, so there is nothing to arbitrate; a stopped producer just leaves
/// the last opening in place, held by the gripper.
fn follow_gripper_target(command: &Option<GripperCommand>) -> Option<f64> {
    command.as_ref().map(|c| c.opening.clamp(0.0, 1.0))
}

/// The published desired velocity: per joint, the tick's position delta scaled to a rate
/// and low-passed. `filters` carries the per-joint state across ticks, so the smoothing is
/// over time, not within a tick. Only the velocity is shaped; the position (`governed_q`)
/// is published as-is.
fn filtered_velocity(
    filters: &mut [LowPassFilter; ARM_DOF],
    governed_q: &JointVec,
    prev_q: &JointVec,
    dt: f64,
) -> JointVec {
    std::array::from_fn(|j| filters[j].filter((governed_q[j] - prev_q[j]) / dt))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cmd(opening: f64) -> Option<GripperCommand> {
        Some(GripperCommand { opening })
    }

    const DT: f64 = 0.01;
    // The node default; well below the 50 Hz Nyquist so it actually attenuates.
    const CUTOFF_HZ: f64 = 15.0;

    fn dq_filters() -> [LowPassFilter; ARM_DOF] {
        std::array::from_fn(|_| LowPassFilter::from_cutoff(CUTOFF_HZ, DT).unwrap())
    }

    #[test]
    fn filtered_velocity_differentiates_position_into_a_rate() {
        // A steady position delta of 0.01 rad/tick at 100 Hz is a 1 rad/s velocity; the
        // first tick seeds on that value (no startup transient).
        let mut filters = dq_filters();
        let prev = [0.0; ARM_DOF];
        let q = [0.01; ARM_DOF];
        let dq = filtered_velocity(&mut filters, &q, &prev, DT);
        assert!(
            dq.iter().all(|v| (v - 1.0).abs() < 1e-12),
            "delta/dt is the rate"
        );
    }

    #[test]
    fn filtered_velocity_attenuates_a_noisy_stream() {
        // A jittering position (alternating +/-) makes the raw per-tick velocity swing
        // by +/- (2*amp/dt); the low-pass carries state across ticks and damps it.
        let mut filters = dq_filters();
        let amp = 0.001;
        let mut prev = [0.0; ARM_DOF];
        let mut worst_raw: f64 = 0.0;
        let mut worst_filtered: f64 = 0.0;
        for k in 0..200 {
            let q = [if k % 2 == 0 { amp } else { -amp }; ARM_DOF];
            let raw = (q[0] - prev[0]) / DT;
            let filtered = filtered_velocity(&mut filters, &q, &prev, DT)[0];
            if k > 1 {
                worst_raw = worst_raw.max(raw.abs());
                worst_filtered = worst_filtered.max(filtered.abs());
            }
            prev = q;
        }
        assert!(
            worst_filtered < worst_raw * 0.5,
            "the low-pass more than halves the jitter amplitude ({worst_filtered} vs {worst_raw})"
        );
    }

    #[test]
    fn follow_clamps_the_wire_fraction() {
        // In-range passes through; past-open and negative commands clamp into
        // [0, 1] at this boundary.
        assert_eq!(follow_gripper_target(&cmd(0.5)), Some(0.5));
        assert_eq!(follow_gripper_target(&cmd(1.5)), Some(1.0));
        assert_eq!(follow_gripper_target(&cmd(-0.5)), Some(0.0));
    }

    #[test]
    fn follow_stays_idle_without_a_command() {
        assert_eq!(follow_gripper_target(&None), None);
    }

    #[test]
    fn a_consumed_command_holds_the_move_endpoint_until_a_newer_one() {
        // The gripper twin of the arm handoff: an accepted move_gripper clears
        // the side's command watch (`send_replace(None)` in the handler), so the
        // gripper follows nothing new and holds the move's endpoint until a
        // command that arrives after the clear. Locks the contract; the handler
        // performing the clear is covered by the live regression.
        let (tx, rx) = watch::channel(None);

        tx.send_replace(cmd(0.6));
        assert_eq!(
            follow_gripper_target(&rx.borrow()),
            Some(0.6),
            "a live streamed opening is followed"
        );

        tx.send_replace(None);
        assert_eq!(
            follow_gripper_target(&rx.borrow()),
            None,
            "a consumed command leaves the gripper holding the move endpoint"
        );

        tx.send_replace(cmd(0.3));
        assert_eq!(
            follow_gripper_target(&rx.borrow()),
            Some(0.3),
            "an opening after the move resumes following"
        );
    }

    // The gripper budget mirrors the arm servo's rollout: the commanded travel
    // at the opening rate, so a long move earns a long leash and a short one
    // stays tight.
    #[test]
    fn gripper_budget_is_the_commanded_travel_at_the_opening_rate() {
        const RATE: f64 = 3.0;
        assert_eq!(gripper_move_budget_s(0.0, 1.0, RATE), 1.0 / RATE);
        // Binary-exact travel (0.25) so the equality is exact.
        assert_eq!(gripper_move_budget_s(0.5, 0.75, 2.0), 0.125);
        // Direction of travel does not matter.
        assert_eq!(
            gripper_move_budget_s(0.8, 0.2, RATE),
            gripper_move_budget_s(0.2, 0.8, RATE)
        );
    }

    #[test]
    fn gripper_move_times_out_at_the_shared_factor_over_budget() {
        // A clamped full-travel move fails once it overruns 2x its budget,
        // exactly as the arm servo grades its rollout.
        let budget = gripper_move_budget_s(0.0, 1.0, 3.0);
        assert!(!motion_timed_out(
            budget * MOTION_TIMEOUT_FACTOR - 0.01,
            budget
        ));
        assert!(motion_timed_out(
            budget * MOTION_TIMEOUT_FACTOR + 0.01,
            budget
        ));
    }

    // The chase's landing arithmetic (`prev + (t - prev)`) can leave IEEE
    // rounding residue; the landing threshold absorbs it so a finished chase
    // cannot dangle one ulp short of terminal.
    #[test]
    fn landing_threshold_absorbs_chase_rounding_residue() {
        let target: f64 = 0.7;
        let mut governed: f64 = 0.13;
        for _ in 0..1000 {
            let step = (target - governed).clamp(-0.03, 0.03);
            governed += step;
        }
        assert!((governed - target).abs() <= OPENING_LANDED_FRAC);
    }
}
