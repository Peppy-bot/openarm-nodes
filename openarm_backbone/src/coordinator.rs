//! The bimanual coordination loop. Every tick it advances both arms' planners
//! and both grippers' openings to candidate setpoints, governs the whole step
//! against the self-collision model in one call (arms and openings are one
//! governed configuration), and publishes the governed per-arm setpoints and
//! per-gripper openings. One loop owns the governor (the single collision
//! model), both planners, and the hub-executed gripper moves, so everything is
//! always governed together against a consistent configuration, and the
//! governed result is fed back so the next tick chases from where each DOF was
//! actually allowed to go.

use std::sync::Arc;
use std::sync::atomic::AtomicBool;
use std::time::{Duration, Instant};

use peppygen::NodeRunner;
use peppygen::emitted_topics::openarm_collision_status::v1::collision_status;
use peppygen::exposed_actions::move_gripper;
use peppygen::pairings::{left_arm_link, left_gripper_link, right_arm_link, right_gripper_link};
use peppylib::runtime::CancellationToken;
use tokio::sync::{mpsc, watch};
use tracing::{error, info, warn};

use control_core::Pacer;

use peppylib::messaging::ProducerRef;

use crate::governor::{GovState, Governor, Guard};
use crate::planner::{BusyGuard, Goal, Planner, fresh};
use crate::streams::{GovernorConfig, GripperCommand, GripperOpening, JointCommand, MeasuredState};
use crate::{ArmPair, JointVec, Side};

/// How long [`seed`] waits for an arm's first measured state before warning that
/// the hub is still blocked, so a silent arm is visible in the log instead of an
/// indefinite quiet stall.
const SEED_WAIT_WARN_PERIOD: Duration = Duration::from_secs(2);

/// One arm's inbound channels into the coordinator: the operator arm command
/// stream, the operator gripper opening command stream, the measured arm state,
/// the measured gripper opening, the accepted-goal queues, and the single-flight
/// busy flags (one for arm moves, one for gripper moves).
pub struct ArmChannels {
    pub command: watch::Receiver<Option<JointCommand>>,
    pub gripper_command: watch::Receiver<Option<GripperCommand>>,
    pub measured: watch::Receiver<Option<MeasuredState>>,
    pub gripper: watch::Receiver<Option<GripperOpening>>,
    pub goals: mpsc::Receiver<Goal>,
    pub busy: Arc<AtomicBool>,
    pub gripper_goals: mpsc::Receiver<GripperGoal>,
    pub gripper_busy: Arc<AtomicBool>,
}

/// The coordinator's run parameters: the control-cycle period, how long a
/// command stream stays fresh before its input counts as released (an operator
/// who stops streaming freezes at the last governed setpoint), and the
/// hub-executed gripper moves' completion tolerance and timeout.
pub struct RunConfig {
    pub cycle_period: Duration,
    pub stream_timeout: Duration,
    pub gripper_tolerance_m: f64,
    pub gripper_move_timeout: Duration,
}

/// An accepted `move_gripper` goal handed to the coordinator, which executes it
/// through the same per-tick governing as everything else (the gripper analog of
/// [`Goal`] for the arms).
pub struct GripperGoal {
    pub target_m: f64,
    pub ctx: move_gripper::GoalContext,
}

/// A hub-executed gripper move in flight: the opening chases `target_frac`
/// through the governor until the measured jaws converge, the goal is cancelled,
/// or the deadline lapses (a governed clamp short of the target ends here). The
/// busy guard releases the side's single-flight slot on any exit.
struct GripperMove {
    target_frac: f64,
    ctx: move_gripper::GoalContext,
    started: Instant,
    deadline: Instant,
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
        stream_timeout,
        gripper_tolerance_m,
        gripper_move_timeout,
    } = config;
    // One publisher per pairing slot (arms and grippers alike). Publishing while
    // a slot is unpaired is a legal no-op, so the hub streams governed setpoints
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
    info!("bimanual hub: both arms reporting; governed streaming begins");

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
    // jaw travel and the opening rate are read from the governor (their single
    // owner) rather than carried here.
    let jaw_open_m = governor.jaw_open_m();
    let opening_rate = governor.max_opening_rate_frac_s();
    let mut governed_openings = ArmPair::new(
        opening(&channels.left.gripper),
        opening(&channels.right.gripper),
    );
    // In-flight hub-executed gripper moves, one single-flight slot per side, and
    // each side's producer-locked follow on the operator's opening stream.
    let mut gripper_moves: ArmPair<Option<GripperMove>> = ArmPair::new(None, None);
    let mut gripper_follows: ArmPair<Option<GripperFollow>> = ArmPair::new(None, None);

    let dt = cycle_period.as_secs_f64();
    // The proximity readout is for human eyes, so publish it at ~20 Hz rather than
    // the control rate: one extra distance query every `readout_every` ticks.
    let readout_every = (0.05 / dt).round().max(1.0) as u64;
    let mut tick: u64 = 0;
    let mut pacer = Pacer::new(cycle_period).expect("control_rate_hz is asserted > 0 at startup");
    loop {
        // Apply the latest operator controls (cheap no-ops when unchanged; invalid
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

        // Service the hub-executed gripper moves: admit a queued goal into a free
        // side and complete an in-flight move on measured convergence,
        // cancellation, or its deadline.
        service_gripper_move(
            &mut gripper_moves.left,
            &mut channels.left,
            measured_openings.left,
            jaw_open_m,
            gripper_tolerance_m,
            gripper_move_timeout,
            now,
        )
        .await;
        service_gripper_move(
            &mut gripper_moves.right,
            &mut channels.right,
            measured_openings.right,
            jaw_open_m,
            gripper_tolerance_m,
            gripper_move_timeout,
            now,
        )
        .await;

        // Resolve each gripper's target for this tick: an in-flight move owns the
        // opening; otherwise a fresh operator command drives it; otherwise the
        // side idles, silent on the wire (the gripper's own watchdog holds the
        // jaws) with the tracked opening re-anchored to the measured jaws so a
        // resume ramps from where the fingers really are.
        let targets = ArmPair::new(
            gripper_target(
                &gripper_moves.left,
                &mut gripper_follows.left,
                &channels.left,
                jaw_open_m,
                stream_timeout,
                now,
            ),
            gripper_target(
                &gripper_moves.right,
                &mut gripper_follows.right,
                &channels.right,
                jaw_open_m,
                stream_timeout,
                now,
            ),
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
        for (side, planner, arm_pub, build, prev_q, governed_q) in [
            (
                Side::Left,
                &mut planners.left,
                &left_arm_pub,
                left_arm_link::arm_setpoints::build_message as BuildSetpoint,
                prev.arms.left,
                governed.arms.left,
            ),
            (
                Side::Right,
                &mut planners.right,
                &right_arm_pub,
                right_arm_link::arm_setpoints::build_message as BuildSetpoint,
                prev.arms.right,
                governed.arms.right,
            ),
        ] {
            let dq: JointVec = std::array::from_fn(|j| (governed_q[j] - prev_q[j]) / dt);
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

        // Publish each active side's governed opening on its pairing slot (the
        // slot scopes the stream to its paired gripper, so the message carries
        // only the opening, in metres); an idle side stays silent and its
        // gripper's watchdog holds the jaws.
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
            match build(opening_frac * jaw_open_m) {
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
/// limits, so a power-up pose past a soft limit does not anchor the hub
/// off-limit). Gating on both means the hub never publishes a setpoint before a
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
                "{} {what} not reported yet; hub waiting to stream",
                side.label()
            ),
        }
    }
}

/// Advance one arm's planner to its candidate setpoint for this tick: anchor on the
/// measured pose (or the held setpoint if no measurement yet), feed the latest
/// operator command, and admit any pending move goal.
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

/// Admit a queued gripper goal into a free side and drive an in-flight move to
/// its terminal: measured convergence within the tolerance completes it,
/// cancellation ends it, and the deadline fails it (a collision-governed clamp
/// short of the target also lands here, so the message says so). The busy slot
/// releases with the move on every path.
async fn service_gripper_move(
    mv: &mut Option<GripperMove>,
    channels: &mut ArmChannels,
    measured_frac: f64,
    jaw_open_m: f64,
    tolerance_m: f64,
    timeout: Duration,
    now: Instant,
) {
    if mv.is_none()
        && let Ok(goal) = channels.gripper_goals.try_recv()
    {
        *mv = Some(GripperMove {
            target_frac: (goal.target_m / jaw_open_m).clamp(0.0, 1.0),
            ctx: goal.ctx,
            started: now,
            deadline: now + timeout,
            _busy: BusyGuard(channels.gripper_busy.clone()),
        });
    }
    let Some(m) = mv.as_ref() else { return };
    let converged = (measured_frac - m.target_frac).abs() * jaw_open_m <= tolerance_m;
    let (success, message, cancelled) = if m.ctx.is_cancelled() {
        (false, "goal cancelled", true)
    } else if converged {
        (true, "move complete", false)
    } else if now >= m.deadline {
        (
            false,
            "timed out short of the target (a collision-governed clamp ends here)",
            false,
        )
    } else {
        return;
    };
    let m = mv.take().expect("in-flight move checked above");
    let elapsed = now.duration_since(m.started).as_secs_f64();
    let measured_m = vec![measured_frac * jaw_open_m];
    let result = if cancelled {
        m.ctx
            .complete_cancelled(success, message.into(), measured_m, elapsed)
            .await
    } else {
        m.ctx
            .complete(success, message.into(), measured_m, elapsed)
            .await
    };
    if let Err(e) = result {
        error!("move_gripper complete: {e}");
    }
}

/// The side's target opening fraction for this tick: an in-flight hub-executed
/// move owns it; otherwise the operator's streamed command drives it through the
/// producer-locked follow; otherwise `None` (idle).
fn gripper_target(
    mv: &Option<GripperMove>,
    follow: &mut Option<GripperFollow>,
    channels: &ArmChannels,
    jaw_open_m: f64,
    stream_timeout: Duration,
    now: Instant,
) -> Option<f64> {
    if let Some(m) = mv {
        return Some(m.target_frac);
    }
    follow_gripper_target(
        follow,
        &channels.gripper_command.borrow().clone(),
        jaw_open_m,
        stream_timeout,
        now,
    )
}

/// The locked gripper-command producer and the target fraction it drives: the
/// opening analog of the planner's follow lock, sharing its semantics (the
/// slot may be bound to several producers, so without the lock two producers
/// would interleave and a foreign stream could hold the deadman open).
struct GripperFollow {
    producer: ProducerRef,
    target_frac: f64,
    last_seq: u64,
    last_fresh: Instant,
}

/// Resolve the streamed opening target: chase the locked operator command,
/// acquiring or releasing the producer lock by freshness (the opening analog of
/// the planner's `follow_target`). The wire metres parse into the governed
/// fraction here, clamped into the jaw travel at the boundary. `None` when no
/// producer is live.
fn follow_gripper_target(
    lock: &mut Option<GripperFollow>,
    command: &Option<GripperCommand>,
    jaw_open_m: f64,
    stream_timeout: Duration,
    now: Instant,
) -> Option<f64> {
    let parse = |position_m: f64| (position_m / jaw_open_m).clamp(0.0, 1.0);
    match lock.as_mut() {
        Some(l) => {
            if let Some(c) = command
                && c.producer == l.producer
                && c.seq != l.last_seq
            {
                l.target_frac = parse(c.position_m);
                l.last_seq = c.seq;
                l.last_fresh = now;
            }
            if !fresh(l.last_fresh, now, stream_timeout) {
                *lock = None;
                return None;
            }
            Some(l.target_frac)
        }
        None => {
            let c = command
                .as_ref()
                .filter(|c| fresh(c.recv_at, now, stream_timeout))?;
            let target_frac = parse(c.position_m);
            *lock = Some(GripperFollow {
                producer: c.producer.clone(),
                target_frac,
                last_seq: c.seq,
                last_fresh: now,
            });
            Some(target_frac)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const JAW: f64 = 0.08;
    const TIMEOUT: Duration = Duration::from_millis(500);

    fn producer(instance: &str) -> ProducerRef {
        ProducerRef::new("core".to_string(), instance.to_string())
    }

    fn cmd(instance: &str, seq: u64, position_m: f64, recv_at: Instant) -> Option<GripperCommand> {
        Some(GripperCommand {
            seq,
            producer: producer(instance),
            recv_at,
            position_m,
        })
    }

    #[test]
    fn follow_parses_the_wire_metres_into_a_clamped_fraction() {
        let now = Instant::now();
        let mut lock = None;
        // Half the jaw travel parses to 0.5; past-travel and negative commands
        // clamp into [0, 1] at this boundary.
        assert_eq!(
            follow_gripper_target(&mut lock, &cmd("op", 1, 0.04, now), JAW, TIMEOUT, now),
            Some(0.5)
        );
        assert_eq!(
            follow_gripper_target(&mut lock, &cmd("op", 2, 1.0, now), JAW, TIMEOUT, now),
            Some(1.0)
        );
        assert_eq!(
            follow_gripper_target(&mut lock, &cmd("op", 3, -0.5, now), JAW, TIMEOUT, now),
            Some(0.0)
        );
    }

    #[test]
    fn follow_ignores_a_foreign_producer_while_locked() {
        let now = Instant::now();
        let mut lock = None;
        assert_eq!(
            follow_gripper_target(&mut lock, &cmd("op", 1, 0.04, now), JAW, TIMEOUT, now),
            Some(0.5)
        );
        // A fresh command from a different producer must not steal the follow:
        // the locked target stands.
        assert_eq!(
            follow_gripper_target(&mut lock, &cmd("intruder", 9, 0.08, now), JAW, TIMEOUT, now),
            Some(0.5),
            "a foreign producer hijacked the gripper follow"
        );
    }

    #[test]
    fn follow_releases_on_staleness_then_relocks_to_the_live_producer() {
        let start = Instant::now();
        let mut lock = None;
        assert_eq!(
            follow_gripper_target(&mut lock, &cmd("op", 1, 0.04, start), JAW, TIMEOUT, start),
            Some(0.5)
        );
        // The locked producer goes silent past the deadman window: the follow
        // releases (None; the coordinator goes silent and re-anchors)...
        let later = start + TIMEOUT + Duration::from_millis(1);
        assert_eq!(
            follow_gripper_target(&mut lock, &cmd("op", 1, 0.04, start), JAW, TIMEOUT, later),
            None,
            "a stale command kept driving the gripper"
        );
        assert!(lock.is_none(), "the stale lock was not released");
        // ...and a fresh producer (any producer) may then acquire it.
        assert_eq!(
            follow_gripper_target(
                &mut lock,
                &cmd("other", 1, 0.08, later),
                JAW,
                TIMEOUT,
                later
            ),
            Some(1.0)
        );
    }

    #[test]
    fn follow_holds_the_last_target_between_messages_while_fresh() {
        let start = Instant::now();
        let mut lock = None;
        assert_eq!(
            follow_gripper_target(&mut lock, &cmd("op", 1, 0.04, start), JAW, TIMEOUT, start),
            Some(0.5)
        );
        // No new message (same seq) inside the window: the target stands, driven
        // by the lock's freshness rather than the command's arrival time.
        let mid = start + TIMEOUT / 2;
        assert_eq!(
            follow_gripper_target(&mut lock, &cmd("op", 1, 0.04, start), JAW, TIMEOUT, mid),
            Some(0.5)
        );
    }

    #[test]
    fn follow_stays_idle_without_a_fresh_command() {
        let now = Instant::now();
        let mut lock = None;
        assert_eq!(
            follow_gripper_target(&mut lock, &None, JAW, TIMEOUT, now),
            None
        );
        let stale = cmd("op", 1, 0.04, now - TIMEOUT * 2);
        assert_eq!(
            follow_gripper_target(&mut lock, &stale, JAW, TIMEOUT, now),
            None,
            "a leftover command acquired the follow"
        );
    }
}
