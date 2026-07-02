//! The bimanual coordination loop. Every tick it advances both arms' planners to
//! candidate setpoints, governs the joint step against the self-collision model,
//! and publishes the governed per-arm setpoint. One loop owns the governor (the
//! single collision model) and both planners, so the two arms are always governed
//! together against a consistent pair of configurations, and the governed result
//! is fed back into each planner so the next tick chases from where the arm was
//! actually allowed to go.

use std::sync::Arc;
use std::sync::atomic::AtomicBool;
use std::time::{Duration, Instant};

use peppygen::NodeRunner;
use peppygen::emitted_topics::openarm_arm_governed_setpoints::v1::arm_governed_setpoints;
use peppygen::emitted_topics::openarm_collision_status::v1::collision_status;
use peppylib::runtime::CancellationToken;
use tokio::sync::{mpsc, watch};
use tracing::{error, info, warn};

use control_core::Pacer;

use crate::governor::Governor;
use crate::planner::{Goal, Planner};
use crate::streams::{GovernorConfig, GripperOpening, JointCommand, MeasuredState};
use crate::{ArmPair, JointVec, Side};

/// How long [`seed`] waits for an arm's first measured state before warning that
/// the hub is still blocked, so a silent arm is visible in the log instead of an
/// indefinite quiet stall.
const SEED_WAIT_WARN_PERIOD: Duration = Duration::from_secs(2);

/// The opening fraction the fingers sit at until the first gripper reading
/// arrives: fully open, the widest finger envelope (analogous to the arm's
/// measured state falling back to the held setpoint before its first reading).
const FULLY_OPEN: f64 = 1.0;

/// One arm's inbound channels into the coordinator: the operator command stream,
/// the measured state, the measured gripper opening, the accepted-goal queue, and
/// the single-flight busy flag.
pub struct ArmChannels {
    pub command: watch::Receiver<Option<JointCommand>>,
    pub measured: watch::Receiver<Option<MeasuredState>>,
    pub gripper: watch::Receiver<Option<GripperOpening>>,
    pub goals: mpsc::Receiver<Goal>,
    pub busy: Arc<AtomicBool>,
}

/// Run the coordination loop. Holds the governor and both planners. Runs until
/// the node's cancellation token fires; returns `Err` if a publisher cannot be
/// declared at bringup. Any return takes the node down (the supervisor in `main`
/// treats it as fatal).
#[allow(clippy::too_many_arguments)] // distinct loop inputs: runtime, governor, planners, channels, config, timing, token
pub async fn run(
    runner: Arc<NodeRunner>,
    mut governor: Governor,
    mut planners: ArmPair<Planner>,
    mut channels: ArmPair<ArmChannels>,
    governor_config: watch::Receiver<GovernorConfig>,
    cycle_period: Duration,
    jaw_open_m: f64,
    token: CancellationToken,
) -> peppygen::Result<()> {
    let publisher = match arm_governed_setpoints::declare_publisher(&runner).await {
        Ok(p) => p,
        Err(e) => {
            error!("declare governed_setpoints publisher: {e}");
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

        let candidate = ArmPair::new(
            tick_arm(&mut channels.left, &mut planners.left, now).await,
            tick_arm(&mut channels.right, &mut planners.right, now).await,
        );

        // Govern the bimanual step from the last published setpoints to the
        // candidates: the closing-velocity barrier limits only the gap-closing
        // component of the joint step, per arm.
        let prev = ArmPair::new(planners.left.setpoint(), planners.right.setpoint());
        // The arms' real pose for the governor's measured-state monitor. Falls back
        // to the held setpoint if an arm has no measurement this instant (only before
        // the first state, which `seed` already gated on), so a momentary gap never
        // reads as a breach.
        let measured = ArmPair::new(
            channels
                .left
                .measured
                .borrow()
                .as_ref()
                .map_or(prev.left, |m| m.positions),
            channels
                .right
                .measured
                .borrow()
                .as_ref()
                .map_or(prev.right, |m| m.positions),
        );

        // Place the collision fingers at each gripper's live opening so the barrier
        // sees the fingers where they actually are, not their full swept envelope.
        // Treated like the measured arm state: the latest reading is used every
        // tick, falling back to fully open only before the first reading arrives.
        governor.set_gripper_openings(
            opening_fraction(*channels.left.gripper.borrow(), jaw_open_m),
            opening_fraction(*channels.right.gripper.borrow(), jaw_open_m),
        );
        let governed = governor.govern(&prev, &candidate, &measured, dt);

        // Publish one governed setpoint per arm. The single publisher just changes
        // arm_id; each follower keeps its own arm. A follower never starves as long
        // as it consumes in a tight loop (receive decoupled from publish), the way
        // the hub's own state listeners and the real arm already do.
        for (side, planner, prev_q, governed_q) in [
            (Side::Left, &mut planners.left, prev.left, governed.left),
            (Side::Right, &mut planners.right, prev.right, governed.right),
        ] {
            let dq: JointVec = std::array::from_fn(|j| (governed_q[j] - prev_q[j]) / dt);
            planner.commit(governed_q);
            match arm_governed_setpoints::build_message(side.arm_id(), governed_q, dq) {
                Ok(msg) => {
                    if let Err(e) = publisher.publish(msg).await {
                        warn!("governed_setpoints publish ({} arm): {e}", side.label());
                    }
                }
                Err(e) => error!("build governed_setpoints ({} arm): {e}", side.label()),
            }
        }

        // Operator proximity readout (throttled): the nearest checked pair's
        // signed distance and link names, live regardless of the governor state.
        if tick.is_multiple_of(readout_every)
            && let Some(p) = governor.proximity(&prev)
        {
            match collision_status::build_message(p.distance, p.link_a, p.link_b) {
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

/// Wait for the arm's first measured state and seed the planner's held setpoint
/// there (clamped into the joint limits, so a power-up pose past a soft limit does
/// not anchor the hub off-limit), so the hub never publishes a setpoint before a
/// real measurement exists.
/// Warns periodically while an arm stays silent so the wait is visible in the log;
/// `Err(Shutdown)` if the measured-state channel closes first.
async fn seed(
    channels: &mut ArmChannels,
    planner: &mut Planner,
    side: Side,
) -> Result<(), Shutdown> {
    loop {
        match tokio::time::timeout(
            SEED_WAIT_WARN_PERIOD,
            channels.measured.wait_for(Option::is_some),
        )
        .await
        {
            Ok(Ok(_)) => break,
            Ok(Err(_)) => {
                error!(
                    "{} arm measured-state channel closed before first state",
                    side.label()
                );
                return Err(Shutdown);
            }
            Err(_) => warn!(
                "{} arm has not reported measured state yet; hub waiting to stream",
                side.label()
            ),
        }
    }
    let q0 = channels
        .measured
        .borrow()
        .expect("gated on first state")
        .positions;
    planner.seed_from_measured(q0);
    Ok(())
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

/// Map a measured gripper opening to the `[0, 1]` fraction the collision model
/// places its fingers at (0 = closed, 1 = fully open): `width / jaw_open_m`,
/// clamped. A missing reading (none received yet) falls back to [`FULLY_OPEN`],
/// the widest, most conservative envelope.
fn opening_fraction(reading: Option<GripperOpening>, jaw_open_m: f64) -> f64 {
    match reading {
        Some(g) => (g.width_m / jaw_open_m).clamp(0.0, 1.0),
        None => FULLY_OPEN,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const JAW_OPEN_M: f64 = 0.044;

    #[test]
    fn a_reading_maps_width_to_a_clamped_fraction() {
        let half = GripperOpening {
            width_m: JAW_OPEN_M / 2.0,
        };
        assert!((opening_fraction(Some(half), JAW_OPEN_M) - 0.5).abs() < 1e-9);
        // Out-of-range widths clamp into [0, 1] rather than escaping the finger travel.
        let over = GripperOpening {
            width_m: JAW_OPEN_M * 1.5,
        };
        assert_eq!(opening_fraction(Some(over), JAW_OPEN_M), 1.0);
        let under = GripperOpening { width_m: -0.01 };
        assert_eq!(opening_fraction(Some(under), JAW_OPEN_M), 0.0);
    }

    #[test]
    fn a_missing_reading_falls_back_to_fully_open() {
        // No gripper telemetry yet: the fingers sit at the widest envelope.
        assert_eq!(opening_fraction(None, JAW_OPEN_M), FULLY_OPEN);
    }
}
