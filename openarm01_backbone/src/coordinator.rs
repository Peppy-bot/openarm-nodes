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
use peppygen::emitted_topics::openarm01_collision_status::v1::collision_status;
use peppygen::emitted_topics::openarm01_arm_governed_setpoints::v1::arm_governed_setpoints;
use tokio::sync::{mpsc, watch};
use tracing::{error, info, warn};

use crate::governor::Governor;
use crate::pacer::Pacer;
use crate::planner::{Goal, Planner};
use crate::streams::{GovernorConfig, JointCommand, MeasuredState};
use crate::{ARM_ID_LEFT, ARM_ID_RIGHT, ArmPair, JointVec};

/// One arm's inbound channels into the coordinator: the operator command stream,
/// the measured state, the accepted-goal queue, and the single-flight busy flag.
pub struct ArmChannels {
    pub command: watch::Receiver<Option<JointCommand>>,
    pub measured: watch::Receiver<Option<MeasuredState>>,
    pub goals: mpsc::Receiver<Goal>,
    pub busy: Arc<AtomicBool>,
}

/// Run the coordination loop forever. Holds the governor and both planners.
pub async fn run(
    runner: Arc<NodeRunner>,
    mut governor: Governor,
    mut planners: ArmPair<Planner>,
    mut channels: ArmPair<ArmChannels>,
    governor_config: watch::Receiver<GovernorConfig>,
    cycle_period: Duration,
) {
    let publisher = match arm_governed_setpoints::declare_publisher(&runner).await {
        Ok(p) => p,
        Err(e) => return error!("declare governed_setpoints publisher: {e}"),
    };
    let status_publisher = match collision_status::declare_publisher(&runner).await {
        Ok(p) => p,
        Err(e) => return error!("declare collision_status publisher: {e}"),
    };

    // Hold each arm's real pose, not a neutral zero: wait for the first measured
    // state from both arms and seed the held setpoint there before publishing.
    if seed(&mut channels.left, &mut planners.left, "left").await.is_err()
        || seed(&mut channels.right, &mut planners.right, "right").await.is_err()
    {
        return;
    }
    info!("bimanual hub: both arms reporting; governed streaming begins");

    let dt = cycle_period.as_secs_f64();
    // The proximity readout is for human eyes, so publish it at ~20 Hz rather than
    // the control rate: one extra distance query every `readout_every` ticks.
    let readout_every = (0.05 / dt).round().max(1.0) as u64;
    let mut tick: u64 = 0;
    let mut pacer = Pacer::new(cycle_period);
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
        let governed = governor.govern(&prev, &candidate, dt);

        // Publish one governed setpoint per arm. The single publisher just changes
        // arm_id; each follower keeps its own arm. A follower never starves as long
        // as it consumes in a tight loop (receive decoupled from publish), the way
        // the hub's own state listeners and the real arm already do.
        for (arm_id, planner, prev_q, governed_q) in [
            (ARM_ID_LEFT, &mut planners.left, prev.left, governed.left),
            (ARM_ID_RIGHT, &mut planners.right, prev.right, governed.right),
        ] {
            let dq: JointVec = std::array::from_fn(|j| (governed_q[j] - prev_q[j]) / dt);
            planner.commit(governed_q);
            match arm_governed_setpoints::build_message(arm_id, governed_q, dq) {
                Ok(msg) => {
                    if let Err(e) = publisher.publish(msg).await {
                        warn!("governed_setpoints publish (arm {arm_id}): {e}");
                    }
                }
                Err(e) => error!("build governed_setpoints (arm {arm_id}): {e}"),
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
        pacer.pace().await;
    }
}

/// The measured-state channel closed before the first state arrived: the node is
/// shutting down, so seeding is abandoned.
struct Shutdown;

/// Wait for the arm's first measured state and seed the planner's held setpoint
/// there, so the hub never publishes a setpoint before a real measurement exists.
/// `Err(Shutdown)` if the channel closes first.
async fn seed(channels: &mut ArmChannels, planner: &mut Planner, side: &str) -> Result<(), Shutdown> {
    if channels.measured.wait_for(Option::is_some).await.is_err() {
        error!("{side} arm measured-state channel closed before first state");
        return Err(Shutdown);
    }
    let q0 = channels.measured.borrow().expect("gated on first state").positions;
    planner.commit(q0);
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
    planner.tick(measured_q, command, &mut channels.goals, &channels.busy, now).await
}
