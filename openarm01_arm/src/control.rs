//! The real arm's control: a single task that owns the motors and runs an MIT
//! control loop following the hub's governed setpoint. The bimanual coordination
//! hub (openarm01_backbone) owns all trajectory generation, stream following, and
//! collision governing, and streams the resolved (q_des, dq_des) per arm; this
//! loop adds only the realtime feedforward (gravity/Coriolis/friction the hub
//! cannot compute remotely) and a final clamp to the joint limits, then commands
//! the motors. There is no mode state machine and no streaming logic here.
//!
//! On shutdown the loop disables the motors and lets the arm go limp. It does not
//! park to a pose: a collision-aware return-to-home is the hub's job (it sees both
//! arms), and a local straight-line joint path would be collision-blind and could
//! command the two arms into each other.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use peppygen::{NodeRunner, Result};
use peppylib::runtime::CancellationToken;
use srs_model::Limit;
use tokio::sync::oneshot;
use tracing::{error, info};

use control_core::Pacer;

use crate::friction;
use crate::stream::{GovernedSetpoint, StreamWiring};
use crate::{ARM_DOF, JointVec};
use openarm_can::ArmCan;

/// All-zero joint vector: the desired velocity sent while holding a pose.
const ZERO: JointVec = [0.0; ARM_DOF];

/// Settle time after disabling the motors on shutdown, before draining ACK
/// frames. Mirrors the ROS2 v10_simple_hardware on_deactivate sleep.
const POST_DISABLE_SLEEP: Duration = Duration::from_millis(100);

#[derive(Clone)]
pub struct ControlConfig {
    pub kp: JointVec,
    pub kd: JointVec,
    pub cycle_period: Duration,
    pub recv_timeout_us: i32,
    /// Joint position limits, parsed from the URDF, the final guard clamp applied
    /// to every governed setpoint before it reaches the motors.
    pub limits: [Limit; ARM_DOF],
}

/// Spawn the arm's control and supervise it. `run_control` is the sole motor
/// writer, so a fire-and-forget spawn would let a panic pass unobserved, leaving
/// the node up with the motors uncontrolled and no control loop. The supervisor
/// watches the control task and, if it ever stops, cancels the node so it restarts
/// under supervision instead of serving blind. `shutdown_tx` is signalled once a
/// clean stop has disabled the motors, so main.rs can then release the lock.
pub async fn spawn(
    runner: &NodeRunner,
    arm: Arc<Mutex<ArmCan>>,
    cfg: ControlConfig,
    model: srs_model::Arm,
    wiring: StreamWiring,
    shutdown_tx: oneshot::Sender<()>,
) -> Result<()> {
    let token = runner.cancellation_token().clone();
    let control = tokio::spawn(run_control(
        arm.clone(),
        cfg,
        model,
        wiring,
        token.clone(),
        shutdown_tx,
    ));
    tokio::spawn(supervise(control, arm, token));
    Ok(())
}

/// Watch the sole motor-writer task. A clean stop returns `Ok` after the loop has
/// already disabled the motors; a panic returns `Err` with the motors still live,
/// so disable them here. Either way cancel the node: on a clean stop the token is
/// already cancelled (idempotent), and on a panic this converts a silently dead
/// control loop into a node restart rather than an arm left with no controller.
async fn supervise(
    control: tokio::task::JoinHandle<()>,
    arm: Arc<Mutex<ArmCan>>,
    token: CancellationToken,
) {
    if let Err(join_error) = control.await {
        error!(%join_error, "control loop terminated unexpectedly; disabling motors");
        disable_motors(&arm);
    }
    token.cancel();
}

/// The single motor-owning control loop. Each tick reads the measured state,
/// computes the feedforward, reports the measured state upward, and commands the
/// motors to the latest governed setpoint (clamped), holding the measured pose
/// until the first governed setpoint arrives or whenever the stream is empty. On
/// cancellation it disables the motors and signals `shutdown_tx`.
async fn run_control(
    arm: Arc<Mutex<ArmCan>>,
    cfg: ControlConfig,
    mut model: srs_model::Arm,
    wiring: StreamWiring,
    shutdown: Shutdown,
) {
    let mut pacer =
        Pacer::new(cfg.cycle_period).expect("control_rate_hz is non-zero (period derives from it)");
    info!("control loop started (MIT follower of governed setpoints, in-process feedforward)");
    loop {
        let (q, qdot) = read_state(&arm, cfg.recv_timeout_us);
        let ff_tau = feedforward(&mut model, &q, &qdot);
        wiring
            .measured
            .send_replace(Some(crate::stream::MeasuredState {
                positions: q,
                velocities: qdot,
            }));

        // Follow the latest governed setpoint; hold the measured pose (zero
        // desired velocity) until the hub's stream is live, so the arm never
        // lunges before the hub is up.
        let (q_des, dq_des) = match *wiring.governed.borrow() {
            Some(GovernedSetpoint { q_des, dq_des }) => {
                clamp_setpoint_to_limits(&q_des, &dq_des, &cfg.limits)
            }
            None => (q, ZERO),
        };

        command(&arm, &cfg, &ff_tau, &q_des, &dq_des);
        // Biased so a cancelled token always wins over an already-due (overrun)
        // tick: on shutdown break out and disable the motors below.
        tokio::select! {
            biased;
            _ = shutdown.token.cancelled() => break,
            _ = pacer.pace() => {}
        }
    }

    // Cancelled: disable the motors and let the arm go limp. A graceful, collision-
    // aware park is the hub's responsibility (it sees both arms); the arm on its own
    // must not drive to a fixed pose, because a collision-blind straight joint path
    // could command the two arms into each other. main.rs awaits `shutdown_tx` so
    // the lock is released only after the motors are off.
    info!("control loop stopping: disabling motors");
    disable_motors(&arm);
    tokio::time::sleep(POST_DISABLE_SLEEP).await;
    {
        let mut a = arm.lock().unwrap_or_else(|e| e.into_inner());
        a.recv_all(cfg.recv_timeout_us);
    }
    // A dropped receiver (main.rs already exited) is fine; nothing to do.
    let _ = shutdown.done.send(());
    info!("control loop stopped (motors disabled)");
}

/// One tick of rigid-body feedforward: gravity and Coriolis from the posed chain
/// (carrying the distal gripper payload) plus locally computed friction, so the
/// PD term only corrects residual error.
fn feedforward(model: &mut srs_model::Arm, q: &JointVec, qdot: &JointVec) -> JointVec {
    let posed = model.at(q);
    let gravity = posed.gravity_torques();
    let coriolis = posed.coriolis_torques(qdot);
    let friction = friction::torques(&friction::V1, qdot);
    std::array::from_fn(|i| gravity[i] + coriolis[i] + friction[i])
}

/// Command the motors once: this tick's feedforward plus PD to the governed
/// position/velocity.
fn command(
    arm: &Mutex<ArmCan>,
    cfg: &ControlConfig,
    ff_tau: &JointVec,
    q_des: &JointVec,
    dq_des: &JointVec,
) {
    let mut a = arm.lock().unwrap_or_else(|e| e.into_inner());
    a.mit_control(&cfg.kp, &cfg.kd, q_des, dq_des, ff_tau);
}

/// Clamp a governed setpoint into the joint position limits, the final guard
/// before the motors. A joint whose target is pinned at a limit also has its
/// desired velocity zeroed when that velocity points further past the stop, so the
/// MIT controller is never commanded to drive outward through a hard limit (the
/// `kd * (dq_des - qdot)` term cannot add outward torque at the wall). Inward
/// (recovering) velocity is preserved.
fn clamp_setpoint_to_limits(
    q: &JointVec,
    dq: &JointVec,
    limits: &[Limit; ARM_DOF],
) -> (JointVec, JointVec) {
    let q_clamped: JointVec = std::array::from_fn(|i| q[i].clamp(limits[i].lo, limits[i].hi));
    let dq_clamped: JointVec = std::array::from_fn(|i| {
        let driving_below = q[i] < limits[i].lo && dq[i] < 0.0;
        let driving_above = q[i] > limits[i].hi && dq[i] > 0.0;
        if driving_below || driving_above {
            0.0
        } else {
            dq[i]
        }
    });
    (q_clamped, dq_clamped)
}

/// Disable all motors so the arm goes limp. Recovers a poisoned lock (unwrap into
/// the inner guard) so the disable runs even if the control loop panicked holding
/// it, since going limp is the safe failure state.
fn disable_motors(arm: &Mutex<ArmCan>) {
    let mut a = arm.lock().unwrap_or_else(|e| e.into_inner());
    a.disable_all();
}

/// Read the measured joint state (positions + velocities) one time.
fn read_state(arm: &Mutex<ArmCan>, recv_timeout_us: i32) -> (JointVec, JointVec) {
    let mut a = arm.lock().unwrap_or_else(|e| e.into_inner());
    a.refresh_all();
    a.recv_all(recv_timeout_us);
    let state = a.get_state();
    (state.positions, state.velocities)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn unit_limits() -> [Limit; ARM_DOF] {
        std::array::from_fn(|_| Limit { lo: -1.0, hi: 1.0 })
    }

    #[test]
    fn clamps_position_and_zeros_outward_velocity_at_a_limit() {
        // Joint 0: target above hi with outward (+) velocity. Joint 1: target below
        // lo with outward (-) velocity. Both clamp to the limit and zero the velocity.
        let q = [2.0, -2.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let dq = [0.5, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0];
        let (qc, dqc) = clamp_setpoint_to_limits(&q, &dq, &unit_limits());
        assert_eq!(qc[0], 1.0);
        assert_eq!(qc[1], -1.0);
        assert_eq!(dqc[0], 0.0, "outward velocity at the upper stop is zeroed");
        assert_eq!(dqc[1], 0.0, "outward velocity at the lower stop is zeroed");
    }

    #[test]
    fn preserves_inward_recovery_velocity_past_a_limit() {
        // Past the limits but velocity points back toward range: keep it so the arm
        // can recover rather than being pinned outside.
        let q = [2.0, -2.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let dq = [-0.5, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0];
        let (_, dqc) = clamp_setpoint_to_limits(&q, &dq, &unit_limits());
        assert_eq!(dqc[0], -0.5, "inward velocity above hi is preserved");
        assert_eq!(dqc[1], 0.5, "inward velocity below lo is preserved");
    }

    #[test]
    fn leaves_in_range_setpoints_untouched() {
        let q = [0.5, -0.5, 0.0, 0.3, -0.3, 0.1, -0.1];
        let dq = [0.4, -0.4, 0.2, -0.2, 0.1, -0.1, 0.0];
        let (qc, dqc) = clamp_setpoint_to_limits(&q, &dq, &unit_limits());
        assert_eq!(qc, q, "in-range positions are unchanged");
        assert_eq!(dqc, dq, "in-range velocities are unchanged");
    }
}
