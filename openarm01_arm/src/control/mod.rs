//! The real arm's control: a single task that owns the motors and runs an MIT
//! control loop following the hub's governed setpoint. The bimanual coordination
//! hub (openarm01_backbone) owns all trajectory generation, stream following, and
//! collision governing, and streams the resolved (q_des, dq_des) per arm; this
//! loop adds only the realtime feedforward (gravity/Coriolis/friction the hub
//! cannot compute remotely) and a final clamp to the joint limits, then commands
//! the motors. There is no mode state machine and no streaming logic here.
//!
//! On shutdown the loop eases the arm back to a known ready pose (the sole motor
//! writer, reusing the dynamics model) before disabling the motors, so the arm
//! settles at a well-conditioned pose instead of dropping when power cuts.

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use peppygen::{NodeRunner, Result};
use peppylib::runtime::CancellationToken;
use srs_model::Limit;
use tokio::sync::oneshot;
use tracing::info;

use crate::friction;
use crate::pacer::Pacer;
use crate::stream::{GovernedSetpoint, StreamWiring};
use crate::trajectory::JointTrajectory;
use crate::{ARM_DOF, JointVec};
use openarm_can::ArmCan;

/// All-zero joint vector: the desired velocity sent while holding a pose.
const ZERO: JointVec = [0.0; ARM_DOF];

/// Non-singular rest configuration the arm eases to on shutdown, before the
/// motors disable. The arm powers off wherever it hung, often on the straight-arm
/// singularity, so this is a known, well-conditioned pose: the elbow (J4) is bent
/// a hair above its URDF lower limit; every other joint rests at 0.
const READY_POSE: JointVec = [0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0];

/// Requested duration (s) of the shutdown return-to-ready move, floored at the
/// joint velocity limits. Sized to fit inside the shutdown grace window alongside
/// the motor disable and lock release.
const PARK_DURATION_S: f64 = 3.0;

/// Per-joint velocity limits (rad/s) flooring the shutdown park duration, from the
/// OpenArm V1.0 URDF (symmetric across sides). Only the park samples a trajectory;
/// normal motion is the hub's already rate-limited governed setpoint.
const PARK_MAX_JOINT_VELOCITY_RAD_S: JointVec =
    [16.754666, 16.754666, 5.445426, 5.445426, 20.943946, 20.943946, 20.943946];

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
    /// Temporary bring-up safety: when set, every tick holds the measured pose
    /// instead of the governed one and logs the target it would have tracked.
    pub dry_run: bool,
}

/// Assert the shutdown ready pose is inside the parsed joint limits, during
/// bringup, so a model whose limits exclude it fails loudly before any hardware
/// is touched rather than panicking inside the spawned control task.
pub fn assert_ready_pose_in_limits(limits: &[Limit; ARM_DOF]) {
    assert!(
        READY_POSE.iter().zip(limits).all(|(&q, l)| l.contains(q)),
        "READY_POSE outside joint limits: {READY_POSE:?}",
    );
}

/// Shutdown coordination for the control loop: `main.rs` cancels `token` to ask
/// the loop to stop, and once it has eased the arm to the ready pose and disabled
/// the motors it signals `done` so `main.rs` can release the lock and exit.
struct Shutdown {
    token: CancellationToken,
    done: oneshot::Sender<()>,
}

/// Spawn the arm's control: a single task that owns the motors (the only motor
/// writer) and follows the hub's governed setpoint stream. No actions are exposed
/// here - the hub owns move admission. `shutdown_tx` is signalled once the loop
/// has parked and disabled the motors, so main.rs can then release the lock.
pub async fn spawn(
    runner: &NodeRunner,
    arm: Arc<Mutex<ArmCan>>,
    cfg: ControlConfig,
    model: srs_model::Arm,
    wiring: StreamWiring,
    shutdown_tx: oneshot::Sender<()>,
) -> Result<()> {
    let token = runner.cancellation_token().clone();
    tokio::spawn(run_control(arm, cfg, model, wiring, token, shutdown_tx));
    Ok(())
}

/// The single motor-owning control loop. Each tick reads the measured state,
/// computes the feedforward, reports the measured state upward, and commands the
/// motors to the latest governed setpoint (clamped), holding the measured pose
/// until the first governed setpoint arrives or whenever the stream is empty. On
/// cancellation it eases to the ready pose, disables the motors, and signals
/// `shutdown_tx`.
async fn run_control(
    arm: Arc<Mutex<ArmCan>>,
    cfg: ControlConfig,
    mut model: srs_model::Arm,
    wiring: StreamWiring,
    shutdown: Shutdown,
) {
    let mut pacer = Pacer::new(cfg.cycle_period);
    info!("control loop started (MIT follower of governed setpoints, in-process feedforward)");
    loop {
        let (q, qdot) = read_state(&arm, cfg.recv_timeout_us);
        let ff_tau = feedforward(&mut model, &q, &qdot);
        wiring.measured.send_replace(Some(crate::stream::MeasuredState {
            positions: q,
            velocities: qdot,
        }));

        // Follow the latest governed setpoint; hold the measured pose (zero
        // desired velocity) until the hub's stream is live, so the arm never
        // lunges before the hub is up.
        let (q_des, dq_des) = match *wiring.governed.borrow() {
            Some(GovernedSetpoint { q_des, dq_des }) => (clamp_to_limits(&q_des, &cfg.limits), dq_des),
            None => (q, ZERO),
        };

        command(&arm, &cfg, &ff_tau, &q, &q_des, &dq_des);
        // Biased so a cancelled token always wins over an already-due (overrun)
        // tick: on shutdown break out and run the return-to-ready + disable below.
        tokio::select! {
            biased;
            _ = shutdown.token.cancelled() => break,
            _ = pacer.pace() => {}
        }
    }

    // Cancelled: ease back to the ready pose (the sole motor writer, reusing the
    // dynamics model) so the arm settles at a known pose instead of dropping when
    // power cuts, then disable the motors. main.rs awaits `shutdown_tx`, keeping
    // this task alive through the park before it releases the lock and exits.
    info!("control loop stopping: easing to ready pose, then disabling motors");
    return_to_ready(&arm, &cfg, &mut model).await;
    {
        // unwrap_or_else: recover even if poisoned so disable_all() always runs.
        let mut a = arm.lock().unwrap_or_else(|e| e.into_inner());
        a.disable_all();
    }
    tokio::time::sleep(POST_DISABLE_SLEEP).await;
    {
        let mut a = arm.lock().unwrap_or_else(|e| e.into_inner());
        a.recv_all(cfg.recv_timeout_us);
    }
    // A dropped receiver (main.rs already exited) is fine; nothing to do.
    let _ = shutdown.done.send(());
    info!("control loop stopped (motors disabled)");
}

/// Best-effort return to [`READY_POSE`] on shutdown: eases from the measured pose
/// over [`PARK_DURATION_S`] with the same gravity/Coriolis/friction feedforward as
/// a normal command, so the arm settles at a known pose instead of dropping when
/// the motors disable. Respects the per-joint velocity limits, so a pose far from
/// ready takes longer than [`PARK_DURATION_S`] rather than lunging.
async fn return_to_ready(arm: &Mutex<ArmCan>, cfg: &ControlConfig, model: &mut srs_model::Arm) {
    let (q0, _) = read_state(arm, cfg.recv_timeout_us);
    let trajectory = JointTrajectory::new(q0, READY_POSE, PARK_MAX_JOINT_VELOCITY_RAD_S, PARK_DURATION_S);
    let mut pacer = Pacer::new(cfg.cycle_period);
    loop {
        let (q, qdot) = read_state(arm, cfg.recv_timeout_us);
        let ff_tau = feedforward(model, &q, &qdot);
        let now = Instant::now();
        let (q_des, dq_des) = trajectory.sample(now);
        {
            let mut a = arm.lock().unwrap_or_else(|e| e.into_inner());
            a.mit_control(&cfg.kp, &cfg.kd, &q_des, &dq_des, &ff_tau);
        }
        if trajectory.is_complete(now) {
            break;
        }
        pacer.pace().await;
    }
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
/// position/velocity. Under `dry_run` it instead holds the measured pose and logs
/// the target it would have tracked, so every motion path is neutered here.
fn command(arm: &Mutex<ArmCan>, cfg: &ControlConfig, ff_tau: &JointVec, q: &JointVec, q_des: &JointVec, dq_des: &JointVec) {
    let mut a = arm.lock().unwrap_or_else(|e| e.into_inner());
    if cfg.dry_run {
        log_dry_target(Instant::now(), q_des);
        a.mit_control(&cfg.kp, &cfg.kd, q, &ZERO, ff_tau);
        return;
    }
    a.mit_control(&cfg.kp, &cfg.kd, q_des, dq_des, ff_tau);
}

/// Clamp a governed target into the joint position limits, the final guard before
/// the motors.
fn clamp_to_limits(q: &JointVec, limits: &[Limit; ARM_DOF]) -> JointVec {
    std::array::from_fn(|i| q[i].clamp(limits[i].lo, limits[i].hi))
}

/// Period between dry-run target logs, throttling the control tick to a readable rate.
const DRY_LOG_PERIOD: Duration = Duration::from_millis(200);

fn log_dry_target(now: Instant, q_des: &JointVec) {
    static LAST_LOG: Mutex<Option<Instant>> = Mutex::new(None);
    let mut last = LAST_LOG.lock().unwrap_or_else(|e| e.into_inner());
    if last.is_none_or(|t| now.duration_since(t) >= DRY_LOG_PERIOD) {
        *last = Some(now);
        info!("dry-run: would follow {}", fmt_joints(q_des));
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

fn fmt_joints(v: &JointVec) -> String {
    let parts: Vec<String> = v.iter().map(|x| format!("{:.3}", x)).collect();
    format!("[{}]", parts.join(", "))
}
