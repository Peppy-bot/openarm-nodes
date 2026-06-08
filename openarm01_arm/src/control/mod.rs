//! The arm's control: a single task that owns the motors and runs a state
//! machine at a fixed rate. Each tick the current [`Mode`] commands the motors
//! exactly once and returns the next mode, so every transition is a return
//! value: `Hold` admits a goal and becomes a move; a move runs to its
//! terminal (completion, cancellation, abort, or timeout), completes its goal,
//! and becomes `Hold` at the last commanded setpoint.

mod cartesian_move;
mod feedback;
mod joint_move;

use std::sync::atomic::AtomicBool;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use peppygen::exposed_actions::openarm01_arm::v1::{move_arm, move_arm_joints};
use peppygen::{NodeRunner, Result};
use srs_model::nalgebra::Isometry3;
use srs_model::Limit;
use tokio::sync::mpsc;
use tracing::info;

use crate::actions::{self, Goal};
use crate::friction;
use crate::pacer::Pacer;
use crate::{ARM_DOF, JointVec};
use cartesian_move::CartesianMove;
use joint_move::JointMove;
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
    /// This arm's joint position limits, parsed from the URDF (per side, via the
    /// `base_link`). Used to reject out-of-range joint move targets.
    pub limits: [Limit; ARM_DOF],
}

/// Everything one control tick reads or writes: the measured state and
/// feedforward computed once at the top of the tick, plus the shared resources a
/// mode needs to command the motors, admit goals, and complete them. Modes take
/// it by reference; the loop rebuilds it every tick.
struct TickIo<'a> {
    arm: &'a Mutex<ArmCan>,
    cfg: &'a ControlConfig,
    model: &'a srs_model::Arm,
    goals: &'a mut mpsc::Receiver<Goal>,
    /// The single-flight flag claimed by the action handlers at goal acceptance.
    /// A mode releases it exactly where it completes a goal's context, so every
    /// claim is released once: at a motion's terminal, or at a failed admission.
    busy: &'a AtomicBool,
    /// Measured joint positions this tick.
    q: JointVec,
    /// Gravity + Coriolis + friction feedforward at the measured state.
    ff_tau: JointVec,
    /// End-effector pose at the measured state, in the arm base frame.
    ee_base: Isometry3<f64>,
    now: Instant,
}

/// The control state machine; see the module docs for the transition story.
// TODO: add streaming modes for external setpoints at the control rate, alongside
// `Hold`, `JointMove`, and `CartesianMove`: a joint-space stream (e.g. an
// openarm-teleop follower tracking a continuous joint target) and a Cartesian
// stream (a continuous pose target, IK-solved per tick like `CartesianMove` but
// with no fixed endpoint). Each is one more variant whose tick polls its input
// and exits to `Hold`.
enum Mode {
    /// Holding a fixed setpoint with gravity/Coriolis/friction feedforward plus PD
    /// (kp/kd): the state between motions (each motion's final config is held) and
    /// the only state that admits a new goal. The PD term holds position, so the
    /// arm stays put rather than sagging at rest.
    Hold { setpoint: JointVec },
    /// Tracking a joint-space trajectory for an accepted `move_arm_joints` goal.
    JointMove(JointMove),
    /// Tracking a Cartesian trajectory for an accepted `move_arm` goal: each tick
    /// samples a pose and solves IK in-process for the joint setpoint.
    CartesianMove(CartesianMove),
}

/// A motion's terminal: its normal result, or cancellation by the caller, which
/// completes via the cancelled terminal with `success = false` in the result
/// payload (the motion stopped short of its target).
enum Completion {
    Done { success: bool, message: &'static str },
    Cancelled,
}

impl Mode {
    /// Run one control tick: command the motors once and return the next mode.
    async fn tick(self, io: &mut TickIo<'_>) -> Mode {
        match self {
            Mode::Hold { setpoint } => hold_tick(setpoint, io).await,
            Mode::JointMove(m) => m.tick(io).await,
            Mode::CartesianMove(m) => m.tick(io).await,
        }
    }
}

/// Hold the latched setpoint, then admit at most one pending goal: joint goals
/// start their trajectory directly; Cartesian goals are planned first (see
/// [`CartesianMove::start`]) and rejected there if unreachable. While a motion
/// runs the goal channel is left alone (the handlers reject via `busy`), so
/// admission is exclusive to `Hold`.
async fn hold_tick(setpoint: JointVec, io: &mut TickIo<'_>) -> Mode {
    command(io, &setpoint, &ZERO);
    match io.goals.try_recv() {
        Ok(Goal::Joints(g)) => Mode::JointMove(JointMove::start(g, io)),
        Ok(Goal::Cartesian(g)) => match CartesianMove::start(g, io).await {
            Some(m) => Mode::CartesianMove(m),
            // Unreachable path: the goal was already completed (failed) at admission.
            None => Mode::Hold { setpoint },
        },
        Err(_) => Mode::Hold { setpoint },
    }
}

/// Spawn the arm's control: a single task that owns the motors (the only motor
/// writer, so hold, joint, and Cartesian moves can never command concurrently)
/// plus the `move_arm_joints` and `move_arm` action handlers, which share one goal
/// channel and one single-flight `busy` flag. Both actions are exposed here, before
/// anything is spawned, so a failed registration fails node bringup instead of
/// panicking inside a detached task. The control task owns the srs_model `Arm` and
/// computes the gravity/Coriolis feedforward (and, for Cartesian moves, the inverse
/// kinematics) in-process every tick.
pub async fn spawn(runner: &NodeRunner, arm: Arc<Mutex<ArmCan>>, cfg: ControlConfig, model: srs_model::Arm) -> Result<()> {
    let joints_action = move_arm_joints::ActionHandle::expose(runner).await?;
    let cartesian_action = move_arm::ActionHandle::expose(runner).await?;
    let busy = Arc::new(AtomicBool::new(false));
    let (goal_tx, goal_rx) = mpsc::channel::<Goal>(1);
    tokio::spawn(run_control(arm, cfg.clone(), goal_rx, busy.clone(), model));
    tokio::spawn(actions::run_move_arm_joints(joints_action, cfg.limits, goal_tx.clone(), busy.clone()));
    tokio::spawn(actions::run_move_arm(cartesian_action, goal_tx, busy));
    Ok(())
}

/// The single motor-owning control loop. Runs forever at `cfg.cycle_period`:
/// reads the measured state, computes the feedforward, and runs one [`Mode`]
/// tick, which commands the motors and yields the next mode. Starts holding the
/// power-on pose (never lunge to zero on boot).
async fn run_control(
    arm: Arc<Mutex<ArmCan>>,
    cfg: ControlConfig,
    mut goals: mpsc::Receiver<Goal>,
    busy: Arc<AtomicBool>,
    mut model: srs_model::Arm,
) {
    let mut pacer = Pacer::new(cfg.cycle_period);
    let (q0, _) = read_state(&arm, cfg.recv_timeout_us);
    let mut mode = Mode::Hold { setpoint: q0 };

    info!("control loop started (in-process gravity compensation + IK)");
    loop {
        let (q, qdot) = read_state(&arm, cfg.recv_timeout_us);
        let (ff_tau, ee_base) = feedforward(&mut model, &q, &qdot);
        let mut io = TickIo {
            arm: &arm,
            cfg: &cfg,
            model: &model,
            goals: &mut goals,
            busy: &busy,
            q,
            ff_tau,
            ee_base,
            now: Instant::now(),
        };
        mode = mode.tick(&mut io).await;
        pacer.pace().await;
    }
}

/// One tick of rigid-body feedforward: gravity and Coriolis from the posed chain
/// (which carries the distal gripper payload) plus locally computed friction, all
/// at full weight, so the PD term only corrects residual error. Poses the chain
/// once and also returns the EE pose (base frame) that the same evaluation yields
/// for free, used by Cartesian admission and feedback.
fn feedforward(model: &mut srs_model::Arm, q: &JointVec, qdot: &JointVec) -> (JointVec, Isometry3<f64>) {
    let posed = model.at(q);
    let gravity = posed.gravity_torques();
    let coriolis = posed.coriolis_torques(qdot);
    let ee_base = posed.ee_pose();
    let friction = friction::torques(&friction::V1, qdot);
    let ff_tau = std::array::from_fn(|i| gravity[i] + coriolis[i] + friction[i]);
    (ff_tau, ee_base)
}

/// Command the motors once: this tick's feedforward plus PD to the desired
/// position/velocity. The single control task is the only caller, so locking
/// here can never contend with another writer.
fn command(io: &TickIo<'_>, q_des: &JointVec, dq_des: &JointVec) {
    let mut a = io.arm.lock().unwrap_or_else(|e| e.into_inner());
    a.mit_control(&io.cfg.kp, &io.cfg.kd, q_des, dq_des, &io.ff_tau);
}

/// Read the measured joint state (positions + velocities) one time.
fn read_state(arm: &Mutex<ArmCan>, recv_timeout_us: i32) -> (JointVec, JointVec) {
    let mut a = arm.lock().unwrap_or_else(|e| e.into_inner());
    a.refresh_all();
    a.recv_all(recv_timeout_us);
    let state = a.get_state();
    (state.positions, state.velocities)
}

/// Decompose a world-frame pose into the interface's `(position, quaternion)`
/// arrays: position `[x, y, z]` (m) and quaternion `[x, y, z, w]`.
fn world_pose_arrays(pose: &Isometry3<f64>) -> ([f64; 3], [f64; 4]) {
    let t = pose.translation.vector;
    let r = pose.rotation;
    ([t.x, t.y, t.z], [r.i, r.j, r.k, r.w])
}

fn fmt_joints(v: &JointVec) -> String {
    let parts: Vec<String> = v.iter().map(|x| format!("{:.3}", x)).collect();
    format!("[{}]", parts.join(", "))
}
