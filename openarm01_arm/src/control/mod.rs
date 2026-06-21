//! The arm's control: a single task that owns the motors and runs a state
//! machine at a fixed rate. Each tick the current [`Mode`] commands the motors
//! exactly once and returns the next mode, so every transition is a return
//! value: `Startup` eases to the ready pose and becomes `Follow`; `Follow` is
//! the ambient default that chases the active command stream (or holds when none
//! is streaming) and preempts into a move when a goal arrives; a move runs to its
//! terminal (completion, cancellation, or abort) and returns to `Follow` at the
//! last commanded setpoint.

mod cartesian_move;
mod chase;
mod follow;
mod joint_move;
mod startup;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use peppygen::exposed_actions::openarm01_arm::v1::{move_arm, move_arm_joints};
use peppygen::{NodeRunner, Result};
use srs_model::nalgebra::Isometry3;
use srs_model::{Jacobian, Limit};
use tokio::sync::{mpsc, watch};
use tracing::info;

use crate::actions::{self, Goal};
use crate::friction;
use crate::pacer::Pacer;
use crate::stream::{JointCommand, StreamWiring};
use crate::{ARM_DOF, JointVec};
use cartesian_move::CartesianMove;
use follow::Follow;
use joint_move::JointMove;
use openarm_can::ArmCan;
use startup::StartupMove;

pub(crate) use startup::assert_ready_pose_in_limits;

/// All-zero joint vector, the zero desired velocity sent alongside a held or
/// commanded position.
const ZERO: JointVec = [0.0; ARM_DOF];

#[derive(Clone)]
pub struct ControlConfig {
    pub kp: JointVec,
    pub kd: JointVec,
    pub cycle_period: Duration,
    pub recv_timeout_us: i32,
    pub max_joint_velocity_rad_s: JointVec,
    /// End-effector linear-speed cap (m/s) for the `Follow` chase. The chase step
    /// is scaled so the hand never translates faster than this, so a first
    /// command or a producer hiccup far from the current pose eases over rather
    /// than lunging. A step that does not move the hand (null to the linear
    /// Jacobian, e.g. self-motion) is left at the motor rate, bounded only by
    /// `max_joint_velocity_rad_s` for continuity.
    pub max_ee_velocity_m_s: f64,
    /// This arm's joint position limits, parsed from the URDF (per side, via the
    /// `base_link`). Used to reject out-of-range joint move targets and to clamp
    /// streamed joint targets.
    pub limits: [Limit; ARM_DOF],
    /// How long `Follow` keeps following a stream source after its last command
    /// before releasing the lock and holding.
    pub stream_timeout: Duration,
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
    /// The single-flight flag claimed by a move action handler at goal acceptance
    /// and held through startup. A move releases it at its terminal; startup
    /// releases it once when it reaches the ready pose. `Follow` runs with it
    /// clear, so a move goal always preempts.
    busy: &'a AtomicBool,
    /// Latest streamed joint command, fed by the `joint_commands` listener and
    /// consumed by [`Follow`].
    joint_stream: &'a watch::Receiver<Option<JointCommand>>,
    /// Measured joint positions this tick.
    q: JointVec,
    /// Gravity + Coriolis + friction feedforward at the measured state.
    ff_tau: JointVec,
    /// End-effector pose at the measured state, in the arm base frame.
    ee_base: Isometry3<f64>,
    /// Geometric Jacobian at the measured state (base frame): rows 0..3 map joint
    /// rates to EE linear velocity, used to cap the `Follow` chase by hand speed.
    jacobian: Jacobian,
    now: Instant,
}

/// The control state machine; see the module docs for the transition story.
enum Mode {
    /// Easing from the measured power-on configuration to the ready pose, once,
    /// before any goal is admitted (`busy` is held through it). Entered when no
    /// producer is streaming at boot: the arm powers off wherever it hung, often
    /// on the straight-arm singularity, so this brings it somewhere
    /// well-conditioned before it holds and waits for a producer or a goal.
    Startup(StartupMove),
    /// The ambient default: chase the active command stream under the joint and
    /// velocity limits (or hold the last setpoint when none is streaming), and
    /// preempt into a move when a goal arrives. The only state that admits a goal.
    Follow(Follow),
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
            Mode::Startup(s) => s.tick(io),
            Mode::Follow(f) => f.tick(io).await,
            Mode::JointMove(m) => m.tick(io).await,
            Mode::CartesianMove(m) => m.tick(io).await,
        }
    }
}

/// Spawn the arm's control: a single task that owns the motors (the only motor
/// writer, so follow and moves can never command concurrently) plus the
/// `move_arm_joints` and `move_arm` action handlers, which share one goal channel
/// and one single-flight `busy` flag. Both actions are exposed here, before
/// anything is spawned, so a failed registration fails node bringup instead of
/// panicking inside a detached task. The control task owns the srs_model `Arm`
/// and computes the gravity/Coriolis feedforward (and, for Cartesian moves, the
/// inverse kinematics) in-process every tick; it reads the streamed joint
/// setpoint and reports the measured state through `wiring`, its connections to
/// the stream tasks.
pub async fn spawn(
    runner: &NodeRunner,
    arm: Arc<Mutex<ArmCan>>,
    cfg: ControlConfig,
    model: srs_model::Arm,
    wiring: StreamWiring,
) -> Result<()> {
    let joints_action = move_arm_joints::ActionHandle::expose(runner).await?;
    let cartesian_action = move_arm::ActionHandle::expose(runner).await?;
    // Born busy: the Startup state holds the flag through its move to the ready
    // pose, so a goal arriving during startup is rejected as busy instead of
    // accepted and silently queued behind a motion the caller never requested.
    let busy = Arc::new(AtomicBool::new(true));
    let (goal_tx, goal_rx) = mpsc::channel::<Goal>(1);
    tokio::spawn(run_control(arm, cfg.clone(), goal_rx, busy.clone(), model, wiring));
    tokio::spawn(actions::run_move_arm_joints(joints_action, cfg.limits, goal_tx.clone(), busy.clone()));
    tokio::spawn(actions::run_move_arm(cartesian_action, goal_tx, busy));
    Ok(())
}

/// The single motor-owning control loop. Runs forever at `cfg.cycle_period`:
/// reads the measured state, computes the feedforward, and runs one [`Mode`]
/// tick, which commands the motors and yields the next mode. Starts by following
/// a producer that is already streaming (live from the power-on pose), else in
/// [`Mode::Startup`] easing to the ready pose; either way it never lunges on boot.
async fn run_control(
    arm: Arc<Mutex<ArmCan>>,
    cfg: ControlConfig,
    mut goals: mpsc::Receiver<Goal>,
    busy: Arc<AtomicBool>,
    mut model: srs_model::Arm,
    wiring: StreamWiring,
) {
    let mut pacer = Pacer::new(cfg.cycle_period);
    let (q0, _) = read_state(&arm, cfg.recv_timeout_us);
    let mut mode = if stream_present(&wiring.joint, &cfg) {
        // A producer is already streaming: follow it live from the power-on pose,
        // velocity-capped, so the arm converges to the operator (and keeps
        // following as they move) with no neutral excursion. Admit goals now.
        busy.store(false, Ordering::Release);
        info!("startup: producer already streaming, following from {}", fmt_joints(&q0));
        Mode::Follow(Follow::idle(q0))
    } else {
        Mode::Startup(StartupMove::new(q0, &cfg))
    };

    info!("control loop started (in-process gravity compensation + IK)");
    loop {
        let (q, qdot) = read_state(&arm, cfg.recv_timeout_us);
        let (ff_tau, ee_base, jacobian) = feedforward(&mut model, &q, &qdot);
        wiring.measured.send_replace(Some(crate::stream::MeasuredState {
            positions: q,
            velocities: qdot,
        }));
        let mut io = TickIo {
            arm: &arm,
            cfg: &cfg,
            model: &model,
            goals: &mut goals,
            busy: &busy,
            joint_stream: &wiring.joint,
            q,
            ff_tau,
            ee_base,
            jacobian,
            now: Instant::now(),
        };
        mode = mode.tick(&mut io).await;
        pacer.pace().await;
    }
}

/// Whether a producer is already streaming joint commands within the watchdog
/// window, so the loop follows it live from boot instead of homing to the ready
/// pose first.
fn stream_present(joint: &watch::Receiver<Option<JointCommand>>, cfg: &ControlConfig) -> bool {
    let now = Instant::now();
    joint.borrow().as_ref().is_some_and(|c| now.duration_since(c.recv_at) <= cfg.stream_timeout)
}

/// One tick of rigid-body feedforward: gravity and Coriolis from the posed chain
/// (which carries the distal gripper payload) plus locally computed friction, all
/// at full weight, so the PD term only corrects residual error. Poses the chain
/// once and also returns the EE pose (base frame) that the same evaluation yields
/// for free (used by Cartesian admission, the state streams, and move results)
/// plus the geometric Jacobian (used to cap the `Follow` chase by hand speed).
fn feedforward(model: &mut srs_model::Arm, q: &JointVec, qdot: &JointVec) -> (JointVec, Isometry3<f64>, Jacobian) {
    let posed = model.at(q);
    let gravity = posed.gravity_torques();
    let coriolis = posed.coriolis_torques(qdot);
    let ee_base = posed.ee_pose();
    let jacobian = posed.jacobian();
    let friction = friction::torques(&friction::V1, qdot);
    let ff_tau = std::array::from_fn(|i| gravity[i] + coriolis[i] + friction[i]);
    (ff_tau, ee_base, jacobian)
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
