//! Per-arm motion planner: the mode state machine that turns one arm's inputs
//! (the operator joint stream, and accepted joint / Cartesian move goals) into a
//! candidate joint setpoint each tick. It does NOT command anything and does not
//! know about the other arm: it produces a candidate, the coordinator governs
//! both arms' candidates against the collision model, and feeds the governed
//! result back via [`Planner::commit`] so the next tick chases from where the arm
//! was actually allowed to go.
//!
//! Every mode reduces to "chase a target": the setpoint advances toward the
//! target at the per-joint velocity limits (Follow also caps end-effector speed),
//! so streaming and moves stay smooth under throttling - when the governor holds
//! the setpoint, the chase simply catches up at the velocity limit once clear,
//! with no jump. Follow's target is the operator command; a joint move's target
//! is the quintic sample; a Cartesian move's target is the IK of the pose sample.

use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use peppygen::exposed_actions::{move_arm, move_arm_joints};
use peppylib::messaging::ProducerRef;
use srs_model::nalgebra::{Isometry3, SVector};
use srs_model::{Arm, ArmAnglePolicy, Jacobian, Limit};
use tokio::sync::mpsc;
use tracing::{error, info};

use crate::chase::{chase_step, clamp_to_limits};
use crate::streams::JointCommand;
use crate::trajectory::{CartesianTrajectory, JointTrajectory, plan_cartesian_duration};
use crate::{ARM_DOF, JointVec};

/// Slack the runtime per-tick Cartesian velocity check allows over the planned
/// limit before aborting (mirrors the arm's backstop over the up-front plan).
const VELOCITY_GUARD_MARGIN: f64 = 1.5;

/// Per-arm static configuration for the planner (the motion limits relocated
/// from the arm). Cloned per side.
#[derive(Clone)]
pub struct PlanConfig {
    pub cycle_period: Duration,
    pub max_joint_velocity_rad_s: JointVec,
    pub max_ee_velocity_m_s: f64,
    pub limits: [Limit; ARM_DOF],
    pub stream_timeout: Duration,
}

/// A goal accepted by an action handler and handed to the planner.
pub enum Goal {
    Joint { target: JointVec, duration_s: f64, ctx: move_arm_joints::GoalContext },
    Cartesian { target: Isometry3<f64>, duration_s: f64, ctx: move_arm::GoalContext },
}

/// The locked operator producer and the chase target it drives.
struct Lock {
    producer: ProducerRef,
    target: JointVec,
    last_seq: u64,
    last_fresh: Instant,
}

enum Mode {
    /// Ambient: chase the locked operator stream, or hold when none is streaming.
    Follow { lock: Option<Lock> },
    /// Tracking a quintic joint trajectory for an accepted move_arm_joints goal.
    JointMove { traj: JointTrajectory, ctx: move_arm_joints::GoalContext },
    /// Tracking a Cartesian pose trajectory for an accepted move_arm goal, solving
    /// IK each tick for the joint target.
    CartesianMove(CartesianMove),
}

struct CartesianMove {
    traj: CartesianTrajectory,
    ctx: move_arm::GoalContext,
    seed: JointVec,
    prev_q_des: JointVec,
    prev_sample_at: Instant,
}

/// One mode's advance result: the joint target this tick, the next mode (the
/// state transition), and whether the arm is in Follow (only then is the EE-speed
/// cap applied to the chase).
struct Advance {
    target: JointVec,
    next_mode: Mode,
    is_follow: bool,
}

pub struct Planner {
    side: &'static str,
    model: Arm,
    cfg: PlanConfig,
    mode: Mode,
    /// Last governed setpoint: the chase base and the value held when idle.
    setpoint: JointVec,
}

impl Planner {
    /// Start holding `start_q` (the arm's first measured pose) with no producer
    /// locked. The coordinator seeds `start_q` from the first `joint_states`.
    pub fn new(side: &'static str, model: Arm, cfg: PlanConfig, start_q: JointVec) -> Self {
        Self { side, model, cfg, mode: Mode::Follow { lock: None }, setpoint: start_q }
    }

    /// Adopt the governed setpoint the coordinator actually published, so the next
    /// tick chases from there (not from the ungoverned candidate).
    pub fn commit(&mut self, governed: JointVec) {
        self.setpoint = governed;
    }

    /// The last published setpoint, the coordinator's `prev` for the governor.
    pub fn setpoint(&self) -> JointVec {
        self.setpoint
    }

    /// Retune the end-effector speed cap at runtime (the operator's control).
    /// Ignores a non-positive or non-finite value, keeping the current cap.
    pub fn set_max_ee_velocity(&mut self, v: f64) {
        if v.is_finite() && v > 0.0 {
            self.cfg.max_ee_velocity_m_s = v;
        }
    }

    /// Produce this tick's candidate setpoint: admit a pending goal, advance the
    /// active mode to a joint target, then chase it under the velocity limits.
    pub async fn tick(
        &mut self,
        measured_q: JointVec,
        command: Option<JointCommand>,
        goals: &mut mpsc::Receiver<Goal>,
        busy: &AtomicBool,
        now: Instant,
    ) -> JointVec {
        let mut mode = std::mem::replace(&mut self.mode, Mode::Follow { lock: None });
        // A move preempts Follow; while a move runs the action handler rejects new
        // goals as busy, so the channel only delivers a goal in Follow.
        if matches!(mode, Mode::Follow { .. })
            && let Ok(goal) = goals.try_recv() {
                mode = self.start_goal(goal, measured_q, now).await;
            }

        let Advance { target, next_mode, is_follow } = self.advance(mode, measured_q, &command, busy, now).await;
        self.mode = next_mode;

        let dt = self.cfg.cycle_period.as_secs_f64();
        let stepped = chase_step(&self.setpoint, &target, &self.cfg.max_joint_velocity_rad_s, dt);
        let stepped = if is_follow { self.cap_ee_speed(measured_q, &stepped) } else { stepped };
        clamp_to_limits(&stepped, &self.cfg.limits)
    }

    /// Advance one mode and yield an [`Advance`] (target, next mode, is_follow).
    /// Owns `mode` (moved in), so `self.model` is free for FK/IK here.
    async fn advance(
        &mut self,
        mode: Mode,
        measured_q: JointVec,
        command: &Option<JointCommand>,
        busy: &AtomicBool,
        now: Instant,
    ) -> Advance {
        match mode {
            Mode::Follow { mut lock } => {
                let target = follow_target(&mut lock, command, self.setpoint, &self.cfg, now);
                Advance { target, next_mode: Mode::Follow { lock }, is_follow: true }
            }
            Mode::JointMove { traj, ctx } => {
                let q_des = traj.sample(now).positions;
                let cancelled = ctx.is_cancelled();
                if cancelled || traj.is_complete(now) {
                    let elapsed = now.duration_since(traj.motion_start).as_secs_f64();
                    let (success, message) =
                        if cancelled { (false, "goal cancelled") } else { (true, "trajectory complete") };
                    let result = if cancelled {
                        ctx.complete_cancelled(false, message.into(), measured_q, elapsed).await
                    } else {
                        ctx.complete(success, message.into(), measured_q, elapsed).await
                    };
                    if let Err(e) = result {
                        error!("{}: move_arm_joints complete: {e}", self.side);
                    }
                    busy.store(false, Ordering::Release);
                    let target = if cancelled { self.setpoint } else { q_des };
                    Advance { target, next_mode: Mode::Follow { lock: None }, is_follow: false }
                } else {
                    Advance { target: q_des, next_mode: Mode::JointMove { traj, ctx }, is_follow: false }
                }
            }
            Mode::CartesianMove(m) => self.advance_cartesian(m, measured_q, busy, now).await,
        }
    }

    /// One Cartesian tick: sample the pose, solve IK (seeded), and complete on
    /// cancel, IK failure, a velocity-guard trip, or normal completion.
    async fn advance_cartesian(&mut self, mut m: CartesianMove, measured_q: JointVec, busy: &AtomicBool, now: Instant) -> Advance {
        let elapsed = now.duration_since(m.traj.motion_start).as_secs_f64();
        if m.ctx.is_cancelled() {
            self.finish_cartesian(&m.ctx, measured_q, false, "goal cancelled", elapsed, true).await;
            busy.store(false, Ordering::Release);
            return Advance { target: m.prev_q_des, next_mode: Mode::Follow { lock: None }, is_follow: false };
        }
        let base_target = self.model.base_pose(&m.traj.sample(now));
        let Some(sol) = self.model.solve_ik(&base_target, ArmAnglePolicy::FromSeed, &m.seed) else {
            self.finish_cartesian(&m.ctx, measured_q, false, "IK failed mid-trajectory (unreachable / singular)", elapsed, false).await;
            busy.store(false, Ordering::Release);
            return Advance { target: m.prev_q_des, next_mode: Mode::Follow { lock: None }, is_follow: false };
        };
        let dt = now.duration_since(m.prev_sample_at).as_secs_f64().max(self.cfg.cycle_period.as_secs_f64() * 0.5);
        if exceeds_velocity_limits(&sol.q, &m.prev_q_des, &self.cfg.max_joint_velocity_rad_s, dt) {
            self.finish_cartesian(&m.ctx, measured_q, false, "joint velocity limit exceeded near singularity", elapsed, false).await;
            busy.store(false, Ordering::Release);
            return Advance { target: m.prev_q_des, next_mode: Mode::Follow { lock: None }, is_follow: false };
        }
        m.prev_q_des = sol.q;
        m.prev_sample_at = now;
        m.seed = sol.q;
        if m.traj.is_complete(now) {
            self.finish_cartesian(&m.ctx, measured_q, true, "cartesian move complete", elapsed, false).await;
            busy.store(false, Ordering::Release);
            Advance { target: sol.q, next_mode: Mode::Follow { lock: None }, is_follow: false }
        } else {
            Advance { target: sol.q, next_mode: Mode::CartesianMove(m), is_follow: false }
        }
    }

    /// Start an accepted goal: a joint trajectory, or a planned Cartesian move
    /// (rejected here, completing the goal failed, if the path is unreachable).
    async fn start_goal(&mut self, goal: Goal, measured_q: JointVec, now: Instant) -> Mode {
        match goal {
            Goal::Joint { target, duration_s, ctx } => {
                info!("{}: move_arm_joints start", self.side);
                let traj = JointTrajectory::new(self.setpoint, target, self.cfg.max_joint_velocity_rad_s, duration_s);
                Mode::JointMove { traj, ctx }
            }
            Goal::Cartesian { target, duration_s, ctx } => {
                let ee_base = self.model.at(&measured_q).ee_pose();
                let start_world = self.model.world_pose(&ee_base);
                let plan = plan_cartesian_duration(&self.model, &start_world, &target, self.setpoint, &self.cfg.max_joint_velocity_rad_s, duration_s);
                let Some(duration) = plan else {
                    let (pos, quat) = world_pose_arrays(&start_world);
                    if let Err(e) = ctx.complete(false, "target path unreachable / no in-limit IK solution".into(), pos, quat, 0.0).await {
                        error!("{}: move_arm complete: {e}", self.side);
                    }
                    return Mode::Follow { lock: None };
                };
                info!("{}: move_arm start, duration={duration:.3}s", self.side);
                Mode::CartesianMove(CartesianMove {
                    traj: CartesianTrajectory::new(start_world, target, duration),
                    ctx,
                    seed: self.setpoint,
                    prev_q_des: self.setpoint,
                    prev_sample_at: now,
                })
            }
        }
    }

    /// Complete a Cartesian goal, reporting the measured world pose at exit.
    async fn finish_cartesian(&mut self, ctx: &move_arm::GoalContext, measured_q: JointVec, success: bool, message: &str, elapsed: f64, cancelled: bool) {
        let ee_base = self.model.at(&measured_q).ee_pose();
        let (pos, quat) = world_pose_arrays(&self.model.world_pose(&ee_base));
        let result = if cancelled {
            ctx.complete_cancelled(false, message.into(), pos, quat, elapsed).await
        } else {
            ctx.complete(success, message.into(), pos, quat, elapsed).await
        };
        if let Err(e) = result {
            error!("{}: move_arm complete: {e}", self.side);
        }
    }

    /// Scale the chase step so the end-effector's linear speed stays under the cap,
    /// using the Jacobian at the measured configuration (mirrors the arm's Follow).
    fn cap_ee_speed(&mut self, measured_q: JointVec, stepped: &JointVec) -> JointVec {
        let jacobian: Jacobian = self.model.at(&measured_q).jacobian();
        let dt = self.cfg.cycle_period.as_secs_f64();
        let delta: JointVec = std::array::from_fn(|i| stepped[i] - self.setpoint[i]);
        let twist = jacobian * SVector::<f64, ARM_DOF>::from_column_slice(&delta);
        let ee_speed = twist.fixed_rows::<3>(0).norm() / dt;
        let scale = if ee_speed.is_finite() && ee_speed > self.cfg.max_ee_velocity_m_s {
            self.cfg.max_ee_velocity_m_s / ee_speed
        } else {
            1.0
        };
        std::array::from_fn(|i| self.setpoint[i] + delta[i] * scale)
    }
}

/// Resolve the Follow target: chase the locked operator command, acquiring or
/// releasing the producer lock by freshness, holding `held` when none is live.
fn follow_target(lock: &mut Option<Lock>, command: &Option<JointCommand>, held: JointVec, cfg: &PlanConfig, now: Instant) -> JointVec {
    let fresh = |c: &JointCommand| now.duration_since(c.recv_at) <= cfg.stream_timeout;
    match lock.as_mut() {
        Some(l) => {
            if let Some(c) = command
                && c.producer == l.producer && c.seq != l.last_seq {
                    l.target = clamp_to_limits(&c.positions, &cfg.limits);
                    l.last_seq = c.seq;
                    l.last_fresh = now;
                }
            if now.duration_since(l.last_fresh) > cfg.stream_timeout {
                *lock = None;
                held
            } else {
                l.target
            }
        }
        None => match command.as_ref().filter(|c| fresh(c)) {
            Some(c) => {
                let target = clamp_to_limits(&c.positions, &cfg.limits);
                *lock = Some(Lock { producer: c.producer.clone(), target, last_seq: c.seq, last_fresh: now });
                target
            }
            None => held,
        },
    }
}

/// Decompose a world-frame pose into the interface's `(position, quaternion)`.
fn world_pose_arrays(pose: &Isometry3<f64>) -> ([f64; 3], [f64; 4]) {
    let t = pose.translation.vector;
    let r = pose.rotation;
    ([t.x, t.y, t.z], [r.i, r.j, r.k, r.w])
}

/// Whether stepping `q_prev -> q_new` over `dt` implies any joint velocity beyond
/// the guard margin times its limit.
fn exceeds_velocity_limits(q_new: &JointVec, q_prev: &JointVec, max_vel: &JointVec, dt: f64) -> bool {
    q_new.iter().zip(q_prev).zip(max_vel).any(|((&n, &p), &v)| (n - p).abs() > v * dt * VELOCITY_GUARD_MARGIN)
}
