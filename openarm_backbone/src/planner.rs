//! Per-arm motion planner: the mode state machine that turns one arm's inputs
//! (the commander's joint stream, and accepted joint / Cartesian move goals) into a
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
//! with no jump. Follow's target is the commander command; a joint move's target
//! is the quintic sample; a Cartesian move's target is the IK of the pose sample.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use peppygen::exposed_actions::{move_arm, move_arm_joints};
use srs_model::nalgebra::{Isometry3, SVector};
use srs_model::{Arm, ArmAnglePolicy, Jacobian, Limit};
use tokio::sync::mpsc;
use tracing::{error, info};

use crate::chase::{chase_step, clamp_to_limits};
use crate::streams::JointCommand;
use crate::trajectory::{CartesianTrajectory, JointTrajectory, plan_cartesian_duration};
use crate::{ARM_DOF, JointVec, Side};

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
}

/// A goal accepted by an action handler and handed to the planner.
pub enum Goal {
    Joint {
        target: JointVec,
        duration_s: f64,
        ctx: move_arm_joints::GoalContext,
    },
    Cartesian {
        target: Isometry3<f64>,
        duration_s: f64,
        ctx: move_arm::GoalContext,
    },
}

/// Releases a single-flight busy flag on drop. Held for the lifetime of a move
/// (an arm's mode here, a gripper move in the coordinator), so a move can never
/// end (success, failure, cancel, or an unreachable plan) without freeing the
/// slot the action handler claimed: no terminal path can leak it.
pub(crate) struct BusyGuard(pub(crate) Arc<AtomicBool>);

impl Drop for BusyGuard {
    fn drop(&mut self) {
        self.0.store(false, Ordering::Release);
    }
}

enum Mode {
    /// Ambient: chase the commander stream, or hold when none is streaming.
    Follow,
    /// Tracking a quintic joint trajectory for an accepted move_arm_joints goal.
    JointMove(JointMove),
    /// Tracking a Cartesian pose trajectory for an accepted move_arm goal, solving
    /// IK each tick for the joint target.
    CartesianMove(CartesianMove),
}

struct JointMove {
    traj: JointTrajectory,
    ctx: move_arm_joints::GoalContext,
    _busy: BusyGuard,
}

struct CartesianMove {
    traj: CartesianTrajectory,
    ctx: move_arm::GoalContext,
    seed: JointVec,
    prev_q_des: JointVec,
    prev_sample_at: Instant,
    _busy: BusyGuard,
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
    side: Side,
    model: Arm,
    cfg: PlanConfig,
    mode: Mode,
    /// Last governed setpoint: the chase base and the value held when idle.
    setpoint: JointVec,
}

impl Planner {
    /// Start holding zero with no producer locked. The coordinator seeds the real
    /// held pose from the first measured state (via [`Planner::commit`]) before any
    /// setpoint is published, so this initial value is never streamed.
    pub fn new(side: Side, model: Arm, cfg: PlanConfig) -> Self {
        Self {
            side,
            model,
            cfg,
            mode: Mode::Follow,
            setpoint: [0.0; ARM_DOF],
        }
    }

    /// Adopt the governed setpoint the coordinator actually published, so the next
    /// tick chases from there (not from the ungoverned candidate).
    pub fn commit(&mut self, governed: JointVec) {
        self.setpoint = governed;
    }

    /// Seed the held setpoint from the arm's first measured pose, clamped into the
    /// joint limits. A power-up pose parked past a soft limit (e.g. the elbow below
    /// its one-sided lower bound, hard against the boundary singularity) would
    /// otherwise anchor the backbone off-limit while the arm clamps every command back to
    /// the limit, leaving the backbone's held setpoint disagreeing with the arm's actual
    /// pose. Clamping the seed keeps the two consistent from the first tick.
    pub fn seed_from_measured(&mut self, measured: JointVec) {
        self.setpoint = clamp_to_limits(&measured, &self.cfg.limits);
    }

    /// The last published setpoint, the coordinator's `prev` for the governor.
    pub fn setpoint(&self) -> JointVec {
        self.setpoint
    }

    /// Retune the end-effector speed cap at runtime (the commander's control).
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
        busy: &Arc<AtomicBool>,
        now: Instant,
    ) -> JointVec {
        let mut mode = std::mem::replace(&mut self.mode, Mode::Follow);
        // A move preempts Follow; while a move runs the action handler rejects new
        // goals as busy, so the channel only delivers a goal in Follow.
        if matches!(mode, Mode::Follow)
            && let Ok(goal) = goals.try_recv()
        {
            mode = self.start_goal(goal, busy.clone(), now).await;
        }

        let Advance {
            target,
            next_mode,
            is_follow,
        } = self.advance(mode, measured_q, &command, now).await;
        self.mode = next_mode;

        let dt = self.cfg.cycle_period.as_secs_f64();
        let stepped = chase_step(
            &self.setpoint,
            &target,
            &self.cfg.max_joint_velocity_rad_s,
            dt,
        );
        if is_follow {
            let jac: Jacobian = self.model.at(&measured_q).jacobian();
            cap_ee_speed(
                &self.setpoint,
                &stepped,
                &jac,
                self.cfg.max_ee_velocity_m_s,
                dt,
            )
        } else {
            clamp_to_limits(&stepped, &self.cfg.limits)
        }
    }

    /// Advance one mode and yield an [`Advance`] (target, next mode, is_follow).
    /// Owns `mode` (moved in), so `self.model` is free for FK/IK here.
    async fn advance(
        &mut self,
        mode: Mode,
        measured_q: JointVec,
        command: &Option<JointCommand>,
        now: Instant,
    ) -> Advance {
        match mode {
            Mode::Follow => {
                let target = follow_target(command, self.setpoint, &self.cfg);
                Advance {
                    target,
                    next_mode: Mode::Follow,
                    is_follow: true,
                }
            }
            Mode::JointMove(JointMove { traj, ctx, _busy }) => {
                let q_des = traj.sample(now);
                let cancelled = ctx.is_cancelled();
                if cancelled || traj.is_complete(now) {
                    let elapsed = now.duration_since(traj.motion_start).as_secs_f64();
                    // Success means the trajectory ran to completion (not cancelled).
                    // The result carries the measured pose, so the caller judges how
                    // close it landed; the governor may have held it short.
                    let (success, message) = if cancelled {
                        (false, "goal cancelled")
                    } else {
                        (true, "trajectory complete")
                    };
                    let result = if cancelled {
                        ctx.complete_cancelled(false, message.into(), measured_q, elapsed)
                            .await
                    } else {
                        ctx.complete(success, message.into(), measured_q, elapsed)
                            .await
                    };
                    if let Err(e) = result {
                        error!("{}: move_arm_joints complete: {e}", self.side.label());
                    }
                    // `_busy` drops here: the slot is released.
                    let target = if cancelled { self.setpoint } else { q_des };
                    Advance {
                        target,
                        next_mode: Mode::Follow,
                        is_follow: false,
                    }
                } else {
                    Advance {
                        target: q_des,
                        next_mode: Mode::JointMove(JointMove { traj, ctx, _busy }),
                        is_follow: false,
                    }
                }
            }
            Mode::CartesianMove(m) => self.advance_cartesian(m, measured_q, now).await,
        }
    }

    /// One Cartesian tick: sample the pose, solve IK (seeded), and complete on
    /// cancel, IK failure, a velocity-guard trip, or normal completion. Any
    /// terminal drops `m` (and with it the busy guard), releasing the slot.
    async fn advance_cartesian(
        &mut self,
        mut m: CartesianMove,
        measured_q: JointVec,
        now: Instant,
    ) -> Advance {
        let elapsed = now.duration_since(m.traj.motion_start).as_secs_f64();
        if m.ctx.is_cancelled() {
            self.finish_cartesian(&m.ctx, measured_q, false, "goal cancelled", elapsed, true)
                .await;
            return Advance {
                target: m.prev_q_des,
                next_mode: Mode::Follow,
                is_follow: false,
            };
        }
        let base_target = self.model.base_pose(&m.traj.sample(now));
        let Some(sol) = self
            .model
            .solve_ik(&base_target, ArmAnglePolicy::FromSeed, &m.seed)
        else {
            self.finish_cartesian(
                &m.ctx,
                measured_q,
                false,
                "IK failed mid-trajectory (unreachable / singular)",
                elapsed,
                false,
            )
            .await;
            return Advance {
                target: m.prev_q_des,
                next_mode: Mode::Follow,
                is_follow: false,
            };
        };
        let dt = now
            .duration_since(m.prev_sample_at)
            .as_secs_f64()
            .max(self.cfg.cycle_period.as_secs_f64() * 0.5);
        if exceeds_velocity_limits(
            &sol.q,
            &m.prev_q_des,
            &self.cfg.max_joint_velocity_rad_s,
            dt,
        ) {
            self.finish_cartesian(
                &m.ctx,
                measured_q,
                false,
                "joint velocity limit exceeded near singularity",
                elapsed,
                false,
            )
            .await;
            return Advance {
                target: m.prev_q_des,
                next_mode: Mode::Follow,
                is_follow: false,
            };
        }
        m.prev_q_des = sol.q;
        m.prev_sample_at = now;
        m.seed = sol.q;
        if m.traj.is_complete(now) {
            self.finish_cartesian(
                &m.ctx,
                measured_q,
                true,
                "cartesian move complete",
                elapsed,
                false,
            )
            .await;
            Advance {
                target: sol.q,
                next_mode: Mode::Follow,
                is_follow: false,
            }
        } else {
            Advance {
                target: sol.q,
                next_mode: Mode::CartesianMove(m),
                is_follow: false,
            }
        }
    }

    /// Start an accepted goal: a joint trajectory, or a planned Cartesian move
    /// (rejected here, completing the goal failed, if the path is unreachable).
    /// The Cartesian start pose is the FK of the held setpoint (the chase base),
    /// not the measured pose, so the first-tick velocity guard compares the IK of
    /// the same configuration the chase continues from and cannot false-trip when
    /// the governor held the arm off its measured pose just before admission.
    async fn start_goal(&mut self, goal: Goal, busy: Arc<AtomicBool>, now: Instant) -> Mode {
        let busy = BusyGuard(busy);
        match goal {
            Goal::Joint {
                target,
                duration_s,
                ctx,
            } => {
                info!("{}: move_arm_joints start", self.side.label());
                let traj = JointTrajectory::new(
                    self.setpoint,
                    target,
                    self.cfg.max_joint_velocity_rad_s,
                    duration_s,
                );
                Mode::JointMove(JointMove {
                    traj,
                    ctx,
                    _busy: busy,
                })
            }
            Goal::Cartesian {
                target,
                duration_s,
                ctx,
            } => {
                let ee_base = self.model.at(&self.setpoint).ee_pose();
                let start_world = self.model.world_pose(&ee_base);
                let plan = plan_cartesian_duration(
                    &self.model,
                    &start_world,
                    &target,
                    self.setpoint,
                    &self.cfg.max_joint_velocity_rad_s,
                    duration_s,
                );
                let Some(duration) = plan else {
                    let (pos, quat) = world_pose_arrays(&start_world);
                    if let Err(e) = ctx
                        .complete(
                            false,
                            "target path unreachable / no in-limit IK solution".into(),
                            pos,
                            quat,
                            0.0,
                        )
                        .await
                    {
                        error!("{}: move_arm complete: {e}", self.side.label());
                    }
                    // `busy` drops here: the slot is released even on this early exit.
                    return Mode::Follow;
                };
                info!(
                    "{}: move_arm start, duration={duration:.3}s",
                    self.side.label()
                );
                Mode::CartesianMove(CartesianMove {
                    traj: CartesianTrajectory::new(start_world, target, duration),
                    ctx,
                    seed: self.setpoint,
                    prev_q_des: self.setpoint,
                    prev_sample_at: now,
                    _busy: busy,
                })
            }
        }
    }

    /// Complete a Cartesian goal, reporting the measured world pose at exit.
    async fn finish_cartesian(
        &mut self,
        ctx: &move_arm::GoalContext,
        measured_q: JointVec,
        success: bool,
        message: &str,
        elapsed: f64,
        cancelled: bool,
    ) {
        let ee_base = self.model.at(&measured_q).ee_pose();
        let (pos, quat) = world_pose_arrays(&self.model.world_pose(&ee_base));
        let result = if cancelled {
            ctx.complete_cancelled(false, message.into(), pos, quat, elapsed)
                .await
        } else {
            ctx.complete(success, message.into(), pos, quat, elapsed)
                .await
        };
        if let Err(e) = result {
            error!("{}: move_arm complete: {e}", self.side.label());
        }
    }
}

/// Scale the chase step so the end-effector's linear speed stays under `max_ee`,
/// using the Jacobian at the measured configuration (mirrors the arm's Follow). A
/// step that does not move the hand passes unchanged.
fn cap_ee_speed(
    setpoint: &JointVec,
    stepped: &JointVec,
    jac: &Jacobian,
    max_ee: f64,
    dt: f64,
) -> JointVec {
    let delta: JointVec = std::array::from_fn(|i| stepped[i] - setpoint[i]);
    let twist = jac * SVector::<f64, ARM_DOF>::from_column_slice(&delta);
    let ee_speed = twist.fixed_rows::<3>(0).norm() / dt;
    let scale = if ee_speed.is_finite() && ee_speed > max_ee {
        max_ee / ee_speed
    } else {
        1.0
    };
    std::array::from_fn(|i| setpoint[i] + delta[i] * scale)
}

/// Resolve the Follow target: chase the commander command (clamped into the
/// joint limits), holding `held` when none has arrived. The command stream is
/// paired to one producer, so there is nothing to arbitrate; if the producer stops
/// the latest command simply persists and the arm holds it through the MIT loop, so
/// no freshness deadman is needed.
fn follow_target(command: &Option<JointCommand>, held: JointVec, cfg: &PlanConfig) -> JointVec {
    match command {
        Some(c) => clamp_to_limits(&c.positions, &cfg.limits),
        None => held,
    }
}

/// Decompose a world-frame pose into the wire `(position, quaternion)` arrays.
fn world_pose_arrays(pose: &Isometry3<f64>) -> ([f64; 3], [f64; 4]) {
    let t = pose.translation.vector;
    let r = pose.rotation;
    ([t.x, t.y, t.z], [r.i, r.j, r.k, r.w])
}

/// Whether stepping `q_prev -> q_new` over `dt` implies any joint velocity beyond
/// the guard margin times its limit.
fn exceeds_velocity_limits(
    q_new: &JointVec,
    q_prev: &JointVec,
    max_vel: &JointVec,
    dt: f64,
) -> bool {
    q_new
        .iter()
        .zip(q_prev)
        .zip(max_vel)
        .any(|((&n, &p), &v)| (n - p).abs() > v * dt * VELOCITY_GUARD_MARGIN)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_cfg() -> PlanConfig {
        PlanConfig {
            cycle_period: Duration::from_millis(10),
            max_joint_velocity_rad_s: [10.0; ARM_DOF],
            max_ee_velocity_m_s: 1.0,
            limits: [Limit {
                lo: -10.0,
                hi: 10.0,
            }; ARM_DOF],
        }
    }

    fn joint_cmd(positions: JointVec) -> JointCommand {
        JointCommand { positions }
    }

    #[test]
    fn seed_from_measured_clamps_a_below_limit_pose_to_the_joint_limits() {
        // Build the real arm: the elbow (j4, index 3) carries a one-sided lower bound
        // of ~0.05 (the singularity floor applied by crate::arm_model), hard against the
        // boundary singularity. A power-up pose with the elbow below it must seed at the
        // limit, not off it.
        let version = openarm_description::HardwareVersion::V1;
        let model = crate::arm_model(version, version.base_link(openarm_description::Side::Left))
            .expect("build left arm from bundled URDF");
        let limits = model.limits();
        let cfg = PlanConfig {
            limits,
            ..test_cfg()
        };
        let mut planner = Planner::new(Side::Left, model, cfg);

        let mut measured = [0.0; ARM_DOF];
        measured[3] = -0.2; // elbow below its lower limit
        planner.seed_from_measured(measured);

        let seeded = planner.setpoint();
        assert_eq!(
            seeded[3], limits[3].lo,
            "elbow seeds at its lower limit, off the singularity"
        );
        assert!(
            seeded[3] >= 0.04,
            "vendored URDF elbow lower limit is ~0.05"
        );
        assert_eq!(seeded[0], 0.0, "an in-range joint is untouched");
    }

    #[test]
    fn busy_guard_releases_slot_on_drop() {
        let busy = Arc::new(AtomicBool::new(true));
        {
            let _g = BusyGuard(busy.clone());
            assert!(busy.load(Ordering::Acquire));
        }
        assert!(
            !busy.load(Ordering::Acquire),
            "guard must free the slot on drop, so no move terminal can leak it"
        );
    }

    #[test]
    fn follow_tracks_the_command() {
        let target = follow_target(
            &Some(joint_cmd([0.2; ARM_DOF])),
            [0.9; ARM_DOF],
            &test_cfg(),
        );
        assert_eq!(target, [0.2; ARM_DOF]);
    }

    #[test]
    fn follow_holds_when_no_command() {
        let held = [0.3; ARM_DOF];
        assert_eq!(follow_target(&None, held, &test_cfg()), held);
    }

    #[test]
    fn a_consumed_command_holds_the_move_endpoint_until_a_newer_one() {
        // The move -> Follow handoff at the command-watch seam. An accepted move
        // clears the side's command watch (what the action handler does with
        // `send_replace(None)`), so Follow holds at the move's endpoint instead
        // of chasing the pre-move streamed target, until a command that arrives
        // after the clear. This locks the contract Follow relies on; that the
        // handler performs the clear is covered by the live regression.
        let (tx, rx) = tokio::sync::watch::channel(None);
        let endpoint = [0.2; ARM_DOF];

        // Streamed before the move: Follow would chase it.
        tx.send_replace(Some(joint_cmd([0.9; ARM_DOF])));
        assert_eq!(
            follow_target(&rx.borrow(), endpoint, &test_cfg()),
            [0.9; ARM_DOF],
            "a live streamed command is chased"
        );

        // The accepted move consumes it: Follow now holds at the move endpoint.
        tx.send_replace(None);
        assert_eq!(
            follow_target(&rx.borrow(), endpoint, &test_cfg()),
            endpoint,
            "a consumed command leaves Follow on the move endpoint, not the stale stream"
        );

        // A command that arrives after the move resumes following.
        tx.send_replace(Some(joint_cmd([0.4; ARM_DOF])));
        assert_eq!(
            follow_target(&rx.borrow(), endpoint, &test_cfg()),
            [0.4; ARM_DOF],
            "a command after the move resumes following"
        );
    }

    #[test]
    fn cap_ee_speed_throttles_a_hand_moving_step_to_the_cap() {
        // Joint 0 moves the hand 1 m per rad along x; the rest do not move it.
        let mut jac = Jacobian::zeros();
        jac[(0, 0)] = 1.0;
        let dt = 0.01;
        let max_ee = 0.25;
        let mut stepped = [0.0; ARM_DOF];
        stepped[0] = 0.1; // 0.1 rad over 0.01 s is 10 m/s of hand speed, over the cap.
        let next = cap_ee_speed(&[0.0; ARM_DOF], &stepped, &jac, max_ee, dt);
        assert!(
            (next[0] - max_ee * dt).abs() < 1e-12,
            "joint 0 not scaled to the cap"
        );
        assert_eq!(next[1..], [0.0; ARM_DOF - 1]);
    }

    #[test]
    fn cap_ee_speed_leaves_a_step_that_does_not_move_the_hand() {
        let jac = Jacobian::zeros();
        let mut stepped = [0.0; ARM_DOF];
        stepped[3] = 0.2;
        let next = cap_ee_speed(&[0.0; ARM_DOF], &stepped, &jac, 0.25, 0.01);
        assert_eq!(next, stepped);
    }

    #[test]
    fn exceeds_velocity_limits_at_the_guard_boundary() {
        let prev = [0.0; ARM_DOF];
        let vmax = [1.0; ARM_DOF];
        let dt = 0.1;
        // limit * dt * margin = 1.0 * 0.1 * 1.5 = 0.15 rad allowed this step.
        let mut under = [0.0; ARM_DOF];
        under[0] = 0.149;
        let mut over = [0.0; ARM_DOF];
        over[0] = 0.151;
        assert!(!exceeds_velocity_limits(&under, &prev, &vmax, dt));
        assert!(exceeds_velocity_limits(&over, &prev, &vmax, dt));
    }
}
