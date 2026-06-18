//! The Cartesian move state: tracks a world-frame pose trajectory for one
//! accepted `move_arm` goal, solving IK in-process each tick.

use std::sync::atomic::Ordering;
use std::time::Instant;

use peppygen::exposed_actions::openarm01_arm::v1::move_arm;
use srs_model::ArmAnglePolicy;
use tracing::{error, info};

use super::follow::Follow;
use super::{Completion, Mode, TickIo, ZERO, command, fmt_joints, world_pose_arrays};
use crate::JointVec;
use crate::actions::CartesianMoveGoal;
use crate::trajectory::{CartesianTrajectory, plan_cartesian_duration};

/// Slack the runtime per-tick velocity check allows over the planned limit before
/// aborting. The planner already sizes the nominal move to the velocity limits, so
/// this backstop trips only on gross excursions the coarse plan missed (an IK
/// branch flip, or a spike between plan samples).
const VELOCITY_GUARD_MARGIN: f64 = 1.5;

pub(super) struct CartesianMove {
    trajectory: CartesianTrajectory,
    ctx: move_arm::GoalContext,
    /// Last solved configuration, seeding the next IK solve for branch/arm-angle
    /// continuity along the path.
    seed: JointVec,
    /// Previous tick's commanded configuration: the finite-difference base for the
    /// velocity feedforward and the per-tick joint-velocity safety check, and the
    /// freeze point if the motion ends early.
    prev_q_des: JointVec,
    /// When `prev_q_des` was commanded. The finite differences above divide by the
    /// real elapsed time, not the nominal cycle period, so a loop overrun (which
    /// stretches the trajectory step taken that tick) does not masquerade as a
    /// joint-velocity violation.
    prev_sample_at: Instant,
}

impl CartesianMove {
    /// Plan and start a Cartesian move: solve IK along the whole path up front
    /// (rejecting an unreachable one) and size the duration so the IK'd joints
    /// stay within their velocity limits, floored at the caller's request. On an
    /// unreachable path the goal is completed (failed) here and `None` is
    /// returned, releasing the single-flight claim.
    pub(super) async fn start(g: CartesianMoveGoal, io: &TickIo<'_>) -> Option<Self> {
        let start_world = io.model.world_pose(&io.ee_base);
        let Some(duration) = plan_cartesian_duration(
            io.model,
            &start_world,
            &g.target,
            io.q,
            &io.cfg.max_joint_velocity_rad_s,
            g.duration_s,
        ) else {
            let (pos, quat) = world_pose_arrays(&start_world);
            if let Err(e) = g
                .ctx
                .complete(false, "target path unreachable / no in-limit IK solution".into(), pos, quat, 0.0)
                .await
            {
                error!("move_arm complete: {e}");
            }
            io.busy.store(false, Ordering::Release);
            return None;
        };
        info!(
            "move_arm: start_q={} target_pos={:?} duration={:.3}s",
            fmt_joints(&io.q),
            g.target.translation.vector,
            duration,
        );
        Some(Self {
            trajectory: CartesianTrajectory::new(start_world, g.target, duration),
            ctx: g.ctx,
            seed: io.q,
            prev_q_des: io.q,
            prev_sample_at: Instant::now(),
        })
    }

    /// Sample the pose, solve IK for the joint setpoint (seeded for continuity),
    /// and command it with finite-difference velocity feedforward. Completes and
    /// returns to `Follow` at the last commanded configuration when the trajectory
    /// finishes, the caller cancels, or it aborts (IK failure or a joint step
    /// grossly exceeding its velocity limit, the runtime backstop over the
    /// up-front plan). A caller that judges the move too long observes progress on
    /// the always-on state streams and cancels.
    pub(super) async fn tick(mut self, io: &mut TickIo<'_>) -> Mode {
        let elapsed = self.trajectory.motion_start.elapsed().as_secs_f64();
        if self.ctx.is_cancelled() {
            // Stop where the path was last commanded rather than finishing the move.
            command(io, &self.prev_q_des, &ZERO);
            return self.finish(io, Completion::Cancelled, elapsed).await;
        }

        let base_target = io.model.base_pose(&self.trajectory.sample(io.now));
        let Some(sol) = io.model.solve_ik(&base_target, ArmAnglePolicy::FromSeed, &self.seed) else {
            // IK failed mid-path (unreachable / singular): hold the last good config.
            command(io, &self.prev_q_des, &ZERO);
            let done = Completion::Done {
                success: false,
                message: "IK failed mid-trajectory (unreachable / singular)",
            };
            return self.finish(io, done, elapsed).await;
        };

        // Real time since the previous sample, not the nominal period: an overrun
        // tick legitimately takes a larger trajectory step, and the guard and
        // feedforward must judge it against the time it actually had. Floored at
        // half the nominal period (paced samples cannot legitimately land closer)
        // so a degenerate near-zero interval cannot explode the finite differences.
        let dt = io
            .now
            .duration_since(self.prev_sample_at)
            .as_secs_f64()
            .max(io.cfg.cycle_period.as_secs_f64() * 0.5);
        if exceeds_velocity_limits(&sol.q, &self.prev_q_des, &io.cfg.max_joint_velocity_rad_s, dt) {
            command(io, &self.prev_q_des, &ZERO);
            let done = Completion::Done {
                success: false,
                message: "joint velocity limit exceeded near singularity",
            };
            return self.finish(io, done, elapsed).await;
        }

        // Velocity feedforward from the IK solution stream; the FromSeed continuity
        // guarantee keeps this difference small.
        let dq_des: JointVec = std::array::from_fn(|i| (sol.q[i] - self.prev_q_des[i]) / dt);
        command(io, &sol.q, &dq_des);
        self.prev_q_des = sol.q;
        self.prev_sample_at = io.now;
        self.seed = sol.q;

        if self.trajectory.is_complete(io.now) {
            self.finish(io, Completion::Done { success: true, message: "cartesian move complete" }, elapsed)
                .await
        } else {
            Mode::CartesianMove(self)
        }
    }

    /// Complete the goal per `completion`, release the single-flight claim, and
    /// return to `Follow` holding the last commanded configuration. `success`
    /// reports that the planned trajectory ran without an internal abort (IK
    /// failure or a velocity-guard trip), not that the measured pose reached the
    /// target within a tolerance. The result carries the measured world pose at
    /// exit for the caller to judge goal achievement.
    async fn finish(self, io: &TickIo<'_>, completion: Completion, elapsed: f64) -> Mode {
        let (pos, quat) = world_pose_arrays(&io.model.world_pose(&io.ee_base));
        let result = match completion {
            Completion::Done { success, message } => {
                self.ctx.complete(success, message.into(), pos, quat, elapsed).await
            }
            Completion::Cancelled => {
                self.ctx.complete_cancelled(false, "goal cancelled".into(), pos, quat, elapsed).await
            }
        };
        if let Err(e) = result {
            error!("move_arm complete: {e}");
        }
        io.busy.store(false, Ordering::Release);
        Mode::Follow(Follow::idle(self.prev_q_des))
    }
}

/// Whether stepping from `q_prev` to `q_new` over `dt_s` implies any joint
/// velocity beyond [`VELOCITY_GUARD_MARGIN`] times its limit.
fn exceeds_velocity_limits(q_new: &JointVec, q_prev: &JointVec, max_vel_rad_s: &JointVec, dt_s: f64) -> bool {
    q_new
        .iter()
        .zip(q_prev)
        .zip(max_vel_rad_s)
        .any(|((&new, &prev), &vmax)| (new - prev).abs() > vmax * dt_s * VELOCITY_GUARD_MARGIN)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ARM_DOF;

    const VMAX: JointVec = [2.0; ARM_DOF];
    const DT: f64 = 0.01;

    #[test]
    fn within_margin_passes() {
        let prev = [0.0; ARM_DOF];
        // At the limit itself: vmax * dt, well inside the 1.5x margin.
        let new = [2.0 * DT; ARM_DOF];
        assert!(!exceeds_velocity_limits(&new, &prev, &VMAX, DT));
    }

    #[test]
    fn one_joint_over_margin_trips() {
        let prev = [0.0; ARM_DOF];
        let mut new = [0.0; ARM_DOF];
        new[3] = 2.0 * DT * VELOCITY_GUARD_MARGIN * 1.01; // just past margin on J4 only
        assert!(exceeds_velocity_limits(&new, &prev, &VMAX, DT));
    }

    #[test]
    fn boundary_is_not_a_trip() {
        // The check is strictly greater-than: exactly margin * vmax * dt passes.
        let prev = [0.0; ARM_DOF];
        let new = [2.0 * DT * VELOCITY_GUARD_MARGIN; ARM_DOF];
        assert!(!exceeds_velocity_limits(&new, &prev, &VMAX, DT));
    }

    #[test]
    fn longer_interval_allows_larger_steps() {
        let prev = [0.0; ARM_DOF];
        let new = [2.0 * 2.0 * DT; ARM_DOF]; // double step
        assert!(exceeds_velocity_limits(&new, &prev, &VMAX, DT));
        assert!(!exceeds_velocity_limits(&new, &prev, &VMAX, 2.0 * DT)); // overrun tick: same step, more time
    }

    #[test]
    fn direction_does_not_matter() {
        let prev = [0.5; ARM_DOF];
        let mut new = prev;
        new[0] = prev[0] - 2.0 * DT * VELOCITY_GUARD_MARGIN * 1.01;
        assert!(exceeds_velocity_limits(&new, &prev, &VMAX, DT));
    }
}
