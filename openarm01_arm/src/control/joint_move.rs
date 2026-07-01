//! The joint-space move state: tracks a quintic trajectory for one accepted
//! `move_arm_joints` goal.

use peppygen::exposed_actions::openarm01_arm::v1::move_arm_joints;
use tracing::{error, info};

use super::follow::Follow;
use super::{Completion, Mode, TickIo, ZERO, command, fmt_joints};
use crate::JointVec;
use crate::trajectory::JointTrajectory;

use std::sync::atomic::Ordering;

pub(super) struct JointMove {
    trajectory: JointTrajectory,
    ctx: move_arm_joints::GoalContext,
}

impl JointMove {
    pub(super) fn start(g: crate::actions::JointMoveGoal, io: &TickIo<'_>) -> Self {
        info!(
            "move_arm_joints: start={} target={}",
            fmt_joints(&io.q),
            fmt_joints(&g.target)
        );
        Self {
            trajectory: JointTrajectory::new(
                io.q,
                g.target,
                io.cfg.max_joint_velocity_rad_s,
                g.duration_s,
            ),
            ctx: g.ctx,
        }
    }

    /// Command the trajectory sample; complete and return to `Follow` at the
    /// current setpoint when the trajectory finishes or the caller cancels
    /// (freezing mid-motion). The trajectory is open-loop and always completes;
    /// a caller that judges it too long observes progress on the always-on state
    /// streams and cancels.
    pub(super) async fn tick(self, io: &mut TickIo<'_>) -> Mode {
        let elapsed = self.trajectory.motion_start.elapsed().as_secs_f64();
        let (q_des, dq_des) = self.trajectory.sample(io.now);
        // On cancel, freeze at the current setpoint (zero desired velocity)
        // instead of tracking on toward the target.
        let cancelled = self.ctx.is_cancelled();
        command(io, &q_des, if cancelled { &ZERO } else { &dq_des });
        if cancelled {
            return self.finish(io, Completion::Cancelled, q_des, elapsed).await;
        }

        if self.trajectory.is_complete(io.now) {
            self.finish(
                io,
                Completion::Done {
                    success: true,
                    message: "trajectory complete",
                },
                q_des,
                elapsed,
            )
            .await
        } else {
            Mode::JointMove(self)
        }
    }

    /// Complete the goal per `completion`, release the single-flight claim, and
    /// return to `Follow` holding `setpoint`, the last commanded configuration.
    /// `success` reports that the trajectory was commanded to completion; the
    /// move is open-loop, so it does not assert the measured joints reached the
    /// target within a tolerance. The result carries the measured positions at
    /// exit (`io.q`) for the caller to judge goal achievement.
    async fn finish(
        self,
        io: &TickIo<'_>,
        completion: Completion,
        setpoint: JointVec,
        elapsed: f64,
    ) -> Mode {
        let result = match completion {
            Completion::Done { success, message } => {
                self.ctx
                    .complete(success, message.into(), io.q, elapsed)
                    .await
            }
            Completion::Cancelled => {
                self.ctx
                    .complete_cancelled(false, "goal cancelled".into(), io.q, elapsed)
                    .await
            }
        };
        if let Err(e) = result {
            error!("move_arm_joints complete: {e}");
        }
        io.busy.store(false, Ordering::Release);
        Mode::Follow(Follow::idle(setpoint))
    }
}
