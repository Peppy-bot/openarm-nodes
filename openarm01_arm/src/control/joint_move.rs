//! The joint-space move state: tracks a quintic trajectory for one accepted
//! `move_arm_joints` goal.

use peppygen::exposed_actions::openarm01_arm::v1::move_arm_joints;
use tracing::{error, info, warn};

use super::feedback::Feedback;
use super::{Completion, Mode, TickIo, ZERO, command, fmt_joints};
use crate::JointVec;
use crate::trajectory::JointTrajectory;

use std::sync::atomic::Ordering;

pub(super) struct JointMove {
    trajectory: JointTrajectory,
    ctx: move_arm_joints::GoalContext,
    feedback: Feedback,
}

impl JointMove {
    pub(super) fn start(g: crate::actions::JointGoal, io: &TickIo<'_>) -> Self {
        info!("move_arm_joints: start={} target={}", fmt_joints(&io.q), fmt_joints(&g.target));
        Self {
            trajectory: JointTrajectory::new(io.q, g.target, io.cfg.max_joint_velocity_rad_s, g.duration_s),
            ctx: g.ctx,
            feedback: Feedback::new(g.feedback_period),
        }
    }

    /// Command the trajectory sample and publish feedback; complete into `Hold`
    /// at the current setpoint when the trajectory finishes, the caller cancels
    /// (freezing mid-motion), or the motion times out.
    pub(super) async fn tick(mut self, io: &mut TickIo<'_>) -> Mode {
        let elapsed = self.trajectory.motion_start.elapsed().as_secs_f64();
        let (q_des, dq_des) = self.trajectory.sample(io.now);
        // On cancel, freeze at the current setpoint (zero desired velocity)
        // instead of tracking on toward the target.
        let cancelled = self.ctx.is_cancelled();
        command(io, &q_des, if cancelled { &ZERO } else { &dq_des });
        if cancelled {
            return self.finish(io, Completion::Cancelled, q_des, elapsed).await;
        }

        if self.feedback.should_publish(io.now) {
            let result = self.ctx.publish_feedback(io.q, elapsed).await;
            if let Some(e) = self.feedback.first_failure(result) {
                warn!("move_arm_joints feedback publish failing, suppressing repeats: {e}");
            }
        }

        if self.trajectory.is_complete(io.now) {
            self.finish(io, Completion::Done { success: true, message: "trajectory complete" }, q_des, elapsed)
                .await
        } else if elapsed > io.cfg.motion_timeout.as_secs_f64() {
            self.finish(io, Completion::Done { success: false, message: "timeout" }, q_des, elapsed).await
        } else {
            Mode::JointMove(self)
        }
    }

    /// Complete the goal per `completion`, release the single-flight claim, and
    /// hold at `setpoint`, the last commanded configuration.
    async fn finish(self, io: &TickIo<'_>, completion: Completion, setpoint: JointVec, elapsed: f64) -> Mode {
        let result = match completion {
            Completion::Done { success, message } => {
                self.ctx.complete(success, message.into(), io.q, elapsed).await
            }
            Completion::Cancelled => {
                self.ctx.complete_cancelled(false, "goal cancelled".into(), io.q, elapsed).await
            }
        };
        if let Err(e) = result {
            error!("move_arm_joints complete: {e}");
        }
        io.busy.store(false, Ordering::Release);
        Mode::Hold { setpoint }
    }
}
