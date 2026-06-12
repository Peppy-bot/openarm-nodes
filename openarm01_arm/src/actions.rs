//! Action admission: validates incoming goals, enforces single-flight, and hands
//! accepted goals to the single motor-owning control task over one channel. These
//! handlers never touch the motors and never read hardware, so their validation is
//! pure. Reachability of a Cartesian target depends on the live joint state, so it
//! is checked in the control task (which owns that state), not here.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use peppygen::exposed_actions::openarm01_arm::v1::{move_arm, move_arm_joints};
use peppylib::runtime::CancellationToken;
use srs_model::nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion};
use tokio::sync::mpsc;
use tracing::error;

use crate::{ARM_DOF, JointVec};
use srs_model::Limit;

/// An accepted goal, handed from an action handler to the single control task.
/// One channel carries both motion kinds so the control task selects on a single
/// point and the same `busy` flag serialises every move.
pub enum Goal {
    Joints(JointGoal),
    Cartesian(CartesianGoal),
}

pub struct JointGoal {
    pub target: JointVec,
    pub duration_s: f64,
    pub feedback_period: Duration,
    pub ctx: move_arm_joints::GoalContext,
}

pub struct CartesianGoal {
    pub target: Isometry3<f64>,
    pub duration_s: f64,
    pub feedback_period: Duration,
    pub ctx: move_arm::GoalContext,
}

/// Serves `move_arm_joints` on its pre-exposed action: validates the joint target
/// against this arm's limits, admits one goal at a time (`busy`), and forwards it
/// to the control task.
pub async fn run_move_arm_joints(
    mut handle: move_arm_joints::ActionHandle,
    limits: [Limit; ARM_DOF],
    goals: mpsc::Sender<Goal>,
    busy: Arc<AtomicBool>,
    token: CancellationToken,
) {
    loop {
        let accept = handle.handle_goal_next_request(|req| {
            let data = &req.data;
            // Reject targets outside this arm's joint limits (also rejects
            // NaN/inf, which Limit::contains treats as out of range).
            if !target_in_limits(&data.joint_positions, &limits) {
                return Ok(move_arm_joints::GoalResponse::reject(
                    "target joint positions out of range",
                ));
            }
            if !(data.duration_s.is_finite() && data.duration_s >= 0.0) {
                return Ok(move_arm_joints::GoalResponse::reject(
                    "duration_s must be finite and >= 0",
                ));
            }
            if claim(&busy) {
                Ok(move_arm_joints::GoalResponse::accept())
            } else {
                Ok(move_arm_joints::GoalResponse::reject("arm is already executing a motion"))
            }
        });
        let ctx = tokio::select! {
            _ = token.cancelled() => break, // node shutting down
            res = accept => match res {
                Ok(Some(ctx)) => ctx,
                Ok(None) => break, // action closed (node shutting down)
                Err(e) => {
                    error!("move_arm_joints goal: {e}");
                    continue;
                }
            },
        };

        let req = &ctx.request().data;
        let goal = Goal::Joints(JointGoal {
            target: req.joint_positions,
            duration_s: req.duration_s,
            feedback_period: feedback_period(req.feedback_frequency),
            ctx,
        });
        send_or_release(&goals, goal, &busy).await;
    }
}

/// Serves `move_arm` on its pre-exposed action: validates the world-frame pose
/// (finite, non-degenerate quaternion) and duration, admits one goal at a time,
/// and forwards it. Whether the pose is reachable is decided by the control task,
/// which holds the live joint state needed to seed the IK solve.
pub async fn run_move_arm(
    mut handle: move_arm::ActionHandle,
    goals: mpsc::Sender<Goal>,
    busy: Arc<AtomicBool>,
    token: CancellationToken,
) {
    loop {
        let accept = handle.handle_goal_next_request(|req| {
            let data = &req.data;
            if parse_target_pose(&data.position, &data.orientation).is_none() {
                return Ok(move_arm::GoalResponse::reject(
                    "invalid target pose (non-finite position or degenerate quaternion)",
                ));
            }
            if !(data.duration_s.is_finite() && data.duration_s >= 0.0) {
                return Ok(move_arm::GoalResponse::reject(
                    "duration_s must be finite and >= 0",
                ));
            }
            if claim(&busy) {
                Ok(move_arm::GoalResponse::accept())
            } else {
                Ok(move_arm::GoalResponse::reject("arm is already executing a motion"))
            }
        });
        let ctx = tokio::select! {
            _ = token.cancelled() => break, // node shutting down
            res = accept => match res {
                Ok(Some(ctx)) => ctx,
                Ok(None) => break, // action closed (node shutting down)
                Err(e) => {
                    error!("move_arm goal: {e}");
                    continue;
                }
            },
        };

        let req = &ctx.request().data;
        let target = parse_target_pose(&req.position, &req.orientation)
            .expect("pose validated at admission");
        let goal = Goal::Cartesian(CartesianGoal {
            target,
            duration_s: req.duration_s,
            feedback_period: feedback_period(req.feedback_frequency),
            ctx,
        });
        send_or_release(&goals, goal, &busy).await;
    }
}

/// Atomically claim the single-flight slot shared by both actions; the control
/// task clears it when the motion finishes. Returns true if the slot was free (the
/// goal may be accepted), false if a motion is already running.
fn claim(busy: &AtomicBool) -> bool {
    busy.compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
        .is_ok()
}

/// Forward an accepted goal to the control task, releasing the single-flight slot
/// if the control task is gone (node shutting down). The goal's `ctx` drops with
/// it, which the engine reports to the client as abandoned.
async fn send_or_release(goals: &mpsc::Sender<Goal>, goal: Goal, busy: &AtomicBool) {
    if let Err(err) = goals.send(goal).await {
        error!("control task unavailable, dropping goal: {err}");
        busy.store(false, Ordering::Release);
    }
}

/// Build a world-frame pose from the request arrays, or `None` if the position is
/// non-finite or the quaternion is degenerate (near-zero norm). The quaternion is
/// `[x, y, z, w]` (matches the kinematics interface) and is normalized.
fn parse_target_pose(position: &[f64; 3], orientation: &[f64; 4]) -> Option<Isometry3<f64>> {
    if position.iter().chain(orientation).any(|v| !v.is_finite()) {
        return None;
    }
    let [x, y, z, w] = *orientation;
    let quat = Quaternion::new(w, x, y, z);
    if quat.norm() < 1e-6 {
        return None; // degenerate quaternion: orientation undefined
    }
    let rotation = UnitQuaternion::from_quaternion(quat);
    let translation = Translation3::new(position[0], position[1], position[2]);
    Some(Isometry3::from_parts(translation, rotation))
}

/// True if every joint target lies within this arm's position limits. Non-finite
/// values (NaN/inf) fall outside any range, so they are rejected too.
fn target_in_limits(target: &JointVec, limits: &[Limit; ARM_DOF]) -> bool {
    limits.iter().zip(target).all(|(limit, &q)| limit.contains(q))
}

/// Convert a feedback frequency in Hz to a Duration. Floors at 1 Hz to avoid divide-by-zero.
fn feedback_period(freq_hz: u32) -> Duration {
    Duration::from_micros(1_000_000 / freq_hz.max(1) as u64)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn feedback_period_floors_zero_freq() {
        assert_eq!(feedback_period(0), Duration::from_secs(1));
    }

    #[test]
    fn target_in_limits_accepts_home_and_rejects_out_of_range() {
        // Synthetic window per joint (the real limits come from the URDF at
        // runtime); j4 is one-sided like the elbow, lower bound at 0.
        let mut limits = [Limit { lo: -1.0, hi: 1.0 }; ARM_DOF];
        limits[3] = Limit { lo: 0.0, hi: 2.0 };

        // Home pose (all zeros) is inside every joint limit.
        assert!(target_in_limits(&[0.0; ARM_DOF], &limits));

        // A single joint past its upper bound fails the whole target.
        let mut over = [0.0; ARM_DOF];
        over[3] = limits[3].hi + 0.1;
        assert!(!target_in_limits(&over, &limits));

        // Non-finite values are rejected (Limit::contains is false for NaN/inf).
        let mut nan = [0.0; ARM_DOF];
        nan[0] = f64::NAN;
        assert!(!target_in_limits(&nan, &limits));
        let mut inf = [0.0; ARM_DOF];
        inf[0] = f64::INFINITY;
        assert!(!target_in_limits(&inf, &limits));
    }

    #[test]
    fn parse_target_pose_normalizes_and_rejects_degenerate() {
        // A non-unit quaternion is accepted and normalized.
        let pose = parse_target_pose(&[0.1, 0.2, 0.3], &[0.0, 0.0, 0.0, 2.0]).expect("valid");
        assert!((pose.rotation.norm() - 1.0).abs() < 1e-12);
        assert!((pose.translation.vector - srs_model::nalgebra::Vector3::new(0.1, 0.2, 0.3)).norm() < 1e-12);

        // Zero quaternion is degenerate.
        assert!(parse_target_pose(&[0.0; 3], &[0.0, 0.0, 0.0, 0.0]).is_none());

        // Non-finite position or orientation is rejected.
        assert!(parse_target_pose(&[f64::NAN, 0.0, 0.0], &[0.0, 0.0, 0.0, 1.0]).is_none());
        assert!(parse_target_pose(&[0.0; 3], &[f64::INFINITY, 0.0, 0.0, 1.0]).is_none());
    }
}
