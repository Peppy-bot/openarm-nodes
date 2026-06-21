//! Action admission: validates incoming goals, enforces single-flight, and hands
//! accepted goals to the single motor-owning control task over one channel. These
//! handlers never touch the motors and never read hardware, so their validation is
//! pure. Reachability of a Cartesian target depends on the live joint state, so it
//! is checked in the control task (which owns that state), not here.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use peppygen::exposed_actions::openarm01_arm::v1::{move_arm, move_arm_joints};
use srs_model::Limit;
use srs_model::nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion};
use tokio::sync::mpsc;
use tracing::error;

use crate::{ARM_DOF, JointVec};

/// An accepted move goal, handed from an action handler to the single control
/// task. One channel carries both move kinds so the control task selects on a
/// single point and the same `busy` flag serialises them.
pub enum Goal {
    JointMove(JointMoveGoal),
    CartesianMove(CartesianMoveGoal),
}

pub struct JointMoveGoal {
    pub target: JointVec,
    pub duration_s: f64,
    pub ctx: move_arm_joints::GoalContext,
}

pub struct CartesianMoveGoal {
    pub target: Isometry3<f64>,
    pub duration_s: f64,
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
) {
    loop {
        let claimed = AtomicBool::new(false);
        let ctx = match handle
            .handle_goal_next_request(|req| {
                let data = &req.data;
                // Reject targets outside this arm's joint limits (also rejects
                // NaN/inf, which Limit::contains treats as out of range).
                if !target_in_limits(&data.joint_positions, &limits) {
                    return Ok(move_arm_joints::GoalResponse::reject(
                        "target joint positions out of range",
                    ));
                }
                if let Err(reason) = check_duration(data.duration_s) {
                    return Ok(move_arm_joints::GoalResponse::reject(reason));
                }
                if claim(&busy, &claimed) {
                    Ok(move_arm_joints::GoalResponse::accept())
                } else {
                    Ok(move_arm_joints::GoalResponse::reject("arm is already executing a motion"))
                }
            })
            .await
        {
            Ok(Some(ctx)) => ctx,
            Ok(None) => break, // action closed (node shutting down)
            Err(e) => {
                error!("move_arm_joints goal: {e}");
                release_if_claimed(&busy, &claimed);
                continue;
            }
        };

        let req = &ctx.request().data;
        let goal = Goal::JointMove(JointMoveGoal {
            target: req.joint_positions,
            duration_s: req.duration_s,
            ctx,
        });
        send_or_release(&goals, goal, &busy).await;
    }
}

/// Serves `move_arm` on its pre-exposed action: validates the world-frame pose
/// (finite, non-degenerate quaternion) and the duration, admits one goal at a
/// time, and forwards it. Whether the pose is reachable is decided by the control
/// task, which holds the live joint state needed to seed the IK solve.
pub async fn run_move_arm(
    mut handle: move_arm::ActionHandle,
    goals: mpsc::Sender<Goal>,
    busy: Arc<AtomicBool>,
) {
    loop {
        let claimed = AtomicBool::new(false);
        let ctx = match handle
            .handle_goal_next_request(|req| {
                let data = &req.data;
                if parse_target_pose(&data.position, &data.orientation).is_none() {
                    return Ok(move_arm::GoalResponse::reject(
                        "invalid target pose (non-finite position or degenerate quaternion)",
                    ));
                }
                if let Err(reason) = check_duration(data.duration_s) {
                    return Ok(move_arm::GoalResponse::reject(reason));
                }
                if claim(&busy, &claimed) {
                    Ok(move_arm::GoalResponse::accept())
                } else {
                    Ok(move_arm::GoalResponse::reject("arm is already executing a motion"))
                }
            })
            .await
        {
            Ok(Some(ctx)) => ctx,
            Ok(None) => break,
            Err(e) => {
                error!("move_arm goal: {e}");
                release_if_claimed(&busy, &claimed);
                continue;
            }
        };

        let req = &ctx.request().data;
        let target = parse_target_pose(&req.position, &req.orientation)
            .expect("pose validated at admission");
        let goal = Goal::CartesianMove(CartesianMoveGoal {
            target,
            duration_s: req.duration_s,
            ctx,
        });
        send_or_release(&goals, goal, &busy).await;
    }
}

/// Try to take the single-flight slot shared by both move actions, recording the
/// outcome in `claimed` so the admission loop's error path releases exactly what
/// this iteration claimed. The control task clears the slot when the accepted
/// motion finishes. Returns true if the slot was free (the goal may be accepted).
fn claim(busy: &AtomicBool, claimed: &AtomicBool) -> bool {
    let acquired = busy
        .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
        .is_ok();
    claimed.store(acquired, Ordering::Relaxed);
    acquired
}

/// Release the slot only if this admission iteration claimed it. The generated
/// `handle_goal_next_request` can return `Err` after the decider accepted (the
/// accept reply can fail to serialise or send), which would otherwise strand
/// `busy` set with no `GoalContext` to ever clear it and wedge all admission. A
/// reject-reply failure during a live motion takes the same path, so a blind
/// release would steal the running move's claim; gating on `claimed` avoids both.
fn release_if_claimed(busy: &AtomicBool, claimed: &AtomicBool) {
    if claimed.load(Ordering::Relaxed) {
        busy.store(false, Ordering::Release);
    }
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

/// Reject a duration that is not a non-negative finite number, which the
/// trajectory timing and `Duration::from_secs_f64` require. How long a move may
/// take is the caller's concern, not the arm's: the caller observes progress on
/// the always-on state streams and cancels the action if the move runs longer than
/// it wants.
fn check_duration(duration_s: f64) -> Result<(), &'static str> {
    if duration_s.is_finite() && duration_s >= 0.0 {
        Ok(())
    } else {
        Err("duration_s must be finite and >= 0")
    }
}

/// True if every joint target lies within this arm's position limits. Non-finite
/// values (NaN/inf) fall outside any range, so they are rejected too.
fn target_in_limits(target: &JointVec, limits: &[Limit; ARM_DOF]) -> bool {
    limits.iter().zip(target).all(|(limit, &q)| limit.contains(q))
}

/// Build a world-frame pose from a `move_arm` goal's `(position, quaternion)`
/// arrays, or `None` if the position is non-finite or the quaternion is degenerate
/// (near-zero norm). The quaternion is `[x, y, z, w]` (matches the kinematics
/// interface) and is normalized.
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

#[cfg(test)]
mod tests {
    use super::*;

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
    fn check_duration_rejects_non_finite_and_negative() {
        assert!(check_duration(0.0).is_ok());
        assert!(check_duration(30.0).is_ok());
        assert!(check_duration(-0.1).is_err());
        assert!(check_duration(f64::NAN).is_err());
        assert!(check_duration(f64::INFINITY).is_err());
    }

    #[test]
    fn claim_takes_the_slot_once_and_records_it() {
        let busy = AtomicBool::new(false);
        let first = AtomicBool::new(false);
        assert!(claim(&busy, &first)); // slot was free
        assert!(first.load(Ordering::Relaxed));

        // Second claim fails and records the failure (so its error path won't
        // release the first claimant's slot).
        let second = AtomicBool::new(false);
        assert!(!claim(&busy, &second));
        assert!(!second.load(Ordering::Relaxed));

        // Only a recorded claim releases.
        release_if_claimed(&busy, &second);
        assert!(busy.load(Ordering::Acquire)); // still held
        release_if_claimed(&busy, &first);
        assert!(!busy.load(Ordering::Acquire)); // released
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
