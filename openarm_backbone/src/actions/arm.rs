//! Arm move-action admission: the `move_arm_joints` and `move_arm` handlers the
//! backbone exposes to the commander. Each validates the goal (arm_id, finiteness,
//! duration, and joint limits for joint moves) and claims the target arm's
//! single-flight slot, then hands the accepted goal to that arm's planner over
//! its goal channel. The planner runs the motion - governed against the other
//! arm - completes the goal, and releases the busy slot at the terminal.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use peppygen::exposed_actions::{move_arm, move_arm_joints};
use peppygen::{NodeRunner, Result};
use srs_model::Limit;
use srs_model::nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion};
use tokio::sync::mpsc;
use tracing::error;

use crate::planner::Goal;
use crate::{ARM_DOF, JointVec, Side};

use crate::actions::claim;

fn accept_move_arm_joints() -> move_arm_joints::GoalDecision {
    move_arm_joints::GoalDecision::Accept(move_arm_joints::GoalResponse::new(true, None))
}

fn reject_move_arm_joints(reason: impl Into<String>) -> move_arm_joints::GoalDecision {
    move_arm_joints::GoalDecision::Reject(move_arm_joints::GoalResponse::new(
        false,
        Some(reason.into()),
    ))
}

fn accept_move_arm() -> move_arm::GoalDecision {
    move_arm::GoalDecision::Accept(move_arm::GoalResponse::new(true, None))
}

fn reject_move_arm(reason: impl Into<String>) -> move_arm::GoalDecision {
    move_arm::GoalDecision::Reject(move_arm::GoalResponse::new(false, Some(reason.into())))
}

fn target_in_limits(q: &JointVec, limits: &[Limit; ARM_DOF]) -> bool {
    q.iter().zip(limits).all(|(&v, l)| v >= l.lo && v <= l.hi)
}

/// Expose `move_arm_joints`: validate + claim, then hand the goal to the arm's
/// planner. The planner releases the busy slot when the move ends.
pub async fn run_move_arm_joints(
    runner: Arc<NodeRunner>,
    goal_txs: [mpsc::Sender<Goal>; 2],
    busy: [Arc<AtomicBool>; 2],
    limits: [[Limit; ARM_DOF]; 2],
) -> Result<()> {
    let mut handle = move_arm_joints::ActionHandle::expose(&runner).await?;
    loop {
        let accepted = handle
            .handle_goal_next_request(|req| {
                let d = &req.data;
                let Some(idx) = Side::from_arm_id(d.arm_id).map(Side::index) else {
                    return Ok(reject_move_arm_joints("arm_id out of range"));
                };
                if !d.joint_positions.iter().all(|v| v.is_finite()) {
                    return Ok(reject_move_arm_joints("non-finite joint target"));
                }
                if !(d.duration_s.is_finite() && d.duration_s >= 0.0) {
                    return Ok(reject_move_arm_joints("invalid duration"));
                }
                if !target_in_limits(&d.joint_positions, &limits[idx]) {
                    return Ok(reject_move_arm_joints("target out of joint limits"));
                }
                if !claim(&busy[idx]) {
                    return Ok(reject_move_arm_joints("arm is already executing a motion"));
                }
                Ok(accept_move_arm_joints())
            })
            .await?;
        let Some(ctx) = accepted else { return Ok(()) };
        let idx = Side::from_arm_id(ctx.request().data.arm_id)
            .map(Side::index)
            .expect("validated on accept");
        let target = ctx.request().data.joint_positions;
        let duration_s = ctx.request().data.duration_s;
        if goal_txs[idx]
            .send(Goal::Joint {
                target,
                duration_s,
                ctx,
            })
            .await
            .is_err()
        {
            busy[idx].store(false, Ordering::Release);
            error!("move_arm_joints: coordinator channel closed");
            return Ok(());
        }
    }
}

/// Expose `move_arm` (Cartesian): validate + claim, then hand the goal to the
/// arm's planner, which plans IK along the path and runs it governed.
pub async fn run_move_arm(
    runner: Arc<NodeRunner>,
    goal_txs: [mpsc::Sender<Goal>; 2],
    busy: [Arc<AtomicBool>; 2],
) -> Result<()> {
    let mut handle = move_arm::ActionHandle::expose(&runner).await?;
    loop {
        let accepted = handle
            .handle_goal_next_request(|req| {
                let d = &req.data;
                let Some(idx) = Side::from_arm_id(d.arm_id).map(Side::index) else {
                    return Ok(reject_move_arm("arm_id out of range"));
                };
                let finite = d
                    .position
                    .iter()
                    .chain(d.orientation.iter())
                    .all(|v| v.is_finite());
                if !finite {
                    return Ok(reject_move_arm("non-finite pose"));
                }
                let quat_norm = d.orientation.iter().map(|v| v * v).sum::<f64>().sqrt();
                if quat_norm < 1e-6 {
                    return Ok(reject_move_arm("degenerate orientation quaternion"));
                }
                if !(d.duration_s.is_finite() && d.duration_s >= 0.0) {
                    return Ok(reject_move_arm("invalid duration"));
                }
                if !claim(&busy[idx]) {
                    return Ok(reject_move_arm("arm is already executing a motion"));
                }
                Ok(accept_move_arm())
            })
            .await?;
        let Some(ctx) = accepted else { return Ok(()) };
        let idx = Side::from_arm_id(ctx.request().data.arm_id)
            .map(Side::index)
            .expect("validated on accept");
        let target = pose_from_arrays(ctx.request().data.position, ctx.request().data.orientation);
        let duration_s = ctx.request().data.duration_s;
        if goal_txs[idx]
            .send(Goal::Cartesian {
                target,
                duration_s,
                ctx,
            })
            .await
            .is_err()
        {
            busy[idx].store(false, Ordering::Release);
            error!("move_arm: coordinator channel closed");
            return Ok(());
        }
    }
}

/// Build a world-frame isometry from the wire arrays: position `[x, y, z]`
/// and quaternion `[x, y, z, w]` (normalized; validated non-degenerate above).
fn pose_from_arrays(p: [f64; 3], q: [f64; 4]) -> Isometry3<f64> {
    let rotation = UnitQuaternion::from_quaternion(Quaternion::new(q[3], q[0], q[1], q[2]));
    Isometry3::from_parts(Translation3::new(p[0], p[1], p[2]), rotation)
}
