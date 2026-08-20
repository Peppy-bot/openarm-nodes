//! Arm move-action admission: the `move_arm_joints` and `move_arm` handlers the
//! backbone exposes. Each validates the goal (arm_id, joint count, finiteness,
//! duration, and joint limits for joint moves) and claims the target arm's
//! single-flight slot, then hands the accepted goal to that arm's planner over
//! its goal channel. The planner runs the motion - governed against the other
//! arm - completes the goal, and releases the busy slot at the terminal.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use peppygen::exposed_actions::limb_motion::{move_arm, move_arm_joints};
use peppygen::{NodeRunner, Result};
use srs_model::Limit;
use tokio::sync::mpsc;
use tracing::error;

use crate::planner::{Goal, JointReply};
use crate::types::{ARM_DOF, JointVec, Side, pose_from_wire};

use crate::actions::claim;

fn target_in_limits(q: &JointVec, limits: &[Limit; ARM_DOF]) -> bool {
    q.iter().zip(limits).all(|(&v, l)| v >= l.lo && v <= l.hi)
}

/// Everything a joint goal must get right before it can claim an arm: a side
/// this backbone drives, exactly [`ARM_DOF`] finite joint positions, a finite
/// non-negative duration, and a target inside that arm's joint limits. Yields
/// the side index and the parsed target, or the reason the goal is refused.
/// Run once to decide the goal and again after acceptance (peppy's goal
/// decision is pre-context), so admission and dispatch cannot disagree.
fn parse_joint_goal(
    d: &move_arm_joints::GoalRequestData,
    limits: &[[Limit; ARM_DOF]; 2],
) -> std::result::Result<(usize, JointVec), String> {
    let Some(idx) = Side::from_arm_id(d.arm_id).map(Side::index) else {
        return Err("arm_id out of range".into());
    };
    let Ok(target) = JointVec::try_from(d.joint_positions.as_slice()) else {
        return Err(format!(
            "expected {ARM_DOF} joint positions, got {}",
            d.joint_positions.len()
        ));
    };
    if !target.iter().all(|v| v.is_finite()) {
        return Err("non-finite joint target".into());
    }
    if !(d.duration_s.is_finite() && d.duration_s >= 0.0) {
        return Err("invalid duration".into());
    }
    if !target_in_limits(&target, &limits[idx]) {
        return Err("target out of joint limits".into());
    }
    Ok((idx, target))
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
                let idx = match parse_joint_goal(&req.data, &limits) {
                    Ok((idx, _target)) => idx,
                    Err(reason) => return Ok(move_arm_joints::GoalDecision::reject(reason)),
                };
                if !claim(&busy[idx]) {
                    return Ok(move_arm_joints::GoalDecision::reject(
                        "arm is already executing a motion",
                    ));
                }
                Ok(move_arm_joints::GoalDecision::accept())
            })
            .await?;
        let Some(ctx) = accepted else { return Ok(()) };
        let (idx, target) =
            parse_joint_goal(&ctx.request().data, &limits).expect("validated on accept");
        let duration_s = ctx.request().data.duration_s;
        if goal_txs[idx]
            .send(Goal::Joint {
                target,
                duration_s,
                reply: JointReply::MoveArmJoints(Box::new(ctx)),
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
                    return Ok(move_arm::GoalDecision::reject("arm_id out of range"));
                };
                if let Err(reason) = pose_from_wire(d.position, d.orientation) {
                    return Ok(move_arm::GoalDecision::reject(format!(
                        "goal pose has {reason}"
                    )));
                }
                if !(d.duration_s.is_finite() && d.duration_s >= 0.0) {
                    return Ok(move_arm::GoalDecision::reject("invalid duration"));
                }
                if !claim(&busy[idx]) {
                    return Ok(move_arm::GoalDecision::reject(
                        "arm is already executing a motion",
                    ));
                }
                Ok(move_arm::GoalDecision::accept())
            })
            .await?;
        let Some(ctx) = accepted else { return Ok(()) };
        let idx = Side::from_arm_id(ctx.request().data.arm_id)
            .map(Side::index)
            .expect("validated on accept");
        let target = pose_from_wire(ctx.request().data.position, ctx.request().data.orientation)
            .expect("validated on accept");
        let duration_s = ctx.request().data.duration_s;
        if goal_txs[idx]
            .send(Goal::Cartesian {
                target,
                duration_s,
                ctx: Box::new(ctx),
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

#[cfg(test)]
mod tests {
    use super::*;

    fn limits() -> [[Limit; ARM_DOF]; 2] {
        std::array::from_fn(|_| std::array::from_fn(|_| Limit { lo: -1.0, hi: 1.0 }))
    }

    fn goal(
        arm_id: u8,
        joint_positions: Vec<f64>,
        duration_s: f64,
    ) -> move_arm_joints::GoalRequestData {
        move_arm_joints::GoalRequestData {
            arm_id,
            joint_positions,
            duration_s,
        }
    }

    #[test]
    fn a_well_formed_goal_parses_to_its_side_and_target() {
        let (idx, target) = parse_joint_goal(&goal(1, vec![0.5; ARM_DOF], 0.0), &limits()).unwrap();
        assert_eq!(idx, Side::Right.index());
        assert_eq!(target, [0.5; ARM_DOF]);
    }

    #[test]
    fn a_goal_with_the_wrong_joint_count_is_refused_naming_both_counts() {
        for count in [0, ARM_DOF - 1, ARM_DOF + 1] {
            let reason = parse_joint_goal(&goal(0, vec![0.0; count], 0.0), &limits()).unwrap_err();
            assert_eq!(
                reason,
                format!("expected {ARM_DOF} joint positions, got {count}")
            );
        }
    }

    #[test]
    fn an_arm_id_this_backbone_does_not_drive_is_refused() {
        let reason = parse_joint_goal(&goal(2, vec![0.0; ARM_DOF], 0.0), &limits()).unwrap_err();
        assert_eq!(reason, "arm_id out of range");
    }

    #[test]
    fn a_non_finite_target_is_refused() {
        let mut q = vec![0.0; ARM_DOF];
        q[2] = f64::NAN;
        let reason = parse_joint_goal(&goal(0, q, 0.0), &limits()).unwrap_err();
        assert_eq!(reason, "non-finite joint target");
    }

    #[test]
    fn a_negative_or_non_finite_duration_is_refused() {
        for duration_s in [-0.1, f64::NAN, f64::INFINITY] {
            let reason =
                parse_joint_goal(&goal(0, vec![0.0; ARM_DOF], duration_s), &limits()).unwrap_err();
            assert_eq!(reason, "invalid duration");
        }
    }

    #[test]
    fn a_target_outside_the_arms_limits_is_refused() {
        let mut q = vec![0.0; ARM_DOF];
        q[6] = 1.5;
        let reason = parse_joint_goal(&goal(0, q, 0.0), &limits()).unwrap_err();
        assert_eq!(reason, "target out of joint limits");
    }
}
