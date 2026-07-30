//! move_to_ready admission (the ready_posture contract): claim both arms,
//! hand each planner an ordinary joint goal to its Ready posture, and
//! complete the one action goal from both terminals. Cancel flips a shared
//! flag the moves poll, so each arm stops the way a cancelled joint move
//! stops.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use peppygen::exposed_actions::ready::move_to_ready;
use peppygen::{NodeRunner, Result};
use tokio::sync::mpsc;
use tracing::error;

use crate::actions::claim;
use crate::planner::{Goal, JointReply, ReadyOutcome, ReadyReply};
use crate::{JointVec, Side};

/// The right-arm Ready posture; the left is its j1..j3 mirror. The
/// commander's gesture library anchors on the same configuration. In-limit
/// for both hardware generations, pinned by test rather than clamped.
const READY_R: JointVec = [0.15, 0.40, -0.48, 0.95, 0.0, 0.0, 0.0];

/// Mirror a right-arm posture onto the left arm: j1..j3 flip sign.
const fn mirror(q: JointVec) -> JointVec {
    [-q[0], -q[1], -q[2], q[3], q[4], q[5], q[6]]
}

fn ready_posture(side: Side) -> JointVec {
    match side {
        Side::Left => mirror(READY_R),
        Side::Right => READY_R,
    }
}

/// Claim both arms' single-flight slots, or name the busy arm. A failure on
/// the second claim unwinds the first, so a refusal never leaves a slot held.
fn claim_both(busy: &[Arc<AtomicBool>; 2]) -> std::result::Result<(), &'static str> {
    if !claim(&busy[Side::Left.index()]) {
        return Err("the left arm is already executing a motion");
    }
    if !claim(&busy[Side::Right.index()]) {
        busy[Side::Left.index()].store(false, Ordering::Release);
        return Err("the right arm is already executing a motion");
    }
    Ok(())
}

/// How the whole-robot goal completes.
#[derive(Debug, PartialEq, Eq)]
enum Terminal {
    Success,
    Failed,
    Cancelled,
}

/// Judge the end of a ready move from what actually came back: the goals
/// dispatched, the outcomes received, and whether a cancel was seen. Cancel
/// wins whatever the outcomes say; otherwise success requires both arms
/// dispatched and both outcomes successful, and the message names the first
/// thing that went wrong.
fn summarize(pending: usize, outcomes: &[ReadyOutcome], cancelled: bool) -> (Terminal, String) {
    if cancelled {
        return (Terminal::Cancelled, "goal cancelled".to_string());
    }
    if pending < 2 {
        return (
            Terminal::Failed,
            "an arm's planner is unavailable".to_string(),
        );
    }
    if outcomes.len() < pending {
        return (Terminal::Failed, "a planner dropped the move".to_string());
    }
    match outcomes.iter().find(|o| !o.success) {
        Some(failed) => (Terminal::Failed, failed.message.clone()),
        None => (Terminal::Success, "both arms at ready".to_string()),
    }
}

/// Expose `move_to_ready`: claim both arms, run both joint moves, complete
/// the goal once both report. One goal at a time; a goal arriving mid-move
/// waits unread until this one completes, then claims the freed arms.
pub async fn run_move_to_ready(
    runner: Arc<NodeRunner>,
    goal_txs: [mpsc::Sender<Goal>; 2],
    busy: [Arc<AtomicBool>; 2],
) -> Result<()> {
    let mut handle = move_to_ready::ActionHandle::expose(&runner).await?;
    loop {
        let accepted = handle
            .handle_goal_next_request(|req| {
                let duration_s = req.data.duration_s;
                if !(duration_s.is_finite() && duration_s >= 0.0) {
                    return Ok(move_to_ready::GoalDecision::reject("invalid duration"));
                }
                if let Err(reason) = claim_both(&busy) {
                    return Ok(move_to_ready::GoalDecision::reject(reason));
                }
                Ok(move_to_ready::GoalDecision::accept())
            })
            .await?;
        let Some(ctx) = accepted else { return Ok(()) };
        let duration_s = ctx.request().data.duration_s;

        let cancelled = Arc::new(AtomicBool::new(false));
        let (done_tx, mut done_rx) = mpsc::channel::<ReadyOutcome>(2);
        let mut pending = 0usize;
        for side in [Side::Left, Side::Right] {
            let idx = side.index();
            let goal = Goal::Joint {
                target: ready_posture(side),
                duration_s,
                reply: JointReply::Ready(ReadyReply {
                    done_tx: done_tx.clone(),
                    cancelled: cancelled.clone(),
                }),
            };
            if goal_txs[idx].send(goal).await.is_err() {
                // The planner is gone; release the claim its goal would have.
                busy[idx].store(false, Ordering::Release);
                error!("move_to_ready: {} goal channel closed", side.label());
            } else {
                pending += 1;
            }
        }
        drop(done_tx);

        let mut outcomes: Vec<ReadyOutcome> = Vec::with_capacity(pending);
        let mut cancel_seen = false;
        while outcomes.len() < pending {
            tokio::select! {
                _ = ctx.cancel_signal(), if !cancel_seen => {
                    cancel_seen = true;
                    cancelled.store(true, Ordering::Release);
                }
                received = done_rx.recv() => match received {
                    Some(outcome) => outcomes.push(outcome),
                    None => break,
                },
            }
        }

        let (terminal, message) = summarize(pending, &outcomes, cancel_seen || ctx.is_cancelled());
        let result = match terminal {
            Terminal::Success => ctx.complete(true, message).await,
            Terminal::Failed => ctx.complete(false, message).await,
            Terminal::Cancelled => ctx.complete_cancelled(false, message).await,
        };
        if let Err(e) = result {
            error!("move_to_ready complete: {e}");
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::ARM_DOF;

    #[test]
    fn ready_mirrors_the_shoulder_and_keeps_the_elbow_and_wrist() {
        let right = ready_posture(Side::Right);
        let left = ready_posture(Side::Left);
        assert_eq!(right, READY_R);
        for j in 0..3 {
            assert_eq!(left[j], -right[j]);
        }
        for j in 3..ARM_DOF {
            assert_eq!(left[j], right[j]);
        }
    }

    #[test]
    fn ready_posture_sits_inside_both_generations_joint_limits() {
        // The planner sends ready_posture unclamped, so an out-of-limit Ready
        // would reach the arms; this pin is what stands in for a clamp.
        use openarm_description::{HardwareVersion, Side as ModelSide};
        for version in [HardwareVersion::V1, HardwareVersion::V2] {
            for (side, model_side) in [
                (Side::Left, ModelSide::Left),
                (Side::Right, ModelSide::Right),
            ] {
                let model = crate::arm_model(version, version.base_link(model_side))
                    .expect("build arm from the bundled URDF");
                let limits = model.limits();
                for (j, (&q, limit)) in ready_posture(side).iter().zip(&limits).enumerate() {
                    assert!(
                        q >= limit.lo && q <= limit.hi,
                        "{version:?} {} j{}: ready {q} outside [{}, {}]",
                        side.label(),
                        j + 1,
                        limit.lo,
                        limit.hi
                    );
                }
            }
        }
    }

    fn outcome(success: bool, message: &str) -> ReadyOutcome {
        ReadyOutcome {
            success,
            message: message.to_string(),
        }
    }

    #[test]
    fn both_successful_outcomes_complete_successfully() {
        let outcomes = [
            outcome(true, "trajectory complete"),
            outcome(true, "trajectory complete"),
        ];
        let (terminal, message) = summarize(2, &outcomes, false);
        assert_eq!(terminal, Terminal::Success);
        assert_eq!(message, "both arms at ready");
    }

    #[test]
    fn a_failed_arm_fails_the_goal_with_its_message() {
        let outcomes = [
            outcome(true, "trajectory complete"),
            outcome(false, "goal cancelled"),
        ];
        let (terminal, message) = summarize(2, &outcomes, false);
        assert_eq!(terminal, Terminal::Failed);
        assert_eq!(message, "goal cancelled");
    }

    #[test]
    fn a_dropped_planner_reply_fails_with_a_matching_message() {
        // done_rx closed after one success: success and message must derive
        // from the same predicate, so this cannot read "both arms at ready".
        let outcomes = [outcome(true, "trajectory complete")];
        let (terminal, message) = summarize(2, &outcomes, false);
        assert_eq!(terminal, Terminal::Failed);
        assert_eq!(message, "a planner dropped the move");
    }

    #[test]
    fn no_dispatched_goal_is_a_planner_failure() {
        let (terminal, message) = summarize(0, &[], false);
        assert_eq!(terminal, Terminal::Failed);
        assert_eq!(message, "an arm's planner is unavailable");
        let (terminal, _) = summarize(1, &[outcome(true, "trajectory complete")], false);
        assert_eq!(terminal, Terminal::Failed);
    }

    #[test]
    fn two_failures_report_the_first_received() {
        let outcomes = [
            outcome(false, "left: IK failed mid-trajectory"),
            outcome(false, "right: motion timed out"),
        ];
        let (terminal, message) = summarize(2, &outcomes, false);
        assert_eq!(terminal, Terminal::Failed);
        assert_eq!(message, "left: IK failed mid-trajectory");
    }

    #[test]
    fn a_cancel_with_nothing_pending_still_completes_cancelled() {
        let (terminal, message) = summarize(0, &[], true);
        assert_eq!(terminal, Terminal::Cancelled);
        assert_eq!(message, "goal cancelled");
    }

    #[test]
    fn a_cancel_after_both_arms_succeeded_reads_as_cancelled() {
        let outcomes = [
            outcome(true, "trajectory complete"),
            outcome(true, "trajectory complete"),
        ];
        let (terminal, message) = summarize(2, &outcomes, true);
        assert_eq!(terminal, Terminal::Cancelled);
        assert_eq!(message, "goal cancelled");
    }
}
