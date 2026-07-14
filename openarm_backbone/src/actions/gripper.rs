//! Gripper move-action admission: the `move_gripper` handler the backbone exposes.
//! Mirrors the arm move admission exactly: validate the goal (gripper_id,
//! finiteness, the [0, 1] opening range), claim the side's single-flight slot, and
//! hand the accepted goal to the coordinator over its gripper goal channel. The
//! coordinator runs the motion through the same per-tick governing as every
//! other DOF, completes the goal on measured convergence, and releases the busy
//! slot at the terminal.

use std::sync::Arc;
use std::sync::atomic::AtomicBool;

use peppygen::exposed_actions::move_gripper::{ActionHandle, GoalResponse};
use peppygen::{NodeRunner, Result};
use tokio::sync::mpsc;
use tracing::error;

use crate::Side;
use crate::actions::claim;
use crate::coordinator::GripperGoal;

/// Expose `move_gripper`: validate + claim, then hand the goal to the
/// coordinator. The coordinator releases the busy slot when the move ends.
pub async fn run_move_gripper(
    runner: Arc<NodeRunner>,
    goal_txs: [mpsc::Sender<GripperGoal>; 2],
    busy: [Arc<AtomicBool>; 2],
) -> Result<()> {
    let mut handle = ActionHandle::expose(&runner).await?;
    loop {
        let accepted = handle
            .handle_goal_next_request(|req| {
                let d = &req.data;
                let Some(idx) = Side::from_gripper_id(d.gripper_id).map(Side::index) else {
                    return Ok(GoalResponse::reject("gripper_id out of range"));
                };
                if !d.opening.is_finite() {
                    return Ok(GoalResponse::reject("non-finite gripper opening"));
                }
                if !(0.0..=1.0).contains(&d.opening) {
                    return Ok(GoalResponse::reject(format!(
                        "opening {} outside [0, 1]",
                        d.opening
                    )));
                }
                if !claim(&busy[idx]) {
                    return Ok(GoalResponse::reject("gripper is already executing a move"));
                }
                Ok(GoalResponse::accept())
            })
            .await?;
        let Some(ctx) = accepted else { return Ok(()) };
        let idx = Side::from_gripper_id(ctx.request().data.gripper_id)
            .map(Side::index)
            .expect("validated on accept");
        let opening = ctx.request().data.opening;
        if goal_txs[idx]
            .send(GripperGoal { opening, ctx })
            .await
            .is_err()
        {
            busy[idx].store(false, std::sync::atomic::Ordering::Release);
            error!("move_gripper: coordinator channel closed");
            return Ok(());
        }
    }
}
