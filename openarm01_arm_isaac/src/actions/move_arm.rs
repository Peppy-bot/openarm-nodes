// move_arm (Cartesian) — always rejects. The real driver solves IK in-process;
// the sim siblings have no kinematics, so callers must use move_arm_joints.
// Serving fast rejections beats letting Cartesian goals time out.

use std::sync::Arc;

use peppygen::NodeRunner;
use peppygen::exposed_actions::openarm01_arm::v1::move_arm;
use peppylib::runtime::CancellationToken;
use tracing::error;

pub async fn run(runner: Arc<NodeRunner>, token: CancellationToken) {
    let mut handle = match move_arm::ActionHandle::expose(&runner).await {
        Ok(h) => h,
        Err(e) => {
            error!("expose move_arm: {e}");
            return;
        }
    };

    loop {
        tokio::select! {
            _ = token.cancelled() => break,
            result = handle.handle_goal_next_request(|_req: &move_arm::GoalRequest| {
                Ok(move_arm::GoalResponse::reject(
                    "cartesian control not available in sim; use move_arm_joints",
                ))
            }) => match result {
                Ok(Some(_ctx)) => {} // unreachable: the decider never accepts
                Ok(None) => break,   // action stream closed (shutdown)
                Err(e) => error!("move_arm goal: {e}"),
            }
        }
    }
}
