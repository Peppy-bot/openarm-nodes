// move_arm action — cartesian (desired_position + desired_orientation).
//
// Stub on this sim sibling: every goal is rejected. The cartesian → joint
// translation requires IK that lives in the backbone's safety pipeline
// (post-MVP Module 4). Until that lands the real control path is
// move_arm_joints; backbone consumers fire that directly with IK-solved
// joints. Matches the behaviour of the openarm01_arm real-default impl
// (Jared, PR #22) so the contract shape is identical across siblings.

use std::sync::Arc;

use peppygen::NodeRunner;
use peppygen::exposed_actions::openarm01_arm::v1::move_arm;
use peppylib::runtime::CancellationToken;
use tracing::{error, warn};

pub async fn run(runner: Arc<NodeRunner>, token: CancellationToken) {
    let mut action_handle = move_arm::ActionHandle::expose(&runner)
        .await
        .expect("expose move_arm");

    loop {
        let goal_request = action_handle.handle_goal_next_request(|_req| {
            warn!(
                "move_arm: rejecting cartesian goal — embedded IK not implemented; \
                 backbone should call move_arm_joints with IK-solved joints"
            );
            Ok(move_arm::GoalResponse::reject(
                "cartesian move_arm not implemented; use move_arm_joints",
            ))
        });
        tokio::select! {
            _ = token.cancelled() => break,
            result = goal_request => {
                match result {
                    Ok(_) => continue,
                    Err(e) => {
                        error!("move_arm goal: {e}");
                    }
                }
            }
        }
    }
}
