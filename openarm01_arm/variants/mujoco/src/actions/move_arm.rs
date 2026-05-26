// move_arm action — cartesian (desired_position + desired_orientation).
//
// Stub on this sim variant: every goal is rejected with accepted=false.
// The cartesian → joint translation requires IK that lives in the
// backbone's safety pipeline (post-MVP Module 4). Until that lands the
// real control path is move_arm_joints; backbone consumers fire that
// directly with IK-solved joints.
//
// Matches the behaviour of the default (real) variant authored by Jared
// (openarm01_arm/variants/default/src/main.rs), so the contract shape is
// identical across all three variants — same reject, same surface, just a
// different physical end-effector.

use std::sync::Arc;

use peppygen::NodeRunner;
use peppygen::exposed_actions::move_arm;
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
            Ok(move_arm::GoalResponse::new(false))
        });
        tokio::select! {
            _ = token.cancelled() => break,
            result = goal_request => {
                if let Err(e) = result {
                    error!("move_arm goal: {e}");
                }
            }
        }
    }
}
