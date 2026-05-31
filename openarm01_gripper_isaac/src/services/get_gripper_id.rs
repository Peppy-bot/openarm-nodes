use std::sync::Arc;

use peppygen::NodeRunner;
use peppygen::exposed_services::openarm01_gripper::v1::get_gripper_id;
use peppylib::runtime::CancellationToken;
use tracing::error;

use crate::config::GripperId;

pub async fn run(
    runner: Arc<NodeRunner>,
    gripper_id: GripperId,
    token: CancellationToken,
) {
    loop {
        tokio::select! {
            _ = token.cancelled() => break,
            result = get_gripper_id::handle_next_request(&runner, |_req| {
                Ok(get_gripper_id::Response::new(gripper_id.0))
            }) => {
                if let Err(e) = result {
                    error!("get_gripper_id: {e}");
                }
            }
        }
    }
}
