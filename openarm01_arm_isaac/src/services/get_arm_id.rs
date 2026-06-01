use std::sync::Arc;

use peppygen::NodeRunner;
use peppygen::exposed_services::openarm01_arm::v1::get_arm_id;
use peppylib::runtime::CancellationToken;
use tracing::error;

use crate::config::ArmId;

pub async fn run(
    runner: Arc<NodeRunner>,
    arm_id: ArmId,
    token: CancellationToken,
) {
    loop {
        tokio::select! {
            _ = token.cancelled() => break,
            result = get_arm_id::handle_next_request(&runner, |_req| {
                Ok(get_arm_id::Response::new(arm_id.0))
            }) => {
                if let Err(e) = result {
                    error!("get_arm_id: {e}");
                }
            }
        }
    }
}
