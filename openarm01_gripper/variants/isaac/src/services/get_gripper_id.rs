use std::sync::Arc;

use peppygen::NodeRunner;
use peppygen::exposed_services::get_gripper_id;
use tracing::error;

use crate::config::GripperId;

pub async fn run(runner: Arc<NodeRunner>, gripper_id: GripperId) {
    loop {
        if let Err(e) = get_gripper_id::handle_next_request(&runner, |_req| {
            Ok(get_gripper_id::Response::new(gripper_id.0))
        })
        .await
        {
            error!("get_gripper_id: {e}");
        }
    }
}
