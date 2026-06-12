use std::time::Duration;

use peppygen::NodeRunner;
use peppygen::consumed_services::robot_init_is_ready;
use peppylib::runtime::CancellationToken;
use tracing::{info, warn};

const IS_READY_POLL_INTERVAL: Duration = Duration::from_millis(500);
const SERVICE_TIMEOUT: Duration = Duration::from_secs(5);

// Block until the launcher-bound robot_initializer reports ready, then return so
// backbone can expose its own actions. Left/right arm + gripper identity is
// launcher-pinned via link_ids, so no runtime discovery is needed.
pub async fn wait_until_ready(runner: &NodeRunner, token: &CancellationToken) {
    loop {
        // Race the poll itself against the token so cancellation arriving
        // mid-poll is seen immediately rather than after SERVICE_TIMEOUT.
        let result = tokio::select! {
            _ = token.cancelled() => return,
            result = robot_init_is_ready::poll(runner, SERVICE_TIMEOUT) => result,
        };
        match result {
            Ok(resp) if resp.data.ready => {
                info!("robot_initializer reported ready");
                return;
            }
            Ok(_) => {}
            Err(e) => warn!(error = %e, "is_ready poll failed; retrying"),
        }
        tokio::select! {
            _ = token.cancelled() => return,
            _ = tokio::time::sleep(IS_READY_POLL_INTERVAL) => {}
        }
    }
}
