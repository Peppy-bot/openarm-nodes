use std::sync::Arc;

use peppygen::NodeRunner;
use peppygen::exposed_services::is_ready;
use peppylib::runtime::CancellationToken;

pub async fn run(runner: Arc<NodeRunner>, token: CancellationToken) {
    tracing::info!("is_ready service started");
    loop {
        tokio::select! {
            _ = token.cancelled() => {
                tracing::info!("is_ready service shutting down");
                break;
            }
            result = is_ready::handle_next_request(&runner, |_req| {
                // TODO(@jarednm): real robot health checks before returning ready
                Ok(is_ready::Response::new(true))
            }) => {
                if let Err(e) = result {
                    tracing::warn!("is_ready handler error: {e}");
                }
            }
        }
    }
}
