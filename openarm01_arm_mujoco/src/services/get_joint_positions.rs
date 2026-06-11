// get_joint_positions — returns the latest 7-DOF joint positions from the
// telemetry cache. Errors until the first joint_states sample arrives so a
// caller can never mistake "no data yet" for a real all-zero pose.

use std::sync::Arc;

use peppygen::NodeRunner;
use peppygen::exposed_services::openarm01_arm::v1::get_joint_positions;
use peppylib::runtime::CancellationToken;
use tracing::error;

use crate::state::SharedState;

const DOF: usize = 7;

pub async fn run(runner: Arc<NodeRunner>, state: Arc<SharedState>, token: CancellationToken) {
    loop {
        let state = state.clone();
        tokio::select! {
            _ = token.cancelled() => break,
            result = get_joint_positions::handle_next_request(&runner, move |_req| {
                let positions: Option<[f64; DOF]> = state
                    .joint_states
                    .lock()
                    .unwrap_or_else(|p| p.into_inner())
                    .as_ref()
                    .and_then(|s| s.positions.as_slice().try_into().ok());
                match positions {
                    Some(p) => Ok(get_joint_positions::Response::new(p)),
                    None => Err(peppylib::PeppyError::Io(std::io::Error::other(
                        "arm telemetry not ready",
                    ))),
                }
            }) => {
                if let Err(e) = result {
                    error!("get_joint_positions: {e}");
                }
            }
        }
    }
}
