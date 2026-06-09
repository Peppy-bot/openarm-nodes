// get_joint_positions — returns the latest 7-DOF joint positions from the
// telemetry cache. Before the first joint_states arrives, responds with
// zeros; backbone polls until live values appear (same shape as is_ready).

use std::sync::Arc;

use peppygen::NodeRunner;
use peppygen::exposed_services::openarm01_arm::v1::get_joint_positions;
use peppylib::runtime::CancellationToken;
use tracing::{error, warn};

use crate::state::SharedState;

const DOF: usize = 7;

pub async fn run(runner: Arc<NodeRunner>, state: Arc<SharedState>, token: CancellationToken) {
    loop {
        let state = state.clone();
        tokio::select! {
            _ = token.cancelled() => break,
            result = get_joint_positions::handle_next_request(&runner, move |_req| {
                let positions = snapshot_positions(&state);
                Ok(get_joint_positions::Response::new(positions))
            }) => {
                if let Err(e) = result {
                    error!("get_joint_positions: {e}");
                }
            }
        }
    }
}

fn snapshot_positions(state: &Arc<SharedState>) -> [f64; DOF] {
    let guard = state.joint_states.lock().unwrap_or_else(|p| p.into_inner());
    match guard.as_ref() {
        Some(latest) if latest.positions.len() == DOF => {
            let mut out = [0.0; DOF];
            out.copy_from_slice(&latest.positions);
            out
        }
        Some(latest) => {
            warn!(
                "get_joint_positions: cache has {} positions, expected {DOF} — returning zeros",
                latest.positions.len()
            );
            [0.0; DOF]
        }
        None => [0.0; DOF],
    }
}
