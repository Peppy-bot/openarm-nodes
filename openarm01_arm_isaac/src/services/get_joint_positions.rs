// get_joint_positions service — returns the latest 7-DOF joint positions
// from the telemetry cache. Used by the backbone to anchor IK / planning
// queries to the arm's current pose.
//
// If telemetry hasn't started yet (the sim's first joint_states hasn't
// arrived), responds with all zeros. Backbone treats this the same way
// `is_ready` short-circuits: it polls until live values appear.

use std::sync::Arc;

use peppygen::NodeRunner;
use peppygen::exposed_services::openarm01_arm::v1::get_joint_positions;
use peppylib::runtime::CancellationToken;
use tracing::{error, warn};

use crate::state::SharedState;

const DOF: usize = 7;

pub async fn run(
    runner: Arc<NodeRunner>,
    state: Arc<SharedState>,
    token: CancellationToken,
) {
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
    let guard = match state.joint_states.try_lock() {
        Ok(g) => g,
        Err(_) => {
            warn!("get_joint_positions: contended cache lock, returning zeros");
            return [0.0; DOF];
        }
    };
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
