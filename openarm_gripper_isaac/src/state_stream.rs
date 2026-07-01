// Consume the sim's measured gripper opening (gripper_states) and cache the
// latest sample for this gripper. The move action reads it on each feedback tick
// to compute convergence + stall. Replaces the old sim-bridge telemetry pipeline
// that re-emitted state; the sim now emits gripper_states directly.

use std::sync::Arc;
use std::time::Instant;

use peppygen::NodeRunner;
use peppygen::consumed_topics::state_gripper_states;
use peppylib::runtime::CancellationToken;
use tracing::error;

use crate::config::{ApertureMap, GripperId};
use crate::state::{GripperStateLatest, SharedState};

pub async fn run(
    runner: Arc<NodeRunner>,
    gripper_id: GripperId,
    map: ApertureMap,
    state: Arc<SharedState>,
    token: CancellationToken,
) {
    let mut subscription = match state_gripper_states::subscribe(&runner).await {
        Ok(subscription) => subscription,
        Err(e) => {
            error!(error = %e, "gripper_states subscribe");
            return;
        }
    };
    loop {
        let received = tokio::select! {
            _ = token.cancelled() => return,
            received = subscription.next() => received,
        };
        let (_producer, msg) = match received {
            Ok(Some(pair)) => pair,
            Ok(None) => return,
            Err(e) => {
                error!(error = %e, "gripper_states receive");
                continue;
            }
        };
        if msg.gripper_id != gripper_id.as_u8() || !msg.position.is_finite() {
            continue;
        }
        let mut latest = state
            .gripper_state
            .lock()
            .unwrap_or_else(|p| p.into_inner());
        *latest = Some(GripperStateLatest {
            opening: map.to_aperture(msg.position),
            recv_at: Instant::now(),
        });
    }
}
