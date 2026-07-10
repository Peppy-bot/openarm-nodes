// Live self-collision proximity for the UI. Consumes the hub's `collision_status`
// stream and writes the nearest-pair distance and link names into UiState, so the
// panel can show how close the arms are and color it against the governor band.

use std::sync::Arc;
use std::time::Instant;

use peppygen::NodeRunner;
use peppygen::consumed_topics::proximity_collision_status;
use peppylib::runtime::CancellationToken;
use tracing::error;

use crate::state::{Disposition, Proximity, SharedState};

pub async fn run(runner: Arc<NodeRunner>, state: SharedState, token: CancellationToken) {
    let mut subscription = match proximity_collision_status::subscribe(&runner).await {
        Ok(subscription) => subscription,
        Err(e) => {
            error!(error = %e, "collision_status subscribe");
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
                error!(error = %e, "collision_status receive");
                continue;
            }
        };
        let mut s = state.lock().unwrap_or_else(|p| p.into_inner());
        s.proximity = Some(Proximity {
            distance: msg.distance,
            link_a: msg.link_a,
            link_b: msg.link_b,
            disposition: Disposition::from_wire(msg.throttled, msg.stopped),
            received_at: Instant::now(),
        });
    }
}
