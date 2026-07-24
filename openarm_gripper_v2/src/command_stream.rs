// Listens for streamed opening setpoints from the paired backbone (the
// `backbone` pairing slot of gripper_link) and keeps the latest one
// in a watch channel for the follow loop. Subscribing while unpaired is legal:
// the subscription stays silent until a backbone pairs, and only the paired
// peer's messages surface, so there is no gripper_id filter. A non-finite
// position is dropped rather than driving the gripper. stream.rs is the
// return direction; this is the command direction.

use std::sync::Arc;

use peppygen::NodeRunner;
use peppygen::paired_topics::backbone;
use peppylib::runtime::CancellationToken;
use tokio::sync::watch;
use tracing::{error, warn};

#[derive(Clone, Copy)]
pub struct GripperCommand {
    pub opening: f64,
    /// Commanded effort cap (N*m at the shaft); `None` when the wire carried
    /// no preference (0), so the configured ceiling applies.
    pub max_effort: Option<f64>,
}

pub async fn run(
    runner: Arc<NodeRunner>,
    latest: watch::Sender<Option<GripperCommand>>,
    token: CancellationToken,
) {
    let mut subscription = match backbone::gripper_setpoints::subscribe(&runner).await {
        Ok(subscription) => subscription,
        Err(e) => {
            error!(error = %e, "gripper_setpoints subscribe");
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
                error!(error = %e, "gripper_setpoints receive");
                continue;
            }
        };
        if !msg.opening.is_finite() || !msg.max_effort.is_finite() {
            warn!("gripper_setpoints: dropping message with non-finite fields");
            continue;
        }
        if msg.max_effort < 0.0 {
            warn!("gripper_setpoints: dropping message with negative max_effort");
            continue;
        }
        latest.send_replace(Some(GripperCommand {
            opening: msg.opening,
            max_effort: (msg.max_effort > 0.0).then_some(msg.max_effort),
        }));
    }
}
