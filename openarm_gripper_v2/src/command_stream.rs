// Listens for streamed opening setpoints from the paired commander (the
// `commander` pairing slot of openarm_gripper_link) and keeps the latest one
// in a watch channel for the follow loop. Subscribing while unpaired is legal:
// the subscription stays silent until a commander pairs, and only the paired
// peer's messages surface, so there is no gripper_id filter. A non-finite
// position is dropped, so a commander gone bad lets the follow lock time out
// instead of driving the gripper. stream.rs is the return direction; this is
// the command direction.

use std::sync::Arc;
use std::time::Instant;

use peppygen::NodeRunner;
use peppygen::peers::commander::gripper_commands;
use peppylib::runtime::CancellationToken;
use tokio::sync::watch;
use tracing::{error, warn};

#[derive(Clone, Copy)]
pub struct GripperCommand {
    pub recv_at: Instant,
    pub position: f64,
}

pub async fn run(
    runner: Arc<NodeRunner>,
    latest: watch::Sender<Option<GripperCommand>>,
    token: CancellationToken,
) {
    let mut subscription = match gripper_commands::subscribe(&runner).await {
        Ok(subscription) => subscription,
        Err(e) => {
            error!(error = %e, "gripper_commands subscribe");
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
                error!(error = %e, "gripper_commands receive");
                continue;
            }
        };
        if !msg.position.is_finite() {
            warn!("gripper_commands: dropping message with non-finite position");
            continue;
        }
        latest.send_replace(Some(GripperCommand {
            recv_at: Instant::now(),
            position: msg.position,
        }));
    }
}
