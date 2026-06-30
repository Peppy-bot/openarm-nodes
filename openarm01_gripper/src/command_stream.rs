// Listens for streamed gripper opening setpoints (openarm01_gripper_command_source)
// and keeps the latest one addressed to this gripper in a watch channel for the
// follow loop. A non-finite position is dropped, so a producer gone bad lets the
// follow lock time out instead of driving the gripper. The existing stream.rs is
// the return direction (gripper_states feedback); this is the command direction.

use std::sync::Arc;
use std::time::Instant;

use peppygen::NodeRunner;
use peppygen::consumed_topics::commander_gripper_commands;
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
    gripper_id: u8,
    latest: watch::Sender<Option<GripperCommand>>,
    token: CancellationToken,
) {
    let mut subscription = match commander_gripper_commands::subscribe(&runner).await {
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
        if msg.gripper_id != gripper_id {
            continue;
        }
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
