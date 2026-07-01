// Listens for streamed gripper opening setpoints (openarm_gripper_commands)
// and keeps the latest one addressed to this gripper in a watch channel for the
// follow loop. A non-finite position is dropped, so a producer gone bad lets the
// follow lock time out instead of driving the gripper.

use std::sync::Arc;
use std::time::Instant;

use peppygen::NodeRunner;
use peppygen::consumed_topics::commander_gripper_commands;
use peppylib::runtime::CancellationToken;
use tokio::sync::watch;
use tracing::{error, warn};

use crate::config::GripperId;

#[derive(Clone, Copy)]
pub struct GripperCommand {
    pub recv_at: Instant,
    pub position: f64,
}

pub async fn run(
    runner: Arc<NodeRunner>,
    gripper_id: GripperId,
    latest: watch::Sender<Option<GripperCommand>>,
    token: CancellationToken,
) {
    loop {
        let received = tokio::select! {
            _ = token.cancelled() => return,
            received = commander_gripper_commands::on_next_message_received(&runner) => received,
        };
        let (_producer, msg) = match received {
            Ok(pair) => pair,
            Err(e) => {
                error!(error = %e, "gripper_commands receive");
                continue;
            }
        };
        if msg.gripper_id != gripper_id.as_u8() {
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
