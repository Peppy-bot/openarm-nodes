// Listens for streamed joint setpoints (openarm01_joint_command_source) and
// keeps the latest one addressed to this arm in a watch channel for the follow
// loop. A message with any non-finite position is dropped, so a producer gone
// bad lets the follow lock time out instead of driving the arm.

use std::sync::Arc;
use std::time::Instant;

use peppygen::NodeRunner;
use peppygen::consumed_topics::commander_joint_commands;
use peppylib::runtime::CancellationToken;
use tokio::sync::watch;
use tracing::{error, warn};

use crate::config::ArmId;
use crate::trajectory::JointVec;

#[derive(Clone, Copy)]
pub struct JointCommand {
    pub recv_at: Instant,
    pub positions: JointVec,
}

pub async fn run(
    runner: Arc<NodeRunner>,
    arm_id: ArmId,
    latest: watch::Sender<Option<JointCommand>>,
    token: CancellationToken,
) {
    loop {
        let received = tokio::select! {
            _ = token.cancelled() => return,
            received = commander_joint_commands::on_next_message_received(&runner) => received,
        };
        let (_producer, msg) = match received {
            Ok(pair) => pair,
            Err(e) => {
                error!(error = %e, "joint_commands receive");
                continue;
            }
        };
        if msg.arm_id != arm_id.raw() {
            continue;
        }
        if !msg.positions.iter().all(|v| v.is_finite()) {
            warn!("joint_commands: dropping message with non-finite positions");
            continue;
        }
        latest.send_replace(Some(JointCommand {
            recv_at: Instant::now(),
            positions: msg.positions,
        }));
    }
}
