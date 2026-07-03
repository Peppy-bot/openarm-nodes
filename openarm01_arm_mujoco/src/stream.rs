// Listens for streamed joint setpoints from the paired commander (the
// `commander` pairing slot of openarm01_arm_joint_link) and keeps the latest
// one in a watch channel for the follow loop. Subscribing while unpaired is
// legal: the subscription stays silent until a commander pairs, and only the
// paired peer's messages surface, so there is no arm_id filter. A message
// with any non-finite position is dropped, so a commander gone bad lets the
// follow lock time out instead of driving the arm.

use std::sync::Arc;
use std::time::Instant;

use peppygen::NodeRunner;
use peppygen::peers::commander::joint_commands;
use peppylib::runtime::CancellationToken;
use tokio::sync::watch;
use tracing::{error, warn};

use crate::trajectory::JointVec;

#[derive(Clone, Copy)]
pub struct JointCommand {
    pub recv_at: Instant,
    pub positions: JointVec,
}

pub async fn run(
    runner: Arc<NodeRunner>,
    latest: watch::Sender<Option<JointCommand>>,
    token: CancellationToken,
) {
    let mut subscription = match joint_commands::subscribe(&runner).await {
        Ok(subscription) => subscription,
        Err(e) => {
            error!(error = %e, "joint_commands subscribe");
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
                error!(error = %e, "joint_commands receive");
                continue;
            }
        };
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
