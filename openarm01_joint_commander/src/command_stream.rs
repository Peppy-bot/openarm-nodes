// Always-on joint_commands publisher. For each armed arm, streams its target
// joint setpoint at command_rate_hz so the arm follows it; a disarmed arm emits
// nothing, so the arm's stream timeout lapses and it holds. Re-publishing every
// tick (even an unchanged target) keeps the arm's producer lock alive between
// operator inputs. The arm clamps and rate-limits what it receives, so this only
// has to deliver the latest setpoint.

use std::sync::Arc;
use std::time::Duration;

use peppygen::NodeRunner;
use peppygen::emitted_topics::openarm01_joint_command_source::v1::joint_commands;
use peppylib::runtime::CancellationToken;
use tracing::{error, warn};

use crate::state::{SharedState, Side};

pub async fn run(
    runner: Arc<NodeRunner>,
    state: SharedState,
    command_rate_hz: u32,
    token: CancellationToken,
) {
    let publisher = match joint_commands::declare_publisher(&runner).await {
        Ok(p) => p,
        Err(e) => return error!("declare joint_commands publisher: {e}"),
    };
    let period = Duration::from_micros(1_000_000 / command_rate_hz as u64);
    let mut failing = false;
    loop {
        tokio::select! {
            _ = token.cancelled() => return,
            _ = tokio::time::sleep(period) => {}
        }
        for side in [Side::Left, Side::Right] {
            let target = {
                let s = state.lock().unwrap_or_else(|p| p.into_inner());
                if !s.arm(side).armed {
                    continue;
                }
                s.arm(side).joints
            };
            let result = async {
                let msg =
                    joint_commands::build_message(side.arm_id(), target).map_err(|e| e.to_string())?;
                publisher.publish(msg).await.map_err(|e| e.to_string())
            }
            .await;
            match result {
                Ok(()) => failing = false,
                Err(e) if !failing => {
                    failing = true;
                    warn!("joint_commands publish failing, suppressing repeats: {e}");
                }
                Err(_) => {}
            }
        }
    }
}
