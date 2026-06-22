// Always-on joint_commands publisher. For each enabled arm, streams its target
// joint setpoint at command_rate_hz so the arm follows it; a disabled arm emits
// nothing, so the arm's stream timeout lapses and it holds. Re-publishing every
// tick (even an unchanged target) keeps the arm's producer lock alive between
// operator inputs. The arm clamps and rate-limits what it receives, so this only
// has to deliver the latest setpoint.
//
// Each arm runs its own publish task on its own interval: a single shared loop
// published Left then Right every tick, and because zenoh's publish resolves
// synchronously the two never overlapped, leaving Right permanently second and
// visibly less smooth whenever Left was also streaming. Independent tasks share
// the cloneable, lock-free publisher (one zenoh session) with no fixed order.

use std::sync::Arc;
use std::time::Duration;

use peppygen::NodeRunner;
use peppygen::emitted_topics::openarm01_joint_command_source::v1::joint_commands;
use peppylib::TopicPublisher;
use peppylib::runtime::CancellationToken;
use tokio::time::MissedTickBehavior;
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
    let left = tokio::spawn(stream_arm(
        publisher.clone(),
        state.clone(),
        Side::Left,
        command_rate_hz,
        token.clone(),
    ));
    let right = tokio::spawn(stream_arm(publisher, state, Side::Right, command_rate_hz, token));
    let _ = tokio::join!(left, right);
}

// Publish one arm's latest setpoint at command_rate_hz while it is enabled.
async fn stream_arm(
    publisher: TopicPublisher,
    state: SharedState,
    side: Side,
    command_rate_hz: u32,
    token: CancellationToken,
) {
    let period = Duration::from_micros(1_000_000 / command_rate_hz as u64);
    // interval (not sleep) so the publish cadence holds at command_rate_hz
    // instead of drifting by the per-tick work time; Delay avoids a catch-up
    // burst after a scheduling hiccup.
    let mut ticker = tokio::time::interval(period);
    ticker.set_missed_tick_behavior(MissedTickBehavior::Delay);
    let mut failing = false;

    loop {
        tokio::select! {
            _ = token.cancelled() => return,
            _ = ticker.tick() => {}
        }

        let target = {
            let s = state.lock().unwrap_or_else(|p| p.into_inner());
            if !s.arm(side).enabled {
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
                warn!(
                    "joint_commands publish failing for {} arm, suppressing repeats: {e}",
                    side.label()
                );
            }
            Err(_) => {}
        }
    }
}
