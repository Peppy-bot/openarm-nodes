// Ambient following of a streamed gripper opening fraction: publish the latest
// opening to the sim; until the first command arrives, hold by publishing
// nothing so the sim keeps its last setpoint. The opening is published
// directly; the sim splits it across the fingers and its servo eases to it.

use peppylib::TopicPublisher;
use peppylib::runtime::CancellationToken;
use tokio::sync::watch;
use tokio::time::MissedTickBehavior;
use tracing::warn;

use crate::config::ControlParams;
use crate::passthrough;
use crate::stream::GripperCommand;

pub async fn run(
    passthrough_pub: TopicPublisher,
    gripper_id: u8,
    cmd: watch::Receiver<Option<GripperCommand>>,
    params: ControlParams,
    token: CancellationToken,
) {
    let mut ticker = tokio::time::interval(params.control_period);
    ticker.set_missed_tick_behavior(MissedTickBehavior::Delay);
    let mut failing = false;

    loop {
        tokio::select! {
            _ = token.cancelled() => return,
            _ = ticker.tick() => {}
        }

        // Follow the latest command; until one arrives, hold (publish nothing,
        // the sim keeps the last setpoint).
        let opening = cmd.borrow().as_ref().map(|c| c.opening);
        let Some(opening) = opening else {
            continue;
        };
        // Clamp defensively (a producer could stream out of range); the sim
        // maps the fraction onto each finger joint's own travel.
        let opening = opening.clamp(0.0, 1.0);

        match passthrough::publish(&passthrough_pub, gripper_id, opening).await {
            Ok(()) => failing = false,
            Err(e) if !failing => {
                failing = true;
                warn!("follow passthrough publish failing, suppressing repeats: {e}");
            }
            Err(_) => {}
        }
    }
}
