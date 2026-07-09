// Ambient following of a streamed gripper opening: publish the latest fresh
// opening to the sim; when the stream goes stale, hold by publishing nothing so
// the sim keeps its last setpoint. The opening is published directly; the sim
// splits it across the fingers and its servo eases to it.

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
    open_m: f64,
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

        // Follow only a command still within the stream timeout; otherwise hold
        // (publish nothing, the sim keeps the last setpoint).
        let position = {
            let guard = cmd.borrow();
            guard
                .as_ref()
                .filter(|c| c.recv_at.elapsed() <= params.stream_timeout)
                .map(|c| c.position)
        };
        let Some(position) = position else {
            continue;
        };
        // Clamp defensively (a producer could stream out of range); the sim
        // maps the aperture onto its finger joints.
        let opening = position.clamp(0.0, open_m);

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
