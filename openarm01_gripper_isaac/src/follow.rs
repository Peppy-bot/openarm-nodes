// Ambient following of a streamed gripper opening. While no move is running
// (busy gate clear), publish the latest fresh opening to the sim; when the stream
// goes stale, hold by publishing nothing so the sim keeps its last setpoint. The
// move action and this loop share the busy gate, so they never both drive the
// gripper. The opening is published directly; the sim splits it across the
// fingers and its servo eases to it.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use peppylib::TopicPublisher;
use peppylib::runtime::CancellationToken;
use tokio::sync::watch;
use tokio::time::MissedTickBehavior;
use tracing::warn;

use crate::config::{ControlParams, GRIPPER_OPEN_M};
use crate::passthrough;
use crate::stream::GripperCommand;

pub async fn run(
    passthrough_pub: TopicPublisher,
    gripper_id: u8,
    busy: Arc<AtomicBool>,
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

        // A move owns the gripper: yield so the action stays the only writer.
        if busy.load(Ordering::Acquire) {
            continue;
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
        // splits the aperture across the two fingers.
        let opening = position.clamp(0.0, GRIPPER_OPEN_M);

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
