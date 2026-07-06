// Live gripper opening for the UI. Each side's pairing slot delivers the
// paired gripper's `gripper_states` — the slot IS the side, so there is no
// gripper_id demux — and writes the latest measured opening into UiState.
// Mirrors joint_states.rs for the arm: a slot with no paired gripper stays
// silent, and the panel shows live state whether or not a move is in flight.

use std::sync::Arc;

use peppygen::NodeRunner;
use peppygen::pairings::{left_gripper, right_gripper};
use peppylib::runtime::CancellationToken;
use tokio::task::JoinHandle;
use tracing::error;

use crate::state::{SharedState, Side};

/// The generated pairing modules are distinct types with identical shapes;
/// this is the one surface the receive loop needs from either side's
/// subscription.
trait GripperStatesSubscription: Send + 'static {
    /// The next paired peer's gripper position, or `None` on shutdown.
    /// Spelled as an RPITIT rather than `async fn` so the future is `Send`
    /// even where the implementor is opaque, as `tokio::spawn` requires.
    fn next_position(&mut self) -> impl Future<Output = peppygen::Result<Option<f64>>> + Send;
}

impl GripperStatesSubscription for left_gripper::gripper_states::Subscription {
    async fn next_position(&mut self) -> peppygen::Result<Option<f64>> {
        Ok(self.next().await?.map(|(_producer, msg)| msg.position))
    }
}

impl GripperStatesSubscription for right_gripper::gripper_states::Subscription {
    async fn next_position(&mut self) -> peppygen::Result<Option<f64>> {
        Ok(self.next().await?.map(|(_producer, msg)| msg.position))
    }
}

fn spawn_side(
    subscription: peppygen::Result<impl GripperStatesSubscription>,
    side: Side,
    state: &SharedState,
    token: &CancellationToken,
) -> Option<JoinHandle<()>> {
    match subscription {
        Ok(subscription) => Some(tokio::spawn(side_loop(
            subscription,
            side,
            state.clone(),
            token.clone(),
        ))),
        Err(e) => {
            error!(error = %e, side = side.label(), "gripper_states subscribe");
            None
        }
    }
}

async fn side_loop(
    mut subscription: impl GripperStatesSubscription,
    side: Side,
    state: SharedState,
    token: CancellationToken,
) {
    loop {
        let received = tokio::select! {
            _ = token.cancelled() => return,
            received = subscription.next_position() => received,
        };
        let position = match received {
            Ok(Some(position)) => position,
            Ok(None) => return,
            Err(e) => {
                error!(error = %e, side = side.label(), "gripper_states receive");
                continue;
            }
        };
        let mut s = state.lock().unwrap_or_else(|p| p.into_inner());
        s.gripper_mut(side).last_feedback = Some(position);
    }
}

pub async fn run(runner: Arc<NodeRunner>, state: SharedState, token: CancellationToken) {
    let (left, right) = tokio::join!(
        left_gripper::gripper_states::subscribe(&runner),
        right_gripper::gripper_states::subscribe(&runner),
    );
    let left = spawn_side(left, Side::Left, &state, &token);
    let right = spawn_side(right, Side::Right, &state, &token);
    if let Some(task) = left {
        let _ = task.await;
    }
    if let Some(task) = right {
        let _ = task.await;
    }
}
