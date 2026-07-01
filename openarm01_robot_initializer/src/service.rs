use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use peppygen::NodeRunner;
use peppygen::consumed_services::{
    left_arm_is_ready, left_gripper_is_ready, right_arm_is_ready, right_gripper_is_ready,
};
use peppygen::exposed_services::openarm01_robot_initializer::v1::is_ready;
use peppylib::runtime::CancellationToken;

// How often the poller re-checks every component, and how long each component
// poll waits before treating an unreachable component as not-ready.
const POLL_INTERVAL: Duration = Duration::from_millis(500);
const POLL_TIMEOUT: Duration = Duration::from_secs(2);

pub async fn run(runner: Arc<NodeRunner>, token: CancellationToken) {
    // The generated is_ready handler closure is synchronous, so it cannot poll
    // the components itself. A background task caches their aggregate readiness
    // here and the handler just reads it.
    let ready = Arc::new(AtomicBool::new(false));
    tokio::spawn(poll_components(
        runner.clone(),
        ready.clone(),
        token.clone(),
    ));

    tracing::info!("is_ready service started");
    loop {
        tokio::select! {
            _ = token.cancelled() => {
                tracing::info!("is_ready service shutting down");
                break;
            }
            result = is_ready::handle_next_request(&runner, |_req| {
                Ok(is_ready::Response::new(ready.load(Ordering::SeqCst)))
            }) => {
                if let Err(e) = result {
                    tracing::warn!("is_ready handler error: {e}");
                }
            }
        }
    }
}

async fn poll_components(
    runner: Arc<NodeRunner>,
    ready: Arc<AtomicBool>,
    token: CancellationToken,
) {
    // Re-poll each tick (not latch) so a component that dies flips the robot back
    // to not-ready. A plain sleep loop, not interval(), to avoid burst catch-up
    // after a slow tick.
    loop {
        ready.store(hardware_ready(&runner).await, Ordering::SeqCst);
        tokio::select! {
            _ = token.cancelled() => break,
            _ = tokio::time::sleep(POLL_INTERVAL) => {}
        }
    }
}

// Sequential && short-circuits: a not-ready component returns false without
// polling the rest. The four poll() Response types differ with no shared trait,
// so the readiness check is inlined per link rather than factored generically.
async fn hardware_ready(runner: &NodeRunner) -> bool {
    matches!(left_arm_is_ready::poll(runner, POLL_TIMEOUT).await, Ok(r) if r.data.ready)
        && matches!(right_arm_is_ready::poll(runner, POLL_TIMEOUT).await, Ok(r) if r.data.ready)
        && matches!(left_gripper_is_ready::poll(runner, POLL_TIMEOUT).await, Ok(r) if r.data.ready)
        && matches!(right_gripper_is_ready::poll(runner, POLL_TIMEOUT).await, Ok(r) if r.data.ready)
}
