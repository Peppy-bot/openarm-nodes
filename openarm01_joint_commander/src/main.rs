mod actions;
mod error;
mod state;
mod ui;

use std::time::Duration;

use peppygen::{NodeBuilder, Parameters, Result};
use tracing::{error, warn};

// How long the shutdown hook waits for in-flight goals to finish cancelling;
// must stay well under lifecycle.shutdown_grace_secs (default 3s).
const GOAL_CANCEL_WAIT: Duration = Duration::from_secs(2);

fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .with_writer(std::io::stderr)
        .init();

    NodeBuilder::new().run(|_params: Parameters, node_runner| async move {
        let token = node_runner.cancellation_token().clone();
        let shared = state::new_shared();

        // Shutdown obligation: halt commander-initiated motion. Cancelling an
        // in-flight goal's preempt token makes its task (still polled while
        // hooks run) send cancel_goal to backbone and finalize through the
        // normal result path, clearing in_flight. Re-collect every pass so a
        // goal fired in the race window around cancellation is caught too.
        let hook_state = shared.clone();
        node_runner.on_shutdown(async move {
            let deadline = tokio::time::Instant::now() + GOAL_CANCEL_WAIT;
            // Publish the deadline before cancelling anything: preempted goal
            // tasks cap their cancel_goal/get_result waits by it (see
            // actions::bounded_by_shutdown) so their fixed 5s/60s timeouts
            // cannot outlive this hook's window.
            hook_state
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .shutdown_deadline = Some(deadline);
            loop {
                let preempts = hook_state
                    .lock()
                    .unwrap_or_else(|p| p.into_inner())
                    .in_flight_preempts();
                if preempts.is_empty() {
                    return;
                }
                for preempt in &preempts {
                    preempt.cancel();
                }
                if tokio::time::Instant::now() >= deadline {
                    warn!(
                        goals = preempts.len(),
                        "in-flight goals were not cancelled within the shutdown wait"
                    );
                    return;
                }
                tokio::time::sleep(Duration::from_millis(25)).await;
            }
        });

        // ui::run is the long-lived HTTP + WebSocket server. It must be spawned
        // rather than awaited here: peppylib registers `node_health` only after
        // the setup closure returns, so awaiting a forever-task starves the
        // health probe and the daemon SIGKILLs the instance after ~10s.
        tokio::spawn(async move {
            if let Err(e) = ui::run(node_runner, shared, token.clone()).await {
                error!(error = %e, "ui server exited with error");
                // Stop from inside per the shutdown contract: a commander that
                // cannot serve its UI should tear the node down rather than
                // linger as a healthy-looking instance that serves nothing.
                token.cancel();
            }
        });
        Ok(())
    })
}
