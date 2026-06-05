mod actions;
mod error;
mod state;
mod ui;

use peppygen::{NodeBuilder, Parameters, Result};
use tracing::error;

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

        // ui::run is the long-lived HTTP + WebSocket server. It must be spawned
        // rather than awaited here: peppylib registers `node_health` only after
        // the setup closure returns, so awaiting a forever-task starves the
        // health probe and the daemon SIGKILLs the instance after ~10s.
        tokio::spawn(async move {
            if let Err(e) = ui::run(node_runner, shared, token).await {
                error!(error = %e, "ui server exited with error");
            }
        });
        Ok(())
    })
}
