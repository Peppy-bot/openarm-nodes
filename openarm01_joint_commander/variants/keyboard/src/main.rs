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

        // The UI loop is the only task — everything else (action firing) is
        // spawned transiently from key handlers. Errors from inside the UI
        // are logged here, not returned, so the node exits cleanly via the
        // TerminalGuard's Drop rather than tearing down on a panic / Result
        // error path.
        if let Err(e) = ui::run(node_runner.clone(), shared, token).await {
            error!(error = %e, "ui loop exited with error");
        }
        Ok(())
    })
}
