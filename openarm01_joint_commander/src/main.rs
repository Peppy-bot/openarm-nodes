mod actions;
mod command_stream;
mod error;
mod gripper_states;
mod joint_states;
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

    NodeBuilder::new().run(|params: Parameters, node_runner| async move {
        let token = node_runner.cancellation_token().clone();
        let shared = state::new_shared();

        // Rate feeds `Duration::from_micros(1_000_000 / rate)`, so a rate above
        // 1 MHz would round to a 0 µs period; no real deployment approaches that,
        // so just guard against zero.
        assert!(params.command_rate_hz > 0, "command_rate_hz must be > 0");

        // Feed the UI live arm + gripper state off the always-on state streams
        // (replaces move-progress relayed through the action feedback topics).
        tokio::spawn(joint_states::run(
            node_runner.clone(),
            shared.clone(),
            token.clone(),
        ));
        tokio::spawn(gripper_states::run(
            node_runner.clone(),
            shared.clone(),
            token.clone(),
        ));

        // Stream operator joint setpoints to the armed arms (deadman in UiState).
        tokio::spawn(command_stream::run(
            node_runner.clone(),
            shared.clone(),
            params.command_rate_hz,
            token.clone(),
        ));

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
