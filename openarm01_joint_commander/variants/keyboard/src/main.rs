use std::sync::Arc;

use joint_commander::{actions, config, input};
use peppygen::{NodeBuilder, Parameters, Result};
use tokio::sync::mpsc;

fn main() -> Result<()> {
    let arm = config::Arm::from_runtime();

    NodeBuilder::new().run(move |_args: Parameters, runner| async move {
        let cfg = Arc::new(
            config::ArmConfig::load(arm)
                .inspect_err(|e| tracing::error!(%e, "config load failed"))?,
        );

        let (tx, rx) = mpsc::channel(32);
        input::spawn_stdin_reader(tx);

        let token = runner.cancellation_token().clone();
        tokio::select! {
            _ = token.cancelled() => {}
            _ = actions::run_command_loop(runner, cfg, rx) => {}
        }

        Ok(())
    })
}
