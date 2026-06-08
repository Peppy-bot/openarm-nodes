mod actions;
mod startup;

use peppygen::{NodeBuilder, Parameters, Result};
use tokio::task::JoinSet;
use tracing::{error, info};

fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::INFO)
        .init();

    NodeBuilder::new().run(|_params: Parameters, node_runner| async move {
        let token = node_runner.cancellation_token().clone();

        // Spawn the entire startup-and-action-handler chain in the background
        // so this setup closure returns immediately and NodeBuilder can
        // register `node_health` before the daemon's health probe fires.
        // Awaiting `wait_until_ready` here blocked the closure for the full
        // Isaac USD-load window (~30-60s on cold start), which tripped peppy's
        // health-probe timeout and the daemon SIGKILL'd the instance before it
        // could expose actions.
        //
        // A JoinSet supervises both handlers so an early exit
        // (ActionHandle::expose error or panic in a callback) surfaces in the
        // logs instead of disappearing.
        tokio::spawn(async move {
            // Gate on the world being ready before exposing any action.
            startup::wait_until_ready(&node_runner, &token).await;

            let mut set = JoinSet::new();
            set.spawn(actions::move_arm_joints::run(
                node_runner.clone(),
                token.clone(),
            ));
            set.spawn(actions::move_gripper::run(
                node_runner.clone(),
                token.clone(),
            ));
            while let Some(joined) = set.join_next().await {
                match joined {
                    Ok(Ok(())) => info!("backbone action handler exited cleanly"),
                    Ok(Err(e)) => error!(error = %e, "backbone action handler returned Err"),
                    Err(e) if e.is_panic() => error!(error = %e, "backbone action handler panicked"),
                    Err(e) => error!(error = %e, "backbone action handler join failed"),
                }
            }
        });

        Ok(())
    })
}
