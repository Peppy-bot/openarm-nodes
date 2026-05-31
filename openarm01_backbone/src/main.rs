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

        // Gate on the world being ready before exposing any action.
        startup::wait_until_ready(&node_runner, &token).await;

        // Spawn action handlers as background tasks and return Ok so NodeBuilder
        // can finish bringing up framework services (notably node_health).
        // Awaiting via try_join! here would hold the closure open forever
        // (loops never complete) and starve framework finalisation, which
        // causes peppy's health probe to time out.
        //
        // A JoinSet supervises both handlers in a detached task so an early
        // exit (ActionHandle::expose error or panic in a callback) surfaces in
        // the logs instead of disappearing.
        let mut set = JoinSet::new();
        set.spawn(actions::move_arm_joints::run(
            node_runner.clone(),
            token.clone(),
        ));
        set.spawn(actions::move_gripper::run(
            node_runner.clone(),
            token.clone(),
        ));
        tokio::spawn(async move {
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
