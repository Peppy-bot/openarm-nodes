mod actions;
mod startup;

use peppygen::{NodeBuilder, Parameters, Result};

fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::INFO)
        .init();

    NodeBuilder::new().run(|_params: Parameters, node_runner| async move {
        let token = node_runner.cancellation_token().clone();

        // Gate on the world being ready, then learn which instance serves each arm/gripper side.
        let routing = std::sync::Arc::new(startup::run(&node_runner, &token).await);

        // Spawn action handlers as background tasks and return Ok so NodeBuilder
        // can finish bringing up framework services (notably node_health).
        // Awaiting via try_join! here would hold the closure open forever
        // (loops never complete) and starve framework finalisation, which
        // causes peppy's health probe to time out.
        tokio::spawn(actions::move_arm_joints::run(
            node_runner.clone(),
            routing.clone(),
            token.clone(),
        ));
        tokio::spawn(actions::move_gripper::run(
            node_runner.clone(),
            routing.clone(),
            token.clone(),
        ));

        Ok(())
    })
}
