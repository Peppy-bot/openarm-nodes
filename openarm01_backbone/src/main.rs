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

        tokio::try_join!(
            actions::move_arm_joints::run(node_runner.clone(), routing.clone(), token.clone()),
            actions::move_gripper::run(node_runner.clone(), routing.clone(), token.clone()),
        )?;
        Ok(())
    })
}
