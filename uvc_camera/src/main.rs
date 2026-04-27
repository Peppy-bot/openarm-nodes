use std::sync::Arc;

use peppygen::{NodeBuilder, NodeRunner, Parameters, Result};
use sim_bridge_core::{read_bridge_config, sim_node_name};

mod bridge;
mod startup;

fn main() -> Result<()> {
    NodeBuilder::new().run(|_args: Parameters, runner: Arc<NodeRunner>| async move {
        let token = runner.cancellation_token();

        let config = read_bridge_config().map_err(|e| {
            tracing::error!("failed to load sim_bridge config: {e}");
            e.to_string()
        })?;
        let sim_node = sim_node_name(&config);

        let daemon = startup::read_daemon_state().map_err(|e| {
            tracing::error!("failed to read daemon state: {e}");
            e
        })?;

        bridge::build(runner, daemon, token, sim_node, &config)
            .run()
            .await;

        Ok(())
    })
}
