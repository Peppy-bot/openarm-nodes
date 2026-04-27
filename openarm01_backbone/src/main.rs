use std::sync::Arc;

use peppygen::{NodeBuilder, NodeRunner, Parameters, Result};
use sim_bridge_core::{read_bridge_config, sim_node_name};

mod bridge;
mod orchestrator;
mod startup;
mod types;

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

        // ── startup: discover which arm and gripper we are paired with ─────────
        let (arm_id, gripper_id) =
            startup::discover_ids(runner.clone(), token.clone()).await?;
        tracing::info!(
            arm_id = arm_id.as_u8(),
            gripper_id = gripper_id.as_u8(),
            "component IDs discovered"
        );

        // ── move_arm orchestration + sim bridge run concurrently ──────────────
        let orch_runner = runner.clone();
        let orch_token = token.clone();
        let bridge_token = token.clone();

        tokio::select! {
            _ = orchestrator::run(orch_runner, orch_token, arm_id) => {
                tracing::info!("orchestrator exited — cancelling bridge");
                token.cancel();
            }
            _ = bridge::build(runner, daemon, bridge_token, sim_node, &config).run() => {
                tracing::info!("bridge exited — cancelling orchestrator");
                token.cancel();
            }
        }

        Ok(())
    })
}
