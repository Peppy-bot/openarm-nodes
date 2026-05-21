mod actions;
mod config;
mod error;
mod pipeline;
mod services;

use peppygen::{NodeBuilder, Parameters, Result};
use tracing::info;

use crate::config::GripperId;

fn main() -> Result<()> {
    tracing_subscriber::fmt().with_max_level(tracing::Level::INFO).init();

    NodeBuilder::new().run(|params: Parameters, node_runner| async move {
        let gripper_id = GripperId(params.gripper_id);
        info!(
            "starting openarm01_gripper:mujoco instance={} gripper_id={}",
            gripper_id.instance_id(), gripper_id.0
        );

        // get_gripper_id service.
        tokio::spawn(services::get_gripper_id::run(node_runner.clone(), gripper_id));

        // move_gripper action — publishes set_ctrl_gripper_<side> raw peppylib
        // each tick, reads latest gripper_state_<side> for feedback. Full
        // implementation pending sim_bridge_core wiring + shared-state design.
        tokio::spawn(actions::move_gripper::run(node_runner.clone(), gripper_id));

        // Telemetry pipelines: subscribe to raw peppylib from robot_initializer,
        // re-emit as typed peppygen on the 8 contract topics. Wired via SimBridge.
        tokio::spawn(pipeline::telemetry::run(node_runner.clone(), gripper_id));

        Ok(())
    })
}
