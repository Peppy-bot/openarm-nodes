mod actions;
mod config;
mod error;
mod pipeline;
mod services;
mod state;

use peppygen::{NodeBuilder, Parameters, Result};
use tracing::info;

use crate::config::GripperId;

fn main() -> Result<()> {
    tracing_subscriber::fmt().with_max_level(tracing::Level::INFO).init();

    NodeBuilder::new().run(|params: Parameters, node_runner| async move {
        let gripper_id = GripperId(params.gripper_id);
        let token = node_runner.cancellation_token().clone();
        info!(
            "starting openarm01_gripper:mujoco instance={} gripper_id={}",
            gripper_id.instance_id(), gripper_id.0
        );

        // Shared latest-gripper-state cache. Written by the telemetry pipeline
        // on each incoming raw gripper_state_<side>; read by move_gripper on
        // each feedback tick for convergence + stall detection.
        let shared = state::new_shared();

        // get_gripper_id service.
        tokio::spawn(services::get_gripper_id::run(
            node_runner.clone(), gripper_id, token.clone(),
        ));

        // Telemetry pipelines: subscribe to raw peppylib from robot_initializer,
        // re-emit as typed peppygen, update shared state. Wired via SimBridge
        // — which gets its own cancel token from node_runner internally.
        tokio::spawn(pipeline::telemetry::run(
            node_runner.clone(), gripper_id, shared.clone(),
        ));

        // move_gripper action: publishes set_ctrl_gripper_<side> raw peppylib,
        // reads latest gripper_state_<side> from shared cache for feedback.
        tokio::spawn(actions::move_gripper::run(
            node_runner.clone(), gripper_id, shared.clone(), token.clone(),
        ));

        Ok(())
    })
}
