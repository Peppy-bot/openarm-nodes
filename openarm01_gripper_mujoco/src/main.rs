mod actions;
mod config;
mod pipeline;
mod services;
mod state;

use std::sync::Arc;

use peppygen::{NodeBuilder, Parameters, Result};
use peppylib::MessengerHandle;
use sim_bridge_core::DaemonState;
use tracing::info;

use crate::config::GripperId;

fn main() -> Result<()> {
    tracing_subscriber::fmt().with_max_level(tracing::Level::INFO).init();

    NodeBuilder::new().run(|params: Parameters, node_runner| async move {
        let gripper_id = GripperId::new(params.gripper_id)
            .expect("gripper_id must be 0 (left) or 1 (right)");
        let token = node_runner.cancellation_token().clone();
        info!(
            "starting openarm01_gripper:mujoco instance={} gripper_id={}",
            gripper_id.instance_id(), gripper_id.as_u8()
        );

        // peppylib daemon + messenger handle shared between the action
        // handler (per-tick set_ctrl publish) and the shutdown handler
        // (ctrl=0.0 on SIGINT/SIGTERM).
        let daemon_info = peppylib::info(&node_runner, None).await
            .expect("peppylib::info");
        let daemon = DaemonState {
            core_node_name: daemon_info.core_node_name,
            messaging_port: daemon_info.messaging_port,
        };
        let handle = Arc::new(
            MessengerHandle::from_host_port("localhost", daemon.messaging_port).await
                .expect("peppylib connect")
        );

        // Shared latest-gripper-state cache. Written by the telemetry pipeline
        // on each incoming raw gripper_state_<side>; read by move_gripper on
        // each feedback tick for convergence + stall detection.
        let shared = state::new_shared();

        tokio::spawn(services::get_gripper_id::run(
            node_runner.clone(), gripper_id, token.clone(),
        ));

        // Telemetry pipelines — SimBridge gets its own cancel token from
        // node_runner internally.
        tokio::spawn(pipeline::telemetry::run(
            node_runner.clone(), gripper_id, shared.clone(),
        ));

        tokio::spawn(actions::move_gripper::run(
            node_runner.clone(), gripper_id, shared.clone(), token.clone(),
            handle.clone(), daemon.clone(),
        ));

        tokio::spawn(actions::move_gripper::shutdown_handler(
            handle.clone(), daemon.clone(), gripper_id, token.clone(),
        ));

        Ok(())
    })
}
