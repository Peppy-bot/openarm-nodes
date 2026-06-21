mod actions;
mod config;
mod pipeline;
mod services;
mod state;
mod trajectory;
mod transport;

use std::sync::Arc;

use peppygen::{NodeBuilder, Parameters, Result};
use peppylib::MessengerHandle;
use sim_bridge_core::DaemonState;
use tracing::info;

use crate::config::ArmId;

fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(tracing_subscriber::EnvFilter::from_default_env())
        .init();

    NodeBuilder::new().run(|params: Parameters, node_runner| async move {
        let arm_id = ArmId::new(params.arm_id).expect("arm_id must be 0 (left) or 1 (right)");
        let token = node_runner.cancellation_token().clone();
        info!(
            "starting openarm01_arm_isaac instance={} arm_id={}",
            arm_id.instance_id(),
            arm_id.raw()
        );

        let daemon_info = peppylib::info(&node_runner, None)
            .await
            .expect("peppylib::info");
        let daemon = DaemonState {
            core_node_name: daemon_info.core_node_name,
            messaging_port: daemon_info.messaging_port,
        };
        let handle = Arc::new(
            MessengerHandle::from_host_port("localhost", daemon.messaging_port)
                .await
                .expect("peppylib connect"),
        );

        let shared = state::new_shared();

        tokio::spawn(services::get_arm_id::run(
            node_runner.clone(),
            arm_id,
            token.clone(),
        ));

        // SimBridge is peppylib-free; this node hands it a peppylib-backed
        // transport (telemetry::run bridges the cancel token internally).
        let transport = transport::PeppylibTransport::new(daemon.clone());
        tokio::spawn(pipeline::telemetry::run(
            node_runner.clone(),
            arm_id,
            shared.clone(),
            transport,
        ));

        tokio::spawn(actions::move_arm::run(node_runner.clone(), token.clone()));

        tokio::spawn(actions::move_arm_joints::run(
            node_runner.clone(),
            arm_id,
            shared.clone(),
            token.clone(),
            handle.clone(),
            daemon.clone(),
        ));

        Ok(())
    })
}
