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
        // Setup failures return Err (not panic) so the runtime's setup-error
        // path runs: token cancelled, hooks registered so far still awaited.
        let arm_id = ArmId::new(params.arm_id)
            .map_err(|e| peppylib::PeppyError::Io(std::io::Error::other(e)))?;
        let token = node_runner.cancellation_token().clone();
        info!(
            "starting openarm01_arm_isaac instance={} arm_id={}",
            arm_id.instance_id(),
            arm_id.raw()
        );

        peppygen::clock::init(&node_runner).await?;

        let daemon_info = peppylib::info(&node_runner, None).await?;
        let daemon = DaemonState {
            core_node_name: daemon_info.core_node_name,
            messaging_port: daemon_info.messaging_port,
        };
        let handle =
            Arc::new(MessengerHandle::from_host_port("localhost", daemon.messaging_port).await?);

        let shared = state::new_shared();

        tokio::spawn(services::get_arm_id::run(
            node_runner.clone(),
            arm_id,
            token.clone(),
        ));

        tokio::spawn(services::get_joint_positions::run(
            node_runner.clone(),
            shared.clone(),
            token.clone(),
        ));

        // SimBridge is peppylib-free; this node hands it a peppylib-backed
        // transport and a tokio_util token its pipelines select on. Cancel
        // that token from a shutdown hook — awaited before the runtime tears
        // down — rather than from a detached forwarder task, which is not
        // guaranteed to be polled again once the node token fires.
        let bridge_token = tokio_util::sync::CancellationToken::new();
        {
            let bridge_token = bridge_token.clone();
            node_runner.on_shutdown(async move { bridge_token.cancel() });
        }
        let transport = transport::PeppylibTransport::new(daemon.clone());
        tokio::spawn(pipeline::telemetry::run(
            node_runner.clone(),
            arm_id,
            shared.clone(),
            transport,
            bridge_token,
        ));

        tokio::spawn(actions::move_arm::run(node_runner.clone(), token.clone()));

        // Awaited by a shutdown hook so an interrupted motion still delivers
        // its terminal status (complete_cancelled) to the caller; this hook
        // runs before the bridge-token one above (reverse registration order),
        // while the rest of the node is still up.
        let inflight = actions::move_arm_joints::InflightMotion::default();
        let accept_loop = tokio::spawn(actions::move_arm_joints::run(
            node_runner.clone(),
            arm_id,
            shared.clone(),
            token.clone(),
            handle.clone(),
            daemon.clone(),
            inflight.clone(),
        ));
        node_runner.on_shutdown(async move {
            // The accept loop registers an accepted goal's motion task into
            // the slot before re-entering its select, so the slot is only
            // final once the loop has exited: join it first, then drain.
            // Otherwise a goal accepted just as the token fired could be
            // registered after wait_idle's take and never be awaited.
            let _ = accept_loop.await;
            inflight.wait_idle().await;
        });

        Ok(())
    })
}
