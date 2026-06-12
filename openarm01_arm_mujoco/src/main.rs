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
            "starting openarm01_arm_mujoco instance={} arm_id={}",
            arm_id.instance_id(),
            arm_id.raw()
        );

        // Stamps must come from peppy's wall/sim clock (per launcher's
        // framework.use_sim_time), not the monolith's time.time() in raw payloads.
        peppygen::clock::init(&node_runner)
            .await
            .expect("peppygen::clock::init");

        // No ctrl-zeroing on exit — unlike the gripper we must NOT publish
        // ctrl=0.0: zeroing arm joint targets would command the arm into a hard
        // self-collision pose. The shutdown hooks below only stop the bridge
        // pipelines and finish an in-flight goal's completion dispatch; the arm
        // holds its last commanded pose.
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

        tokio::spawn(services::get_joint_positions::run(
            node_runner.clone(),
            shared.clone(),
            token.clone(),
        ));

        // SimBridge is peppylib-free and watches a tokio_util token; cancel it
        // from a hook (awaited before teardown) rather than a detached
        // forwarder task, which would race the runtime drop and could leave
        // the pipelines never observing cancellation.
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

        // Await an in-flight motion goal from a hook so its completion
        // dispatch (notably complete_cancelled after the token fires) is
        // guaranteed to reach the action client before the runtime tears down.
        let inflight = actions::move_arm_joints::InflightMotion::default();
        {
            let inflight = inflight.clone();
            node_runner.on_shutdown(async move { inflight.wait_idle().await });
        }
        tokio::spawn(actions::move_arm_joints::run(
            node_runner.clone(),
            arm_id,
            shared.clone(),
            token.clone(),
            handle.clone(),
            daemon.clone(),
            inflight,
        ));

        Ok(())
    })
}
