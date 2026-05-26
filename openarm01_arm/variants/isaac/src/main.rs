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

use crate::config::ArmId;

fn main() -> Result<()> {
    tracing_subscriber::fmt().with_max_level(tracing::Level::INFO).init();

    NodeBuilder::new().run(|params: Parameters, node_runner| async move {
        let arm_id = ArmId::new(params.arm_id)
            .expect("arm_id must be 0 (left) or 1 (right)");
        let token = node_runner.cancellation_token().clone();
        info!(
            "starting openarm01_arm:isaac instance={} arm_id={}",
            arm_id.instance_id(), arm_id.0
        );

        // peppylib daemon + messenger handle shared with the action handler
        // for per-tick set_ctrl publishes. No shutdown handler — unlike the
        // gripper we must NOT publish ctrl=0.0 on exit: zeroing arm joint
        // targets would command the arm into a hard self-collision pose.
        // peppy node stop / SIGINT just cancels the token; the action loop
        // exits cleanly with the arm holding its current commanded pose.
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

        // Shared latest-joint-states cache. Written by the telemetry pipeline
        // on each incoming raw joint_states_<side>; read by move_arm_joints on
        // each feedback tick for convergence + stall detection, and by
        // get_joint_positions for one-shot service responses.
        let shared = state::new_shared();

        tokio::spawn(services::get_arm_id::run(
            node_runner.clone(), arm_id, token.clone(),
        ));

        tokio::spawn(services::get_joint_positions::run(
            node_runner.clone(), shared.clone(), token.clone(),
        ));

        // Telemetry pipelines — SimBridge gets its own cancel token from
        // node_runner internally.
        tokio::spawn(pipeline::telemetry::run(
            node_runner.clone(), arm_id, shared.clone(),
        ));

        // Cartesian move_arm is a stub-reject until backbone IK lands.
        tokio::spawn(actions::move_arm::run(
            node_runner.clone(), token.clone(),
        ));

        tokio::spawn(actions::move_arm_joints::run(
            node_runner.clone(), arm_id, shared.clone(), token.clone(),
            handle.clone(), daemon.clone(),
        ));

        Ok(())
    })
}
