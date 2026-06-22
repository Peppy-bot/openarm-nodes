mod actions;
mod config;
mod follow;
mod pipeline;
mod services;
mod setctrl;
mod state;
mod stream;
mod trajectory;
mod transport;

use std::sync::Arc;
use std::sync::atomic::AtomicBool;

use peppygen::{NodeBuilder, Parameters, Result};
use peppylib::MessengerHandle;
use sim_bridge_core::DaemonState;
use tokio::sync::watch;
use tracing::info;

use crate::config::{ArmId, ControlParams};

fn main() -> Result<()> {
    tracing_subscriber::fmt().with_max_level(tracing::Level::INFO).init();

    NodeBuilder::new().run(|params: Parameters, node_runner| async move {
        let arm_id = ArmId::new(params.arm_id).expect("arm_id must be 0 (left) or 1 (right)");
        let token = node_runner.cancellation_token().clone();
        info!(
            "starting openarm01_arm_mujoco instance={} arm_id={}",
            arm_id.instance_id(),
            arm_id.raw()
        );

        // No shutdown handler — unlike the gripper we must NOT publish ctrl=0.0
        // on exit: zeroing arm joint targets would command the arm into a hard
        // self-collision pose. SIGINT cancels the token; the action loop exits
        // with the arm holding its last commanded pose.
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
        let control = ControlParams::from_params(&params);

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

        // One set_ctrl publisher and one busy gate, shared by the move action and
        // the follow loop so only one drives the sim at a time.
        let side = arm_id.side_word();
        let set_ctrl_pub = setctrl::declare_publisher(&handle, &daemon, side)
            .await
            .expect("declare set_ctrl publisher");
        let actuator_names = Arc::new(setctrl::actuator_names(side));
        let busy = Arc::new(AtomicBool::new(false));

        // Stream listener -> follow loop: the listener keeps the latest streamed
        // setpoint, the follow loop drives it between moves.
        let (cmd_tx, cmd_rx) = watch::channel(None);
        tokio::spawn(stream::run(
            node_runner.clone(),
            arm_id,
            cmd_tx,
            token.clone(),
        ));
        tokio::spawn(follow::run(
            set_ctrl_pub.clone(),
            actuator_names.clone(),
            busy.clone(),
            shared.clone(),
            cmd_rx,
            control,
            token.clone(),
        ));

        tokio::spawn(actions::move_arm_joints::run(
            node_runner.clone(),
            shared.clone(),
            token.clone(),
            set_ctrl_pub,
            actuator_names,
            busy,
            control,
        ));

        Ok(())
    })
}
