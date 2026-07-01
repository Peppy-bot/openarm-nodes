mod actions;
mod config;
mod follow;
mod passthrough;
mod services;
mod state;
mod state_stream;
mod stream;
mod trajectory;

use std::sync::Arc;
use std::sync::atomic::AtomicBool;

use peppygen::{NodeBuilder, Parameters, Result};
use tokio::sync::watch;
use tracing::info;

use crate::config::{ArmId, ControlParams};

fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::INFO)
        .init();

    NodeBuilder::new().run(|params: Parameters, node_runner| async move {
        let arm_id = ArmId::new(params.arm_id).expect("arm_id must be 0 (left) or 1 (right)");
        let token = node_runner.cancellation_token().clone();
        info!(
            "starting openarm01_arm_mujoco instance={} arm_id={}",
            arm_id.instance_id(),
            arm_id.raw()
        );

        // No shutdown handler. Unlike the gripper we must NOT publish ctrl=0.0
        // on exit: zeroing arm joint targets would command the arm into a hard
        // self-collision pose. SIGINT cancels the token; the action loop exits
        // with the arm holding its last commanded pose.

        let shared = state::new_shared();
        let control = ControlParams::from_params(&params);

        tokio::spawn(services::get_arm_id::run(
            node_runner.clone(),
            arm_id,
            token.clone(),
        ));

        // Consume the sim's measured joint state (joint_states) to anchor moves
        // and re-anchor the follow chase.
        tokio::spawn(state_stream::run(
            node_runner.clone(),
            arm_id,
            shared.clone(),
            token.clone(),
        ));

        tokio::spawn(actions::move_arm::run(node_runner.clone(), token.clone()));

        // One passthrough publisher and one busy gate, shared by the move action
        // and the follow loop so only one drives the sim at a time.
        let passthrough_pub = passthrough::declare_publisher(&node_runner)
            .await
            .expect("declare passthrough publisher");
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
            passthrough_pub.clone(),
            arm_id.raw(),
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
            passthrough_pub,
            arm_id.raw(),
            busy,
            control,
        ));

        Ok(())
    })
}
