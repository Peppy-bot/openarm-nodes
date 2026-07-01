mod actions;
mod config;
mod follow;
mod passthrough;
mod state;
mod state_stream;
mod stream;

use std::sync::Arc;
use std::sync::atomic::AtomicBool;

use peppygen::{NodeBuilder, Parameters, Result};
use tokio::sync::watch;
use tracing::info;

use crate::config::{ApertureMap, ControlParams, GripperId};

fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::INFO)
        .init();

    NodeBuilder::new().run(|params: Parameters, node_runner| async move {
        let gripper_id =
            GripperId::new(params.gripper_id).expect("gripper_id must be 0 (left) or 1 (right)");
        // Prismatic (v1) vs revolute (v2) finger geometry: maps the jaw opening
        // (m) on the shared gripper interface to the sim passthrough value.
        let map = ApertureMap::for_version(&params.hardware_version, gripper_id);
        let token = node_runner.cancellation_token().clone();
        info!(
            "starting openarm_gripper_mujoco instance={} gripper_id={}",
            gripper_id.instance_id(),
            gripper_id.as_u8()
        );

        peppygen::clock::init(&node_runner)
            .await
            .expect("peppygen::clock::init");

        let shared = state::new_shared();

        // Consume the sim's measured opening (gripper_states) to feed the move
        // action's convergence/stall feedback.
        tokio::spawn(state_stream::run(
            node_runner.clone(),
            gripper_id,
            map,
            shared.clone(),
            token.clone(),
        ));

        // One passthrough publisher and one busy gate, shared by the move action
        // and the follow loop so only one drives the sim at a time.
        let passthrough_pub = passthrough::declare_publisher(&node_runner)
            .await
            .expect("declare passthrough publisher");
        let busy = Arc::new(AtomicBool::new(false));
        let control = ControlParams::from_params(&params);

        // Stream listener -> follow loop: the listener keeps the latest streamed
        // opening, the follow loop drives it between moves.
        let (cmd_tx, cmd_rx) = watch::channel(None);
        tokio::spawn(stream::run(
            node_runner.clone(),
            gripper_id,
            cmd_tx,
            token.clone(),
        ));
        tokio::spawn(follow::run(
            passthrough_pub.clone(),
            gripper_id.as_u8(),
            map,
            busy.clone(),
            cmd_rx,
            control,
            token.clone(),
        ));

        tokio::spawn(actions::move_gripper::run(
            node_runner.clone(),
            shared.clone(),
            token.clone(),
            passthrough_pub,
            gripper_id.as_u8(),
            map,
            busy,
        ));

        Ok(())
    })
}
