mod actions;
mod config;
mod follow;
mod passthrough;
mod state;
mod state_stream;
mod stream;

use std::sync::Arc;
use std::sync::atomic::AtomicBool;

use openarm_description::HardwareVersion;
use peppygen::{NodeBuilder, Parameters, Result};
use tokio::sync::watch;
use tracing::info;

use crate::config::{ControlParams, GripperId};

fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::INFO)
        .init();

    NodeBuilder::new().run(|params: Parameters, node_runner| async move {
        let gripper_id =
            GripperId::new(params.gripper_id).expect("gripper_id must be 0 (left) or 1 (right)");
        // The hardware generation sets the jaw's full-open width; everything on
        // the wire stays in aperture meters (the sim maps them onto its fingers).
        let version: HardwareVersion = params
            .hardware_version
            .parse()
            .unwrap_or_else(|e| panic!("hardware_version: {e}"));
        let open_m = version.jaw_open_m();
        let token = node_runner.cancellation_token().clone();
        info!(
            "starting openarm_gripper_isaac instance={} gripper_id={}",
            gripper_id.instance_id(),
            gripper_id.as_u8()
        );

        peppygen::clock::init(&node_runner)
            .await
            .expect("peppygen::clock::init");

        let shared = state::new_shared();

        // Consume the sim's measured opening (gripper_states) to feed the move
        // action's convergence/stall feedback. Supervised: if the consumer ever
        // exits, whether a clean close on shutdown or an unexpected error, the
        // feedback path is dead, so cancel the node to restart it rather than
        // leaving it healthy but inert.
        {
            let runner = node_runner.clone();
            let shared = shared.clone();
            let token = token.clone();
            tokio::spawn(async move {
                state_stream::run(runner, gripper_id, shared, token.clone()).await;
                token.cancel();
            });
        }

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
        // Supervised like the state stream: a dead command consumer leaves the
        // gripper unresponsive to streamed openings.
        {
            let runner = node_runner.clone();
            let token = token.clone();
            tokio::spawn(async move {
                stream::run(runner, cmd_tx, token.clone()).await;
                token.cancel();
            });
        }
        tokio::spawn(follow::run(
            passthrough_pub.clone(),
            gripper_id.as_u8(),
            open_m,
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
            open_m,
            busy,
        ));

        Ok(())
    })
}
