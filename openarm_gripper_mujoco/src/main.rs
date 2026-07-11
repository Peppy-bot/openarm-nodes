mod config;
mod follow;
mod passthrough;
mod state_stream;
mod stream;

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
            "starting openarm_gripper_mujoco instance={} gripper_id={}",
            gripper_id.instance_id(),
            gripper_id.as_u8()
        );

        peppygen::clock::init(&node_runner)
            .await
            .expect("peppygen::clock::init");

        // Relay the sim's measured opening (gripper_states) to the paired
        // backbone. Supervised: if the consumer ever exits, whether a clean
        // close on shutdown or an unexpected error, the state relay is dead, so
        // cancel the node to restart it rather than leaving it healthy but inert.
        {
            let runner = node_runner.clone();
            let token = token.clone();
            tokio::spawn(async move {
                state_stream::run(runner, gripper_id, token.clone()).await;
                token.cancel();
            });
        }

        // The passthrough publisher the follow loop drives the sim through.
        let passthrough_pub = passthrough::declare_publisher(&node_runner)
            .await
            .expect("declare passthrough publisher");
        let control = ControlParams::from_params(&params);

        // Stream listener -> follow loop: the listener keeps the latest streamed
        // opening, the follow loop drives the sim toward it.
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
            passthrough_pub,
            gripper_id.as_u8(),
            open_m,
            cmd_rx,
            control,
            token.clone(),
        ));

        Ok(())
    })
}
