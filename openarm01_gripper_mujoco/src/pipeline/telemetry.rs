// Re-emit the sim gripper's measured opening on the standardized gripper_states
// stream. The bridge sends per-finger joint positions; the opening is their sum
// (each finger holds half the aperture). The cache feeds move_gripper.

use std::sync::Arc;

use peppygen::NodeRunner;
use peppygen::emitted_topics::openarm01_gripper_state_source::v1::gripper_states;
use peppylib::TopicPublisher;
use serde::Deserialize;
use sim_bridge_core::{BoxFuture, SimBridge};
use tracing::{error, info};

use crate::config::GripperId;
use crate::state::{GripperStateLatest, SharedState};
use crate::transport::PeppylibTransport;

#[derive(Debug, Clone, Deserialize)]
struct GripperStateRaw {
    positions: Vec<f64>,
}

pub async fn run(
    runner: Arc<NodeRunner>,
    gripper_id: GripperId,
    state: Arc<SharedState>,
    transport: Arc<PeppylibTransport>,
) {
    let side = gripper_id.side_word();
    let id = gripper_id.as_u8();
    info!("telemetry: starting gripper_states pipeline (gripper_id={id} side={side})");

    // sim_node = the publisher node name configured in the bridge_extension.
    let sim_node: Arc<str> = Arc::from("sim");

    // sim_bridge_core takes a tokio_util token; cancel it on shutdown so the
    // bridge tears down before the runtime drops.
    let token = tokio_util::sync::CancellationToken::new();
    {
        let bridge_token = token.clone();
        runner.on_shutdown(async move { bridge_token.cancel() });
    }

    let gripper_state_topic: Arc<str> = Arc::from(format!("gripper_state_{side}"));

    let gs_pub = match gripper_states::declare_publisher(&runner).await {
        Ok(p) => p,
        Err(e) => return error!("declare gripper_states publisher: {e}"),
    };

    let bridge = SimBridge::new(runner.clone(), transport, token, sim_node).sim_to_os(
        gripper_state_topic,
        move |_runner, msg: GripperStateRaw| -> BoxFuture<std::result::Result<(), String>> {
            let state = state.clone();
            let publisher = gs_pub.clone();
            Box::pin(async move {
                {
                    let mut latest = state.gripper_state.lock().unwrap_or_else(|p| p.into_inner());
                    *latest = Some(GripperStateLatest {
                        positions: msg.positions.clone(),
                    });
                }
                emit_gripper_state(&publisher, id, &msg).await
            })
        },
    );

    bridge.run().await;
    info!("telemetry: gripper_states pipeline exited");
}

async fn emit_gripper_state(
    publisher: &TopicPublisher,
    gripper_id: u8,
    msg: &GripperStateRaw,
) -> std::result::Result<(), String> {
    // Opening = total aperture = sum of finger positions (each holds half). Skip
    // until the sim sends a sample so a consumer never sees a spurious 0.
    if msg.positions.is_empty() {
        return Ok(());
    }
    let opening = msg.positions.iter().sum::<f64>();
    let payload = gripper_states::build_message(gripper_id, opening).map_err(|e| e.to_string())?;
    publisher.publish(payload).await.map_err(|e| e.to_string())
}
