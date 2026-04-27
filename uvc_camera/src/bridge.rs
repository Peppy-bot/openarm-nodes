use std::sync::Arc;
use std::time::SystemTime;

use peppygen::NodeRunner;
use peppylib::runtime::CancellationToken;
use sim_bridge_core::{
    BridgeConfig, DaemonState, SimBridge,
    types::messages::VideoStreamMsg,
};

pub fn build(
    runner: Arc<NodeRunner>,
    daemon: DaemonState,
    token: CancellationToken,
    sim_node: Arc<str>,
    config: &BridgeConfig,
) -> SimBridge<NodeRunner> {
    let mut bridge = SimBridge::new(runner, daemon, token, sim_node);

    for pub_cfg in &config.publishers {
        match pub_cfg.type_name.as_str() {
            "rgb_camera" => {
                let topic = Arc::from(pub_cfg.topic.as_str());
                tracing::info!(topic = %topic, "registering rgb_camera pipeline");
                bridge = bridge.sim_to_os::<VideoStreamMsg, _>(
                    topic,
                    |runner, msg| {
                        Box::pin(async move {
                            use peppygen::emitted_topics::video_stream::{self, MessageHeader};
                            video_stream::emit(
                                &runner,
                                MessageHeader {
                                    stamp: SystemTime::now(),
                                    frame_id: msg.header.frame_id,
                                },
                                msg.encoding,
                                msg.width,
                                msg.height,
                                msg.frame,
                            )
                            .await
                            .map_err(|e| e.to_string())
                        })
                    },
                );
            }
            unknown => tracing::warn!("unknown publisher type '{unknown}' in config — skipped"),
        }
    }

    bridge
}
