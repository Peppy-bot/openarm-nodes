// Peppylib-backed RawTransport for sim_bridge_core. The shared lib never
// links peppylib (it is generated per node by `peppy node sync`), so each sim
// node supplies this adapter and hands it to SimBridge.

use std::sync::Arc;

use peppylib::config::QoSProfile;
use peppylib::messaging::{ConsumerFilter, SenderTarget};
use peppylib::{MessengerHandle, Payload, TopicMessenger};
use sim_bridge_core::{DaemonState, RawQoS, RawSubscription, RawTransport, TransportFuture};

// Raw sim-bridge topics publish under this off-bus placeholder identity
// (see node-contracts § "Internal sim-bridge topics").
const BRIDGE_NODE_NAME: &str = "sim_bridge";

fn to_qos(qos: RawQoS) -> QoSProfile {
    match qos {
        RawQoS::Standard => QoSProfile::Standard,
        RawQoS::SensorData => QoSProfile::SensorData,
    }
}

pub struct PeppylibTransport {
    daemon: DaemonState,
    // Cached emit connection; dropped on failure so the next emit reconnects.
    handle: tokio::sync::Mutex<Option<MessengerHandle>>,
}

impl PeppylibTransport {
    pub fn new(daemon: DaemonState) -> Arc<Self> {
        Arc::new(Self {
            daemon,
            handle: tokio::sync::Mutex::new(None),
        })
    }
}

pub struct PeppylibSubscription {
    // The connection must outlive the subscription or the stream closes.
    _handle: MessengerHandle,
    sub: peppylib::messaging::Subscription,
}

impl RawSubscription for PeppylibSubscription {
    fn next(&mut self) -> TransportFuture<'_, Option<Vec<u8>>> {
        Box::pin(async move {
            self.sub
                .on_next_message()
                .await
                .map(|m| m.payload().as_ref().to_vec())
        })
    }
}

impl RawTransport for PeppylibTransport {
    type Subscription = PeppylibSubscription;

    fn subscribe<'a>(
        &'a self,
        instance_id: &'a str,
        source_node: &'a str,
        topic: &'a str,
        qos: RawQoS,
    ) -> TransportFuture<'a, std::result::Result<Self::Subscription, String>> {
        Box::pin(async move {
            let handle = MessengerHandle::from_host_port("localhost", self.daemon.messaging_port)
                .await
                .map_err(|e| format!("connect: {e}"))?;
            let target = SenderTarget::node(source_node, "v1")
                .map_err(|e| format!("invalid source target '{source_node}': {e}"))?;
            let sub = TopicMessenger::subscribe(
                &handle,
                &self.daemon.core_node_name,
                instance_id,
                Some(target),
                false,
                topic,
                None,
                &ConsumerFilter::Any,
                to_qos(qos),
            )
            .await
            .map_err(|e| e.to_string())?;
            Ok(PeppylibSubscription {
                _handle: handle,
                sub,
            })
        })
    }

    fn emit<'a>(
        &'a self,
        instance_id: &'a str,
        topic: &'a str,
        qos: RawQoS,
        payload: Vec<u8>,
    ) -> TransportFuture<'a, std::result::Result<(), String>> {
        Box::pin(async move {
            let mut guard = self.handle.lock().await;
            if guard.is_none() {
                *guard = Some(
                    MessengerHandle::from_host_port("localhost", self.daemon.messaging_port)
                        .await
                        .map_err(|e| format!("connect: {e}"))?,
                );
            }
            let handle = guard.as_ref().expect("cached handle just set");
            let target = SenderTarget::node(BRIDGE_NODE_NAME, "v1")
                .map_err(|e| format!("invalid bridge target: {e}"))?;
            if let Err(e) = TopicMessenger::emit(
                handle,
                &self.daemon.core_node_name,
                instance_id,
                target,
                topic,
                to_qos(qos),
                Payload::from(payload),
            )
            .await
            {
                *guard = None;
                return Err(e.to_string());
            }
            Ok(())
        })
    }
}
