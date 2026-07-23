//! Engine-agnostic sim gripper follower: a pure relay between its two
//! gripper_link pairings. The backbone's governed opening setpoints (with the
//! operator's effort cap) forward to the sim engine's matching limb slot and
//! the engine's measured state forwards back to the backbone, stamps
//! untouched, so both peers see the conversation they would have with a real
//! counterpart. Non-finite values are dropped rather than forwarded, the same
//! guard every follower applies at ingestion.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use peppygen::exposed_services::ready::is_ready;
use peppygen::paired_topics::{backbone, engine};
use peppygen::{NodeBuilder, NodeRunner, Parameters, Result};
use peppylib::runtime::CancellationToken;
use tracing::{error, warn};

/// Forward the backbone's governed gripper_setpoints to the engine.
async fn relay_setpoints(runner: Arc<NodeRunner>, token: CancellationToken) {
    let mut sub = match backbone::gripper_setpoints::subscribe(&runner).await {
        Ok(s) => s,
        Err(e) => return error!("gripper_setpoints subscribe: {e}"),
    };
    let publisher = match engine::gripper_setpoints::declare_publisher(&runner).await {
        Ok(p) => p,
        Err(e) => return error!("declare engine gripper_setpoints publisher: {e}"),
    };
    let mut failing = false;
    loop {
        let received = tokio::select! {
            _ = token.cancelled() => return,
            received = sub.next() => received,
        };
        let msg = match received {
            Ok(Some((_, msg))) => msg,
            Ok(None) => return,
            Err(e) => {
                error!("gripper_setpoints receive: {e}");
                continue;
            }
        };
        if !msg.opening.is_finite() || !msg.max_effort.is_finite() {
            warn!("dropping non-finite gripper_setpoints");
            continue;
        }
        let result = match engine::gripper_setpoints::build_message(
            msg.stamp,
            msg.opening,
            msg.max_effort,
        ) {
            Ok(payload) => publisher.publish(payload).await.map_err(|e| e.to_string()),
            Err(e) => Err(e.to_string()),
        };
        match result {
            Ok(()) => failing = false,
            Err(e) if !failing => {
                failing = true;
                warn!("engine gripper_setpoints publish failing, suppressing repeats: {e}");
            }
            Err(_) => {}
        }
    }
}

/// Forward the engine's measured gripper_states to the backbone, latching the
/// component-ready flag on the first one: this limb's physics is live.
async fn relay_states(runner: Arc<NodeRunner>, ready: Arc<AtomicBool>, token: CancellationToken) {
    let mut sub = match engine::gripper_states::subscribe(&runner).await {
        Ok(s) => s,
        Err(e) => return error!("engine gripper_states subscribe: {e}"),
    };
    let publisher = match backbone::gripper_states::declare_publisher(&runner).await {
        Ok(p) => p,
        Err(e) => return error!("declare gripper_states publisher: {e}"),
    };
    let mut failing = false;
    loop {
        let received = tokio::select! {
            _ = token.cancelled() => return,
            received = sub.next() => received,
        };
        let msg = match received {
            Ok(Some((_, msg))) => msg,
            Ok(None) => return,
            Err(e) => {
                error!("engine gripper_states receive: {e}");
                continue;
            }
        };
        if !msg.opening.is_finite() || !msg.effort.is_finite() || !msg.max_effort.is_finite() {
            warn!("dropping non-finite gripper_states");
            continue;
        }
        let result = match backbone::gripper_states::build_message(
            msg.stamp,
            msg.opening,
            msg.effort,
            msg.max_effort,
        ) {
            Ok(payload) => publisher.publish(payload).await.map_err(|e| e.to_string()),
            Err(e) => Err(e.to_string()),
        };
        match result {
            Ok(()) => {
                failing = false;
                ready.store(true, Ordering::SeqCst);
            }
            Err(e) if !failing => {
                failing = true;
                warn!("gripper_states publish failing, suppressing repeats: {e}");
            }
            Err(_) => {}
        }
    }
}

fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .init();

    NodeBuilder::new().run(|_params: Parameters, node_runner| async move {
        let token = node_runner.cancellation_token().clone();
        // Component readiness the robot_initializer aggregates: false until the
        // first engine state has been relayed, exactly like the real follower's
        // motors-enabled-and-serving gate.
        let ready = Arc::new(AtomicBool::new(false));
        {
            let runner = node_runner.clone();
            let ready = ready.clone();
            tokio::spawn(async move {
                loop {
                    if let Err(e) = is_ready::handle_next_request(&runner, |_req| {
                        Ok(is_ready::Response::new(ready.load(Ordering::SeqCst)))
                    })
                    .await
                    {
                        error!("is_ready: {e}");
                    }
                }
            });
        }
        let setpoints = tokio::spawn(relay_setpoints(node_runner.clone(), token.clone()));
        let states = tokio::spawn(relay_states(node_runner.clone(), ready, token.clone()));
        // A dead relay leg would hold its direction silently while the node
        // reports healthy; cancel the node so the runtime restarts it.
        tokio::spawn(async move {
            tokio::select! {
                _ = setpoints => {}
                _ = states => {}
            }
            token.cancel();
        });
        Ok(())
    })
}
