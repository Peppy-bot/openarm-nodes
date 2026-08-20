//! The node's bringup: bind the clock, answer readiness, and supervise the
//! relay legs and the heartbeat.

use std::sync::{Arc, Mutex};
use std::time::SystemTime;

use peppygen::exposed_services::ready::is_ready;
use peppygen::{NodeRunner, Parameters, Result};
use tracing::error;

use crate::health::publish_health;
use crate::relay::{relay_setpoints, relay_states};

/// The node's setup: binds the clock, answers readiness, and spawns the two
/// relay legs plus the health heartbeat, supervised so any leg's exit
/// cancels the node.
pub async fn setup(_params: Parameters, node_runner: Arc<NodeRunner>) -> Result<()> {
    // Health timestamps read the daemon-resolved clock (sim time under a
    // simulated clock), like every producer-side timestamp in the stack.
    peppygen::clock::init(&node_runner).await?;
    let token = node_runner.cancellation_token().clone();
    // When the engine last relayed a state. Readiness latches on the
    // first (like the real follower's motors-enabled-and-serving gate,
    // and deliberately never unlatches: mid-session recovery is the
    // runtime's restart, not a ready flap); the health heartbeat asks
    // for recency.
    let relayed: Arc<Mutex<Option<SystemTime>>> = Arc::new(Mutex::new(None));
    {
        let runner = node_runner.clone();
        let relayed = relayed.clone();
        tokio::spawn(async move {
            loop {
                if let Err(e) = is_ready::handle_next_request(&runner, |_req| {
                    Ok(is_ready::Response::new(
                        relayed.lock().unwrap_or_else(|e| e.into_inner()).is_some(),
                    ))
                })
                .await
                {
                    error!("is_ready: {e}");
                }
            }
        });
    }
    let setpoints = tokio::spawn(relay_setpoints(node_runner.clone(), token.clone()));
    let health_relayed = relayed.clone();
    let states = tokio::spawn(relay_states(node_runner.clone(), relayed, token.clone()));
    let health = tokio::spawn(publish_health(
        node_runner.clone(),
        health_relayed,
        token.clone(),
    ));
    // A dead relay leg or heartbeat would hold its part silently while
    // the node reports healthy; cancel the node so the runtime restarts it.
    tokio::spawn(async move {
        tokio::select! {
            _ = setpoints => {}
            _ = states => {}
            _ = health => {}
        }
        token.cancel();
    });
    Ok(())
}
