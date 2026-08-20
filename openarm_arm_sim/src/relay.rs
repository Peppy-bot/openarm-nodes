//! The two relay legs: the backbone's governed setpoints down to the engine,
//! the engine's measured states back up to the backbone. Pure passthrough,
//! timestamps untouched, non-finite messages dropped at ingestion.

use std::sync::{Arc, Mutex};
use std::time::{Duration, SystemTime};

use peppygen::NodeRunner;
use peppygen::paired_topics::{backbone, engine};
use peppylib::runtime::CancellationToken;
use tracing::{error, info, warn};

/// Pause after a receive error before retrying, so a persistently broken
/// subscription cannot hot-spin the relay or flood the log.
const RECEIVE_ERROR_BACKOFF: Duration = Duration::from_millis(100);

fn all_finite(vecs: &[&[f64]]) -> bool {
    vecs.iter().all(|v| v.iter().all(|x| x.is_finite()))
}

/// Forward the backbone's governed joint_setpoints to the engine.
pub(crate) async fn relay_setpoints(runner: Arc<NodeRunner>, token: CancellationToken) {
    let mut sub = match backbone::joint_setpoints::subscribe(&runner).await {
        Ok(s) => s,
        Err(e) => return error!("joint_setpoints subscribe: {e}"),
    };
    let publisher = match engine::joint_setpoints::declare_publisher(&runner).await {
        Ok(p) => p,
        Err(e) => return error!("declare engine joint_setpoints publisher: {e}"),
    };
    let mut failing = false;
    let mut first = true;
    loop {
        let received = tokio::select! {
            _ = token.cancelled() => return,
            received = sub.next() => received,
        };
        let msg = match received {
            Ok(Some((_, msg))) => msg,
            Ok(None) => return,
            Err(e) => {
                error!("joint_setpoints receive: {e}");
                tokio::time::sleep(RECEIVE_ERROR_BACKOFF).await;
                continue;
            }
        };
        if !all_finite(&[&msg.positions, &msg.velocities, &msg.efforts]) {
            warn!("dropping non-finite joint_setpoints");
            continue;
        }
        let result = match engine::joint_setpoints::build_message(
            msg.timestamp,
            msg.positions,
            msg.velocities,
            msg.efforts,
        ) {
            Ok(payload) => publisher.publish(payload).await.map_err(|e| e.to_string()),
            Err(e) => Err(e.to_string()),
        };
        match result {
            Ok(()) => {
                failing = false;
                if first {
                    first = false;
                    info!("first setpoint relayed to the engine");
                }
            }
            Err(e) if !failing => {
                failing = true;
                warn!("engine joint_setpoints publish failing, suppressing repeats: {e}");
            }
            Err(_) => {}
        }
    }
}

/// Forward the engine's measured joint_states to the backbone, recording the
/// timestamp of each relayed one in `relayed`: the first marks this limb's physics
/// live, and recency is what lets the health heartbeat vouch for the limb.
/// The timestamp is the engine's daemon-clock capture time, the same clock the
/// heartbeat stamps with, so the recency gate holds under a simulated clock
/// that does not advance at wall rate.
pub(crate) async fn relay_states(
    runner: Arc<NodeRunner>,
    relayed: Arc<Mutex<Option<SystemTime>>>,
    token: CancellationToken,
) {
    let mut sub = match engine::joint_states::subscribe(&runner).await {
        Ok(s) => s,
        Err(e) => return error!("engine joint_states subscribe: {e}"),
    };
    let publisher = match backbone::joint_states::declare_publisher(&runner).await {
        Ok(p) => p,
        Err(e) => return error!("declare joint_states publisher: {e}"),
    };
    let mut failing = false;
    let mut first = true;
    loop {
        let received = tokio::select! {
            _ = token.cancelled() => return,
            received = sub.next() => received,
        };
        let msg = match received {
            Ok(Some((_, msg))) => msg,
            Ok(None) => return,
            Err(e) => {
                error!("engine joint_states receive: {e}");
                tokio::time::sleep(RECEIVE_ERROR_BACKOFF).await;
                continue;
            }
        };
        if !all_finite(&[&msg.positions, &msg.velocities, &msg.efforts]) {
            warn!("dropping non-finite joint_states");
            continue;
        }
        let timestamp = msg.timestamp;
        let result = match backbone::joint_states::build_message(
            msg.timestamp,
            msg.positions,
            msg.velocities,
            msg.efforts,
        ) {
            Ok(payload) => publisher.publish(payload).await.map_err(|e| e.to_string()),
            Err(e) => Err(e.to_string()),
        };
        match result {
            Ok(()) => {
                failing = false;
                if first {
                    first = false;
                    info!("first state relayed to the backbone");
                }
                *relayed.lock().unwrap_or_else(|e| e.into_inner()) = Some(timestamp);
            }
            Err(e) if !failing => {
                failing = true;
                warn!("joint_states publish failing, suppressing repeats: {e}");
            }
            Err(_) => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn finiteness_guard_covers_every_vector() {
        assert!(all_finite(&[&[0.1, 0.2], &[], &[1.0]]));
        assert!(!all_finite(&[&[0.1], &[f64::NAN]]));
        assert!(!all_finite(&[&[f64::INFINITY], &[0.0]]));
    }
}
