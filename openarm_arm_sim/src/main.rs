//! Engine-agnostic sim arm follower: a pure relay between its two joint_link
//! pairings. The backbone's governed setpoints forward to the sim engine's
//! matching limb slot and the engine's measured state forwards back to the
//! backbone, stamps untouched, so both peers see the conversation they would
//! have with a real counterpart. Non-finite values are dropped rather than
//! forwarded, the same guard every follower applies at ingestion.

use std::sync::{Arc, Mutex};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use control_core::motor_health::{HEALTH_PERIOD, STATE_STALE_AFTER};
use peppygen::emitted_topics::motor_health::motor_health;
use peppygen::exposed_services::ready::is_ready;
use peppygen::paired_topics::{backbone, engine};
use peppygen::{NodeBuilder, NodeRunner, Parameters, Result};
use peppylib::runtime::CancellationToken;
use tracing::{error, info, warn};

/// Pause after a receive error before retrying, so a persistently broken
/// subscription cannot hot-spin the relay or flood the log.
const RECEIVE_ERROR_BACKOFF: Duration = Duration::from_millis(100);

/// Joints per arm: the length of the health `level` vector consumers expect.
const ARM_DOF: usize = 7;

fn all_finite(vecs: &[&[f64]]) -> bool {
    vecs.iter().all(|v| v.iter().all(|x| x.is_finite()))
}

/// Forward the backbone's governed joint_setpoints to the engine.
async fn relay_setpoints(runner: Arc<NodeRunner>, token: CancellationToken) {
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
            msg.stamp,
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
/// stamp of each relayed one in `relayed`: the first marks this limb's physics
/// live, and recency is what lets the health heartbeat vouch for the limb.
/// The stamp is the engine's daemon-clock capture time, the same clock the
/// heartbeat stamps with, so the recency gate holds under a simulated clock
/// that does not advance at wall rate.
async fn relay_states(
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
        let stamp = msg.stamp;
        let result = match backbone::joint_states::build_message(
            msg.stamp,
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
                *relayed.lock().unwrap_or_else(|e| e.into_inner()) = Some(stamp);
            }
            Err(e) if !failing => {
                failing = true;
                warn!("joint_states publish failing, suppressing repeats: {e}");
            }
            Err(_) => {}
        }
    }
}

/// Emit the "present, not sensed" motor_health heartbeat: nominal levels and
/// empty reading vectors, because the engine reports no effort or
/// temperature for this limb.
///
/// Held while `relayed` is empty or stale. Nothing is known about the limb
/// before the first engine state, and nothing current is known once states
/// stop arriving, so vouching in either case would report a limb whose
/// physics is absent as a healthy one. A held heartbeat is what lets
/// consumers age the last report out and name this producer dead.
/// Staleness is judged on the daemon clock, the base both the engine's
/// stamps and this heartbeat's stamps come from.
async fn publish_health(
    runner: Arc<NodeRunner>,
    relayed: Arc<Mutex<Option<SystemTime>>>,
    token: CancellationToken,
) {
    let publisher = match motor_health::declare_publisher(&runner).await {
        Ok(p) => p,
        Err(e) => return error!("declare motor_health publisher: {e}"),
    };
    let mut ticker = tokio::time::interval(HEALTH_PERIOD);
    // A starved task must resume at the cadence, not fire a catch-up burst
    // of stamps that all claim to be the current condition.
    ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
    let mut failing = false;
    loop {
        tokio::select! {
            _ = token.cancelled() => return,
            _ = ticker.tick() => {}
        }
        // Vouch only for a limb whose physics spoke recently; the doc above
        // is the reasoning. The window is the same one every follower uses
        // to call a motor silent. One clock read serves both the gate and
        // the stamp, so the two cannot disagree.
        let now = match peppygen::clock::now_ns() {
            Ok(ns) => UNIX_EPOCH + Duration::from_nanos(ns),
            Err(e) => {
                if !failing {
                    failing = true;
                    warn!("motor_health held, clock not ready, suppressing repeats: {e}");
                }
                continue;
            }
        };
        let current = relayed
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            // A stamp ahead of `now` is fresher than now, not stale.
            .is_some_and(|at| {
                now.duration_since(at)
                    .map_or(true, |age| age < STATE_STALE_AFTER)
            });
        if !current {
            continue;
        }
        let result = async {
            let msg = motor_health::build_message(
                now,
                vec![0; ARM_DOF],
                Vec::new(),
                Vec::new(),
                Vec::new(),
                Vec::new(),
                Vec::new(),
            )
            .map_err(|e| e.to_string())?;
            publisher.publish(msg).await.map_err(|e| e.to_string())
        }
        .await;
        match result {
            Ok(()) => failing = false,
            Err(e) if !failing => {
                failing = true;
                warn!("motor_health publish failing, suppressing repeats: {e}");
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
        // Health stamps read the daemon-resolved clock (sim time under a
        // simulated clock), like every producer-side stamp in the stack.
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
    })
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
