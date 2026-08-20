//! The motor_health heartbeat: vouches for the limb only while the engine's
//! relayed state is recent on the daemon clock.

use std::sync::{Arc, Mutex};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use control_core::motor_health::{HEALTH_PERIOD, STATE_STALE_AFTER};
use peppygen::NodeRunner;
use peppygen::emitted_topics::motor_health::motor_health;
use peppylib::runtime::CancellationToken;
use tracing::{error, warn};

/// Joints per arm: the length of the health `level` vector consumers expect.
const ARM_DOF: usize = 7;

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
/// timestamps and this heartbeat's timestamps come from.
pub(crate) async fn publish_health(
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
    // of timestamps that all claim to be the current condition.
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
        // the timestamp, so the two cannot disagree.
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
            // A timestamp ahead of `now` is fresher than now, not stale.
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
