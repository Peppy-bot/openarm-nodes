//! Engine-agnostic sim gripper follower: a pure relay between its two
//! gripper_link pairings. The backbone's governed opening setpoints (with the
//! operator's effort cap) forward to the sim engine's matching limb slot and
//! the engine's measured state forwards back to the backbone, stamps
//! untouched, so both peers see the conversation they would have with a real
//! counterpart. Non-finite values are dropped rather than forwarded, the same
//! guard every follower applies at ingestion.

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant, UNIX_EPOCH};

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

/// Motors per gripper: the length of the health `level` vector consumers
/// expect.
const GRIPPER_MOTORS: usize = 1;

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
                error!("gripper_setpoints receive: {e}");
                tokio::time::sleep(RECEIVE_ERROR_BACKOFF).await;
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
            Ok(()) => {
                failing = false;
                if first {
                    first = false;
                    info!("first setpoint relayed to the engine");
                }
            }
            Err(e) if !failing => {
                failing = true;
                warn!("engine gripper_setpoints publish failing, suppressing repeats: {e}");
            }
            Err(_) => {}
        }
    }
}

/// Forward the engine's measured gripper_states to the backbone, recording
/// the time of each relayed one in `relayed`: the first marks this limb's
/// physics live, and recency is what lets the health heartbeat vouch for the
/// limb.
async fn relay_states(
    runner: Arc<NodeRunner>,
    relayed: Arc<Mutex<Option<Instant>>>,
    token: CancellationToken,
) {
    let mut sub = match engine::gripper_states::subscribe(&runner).await {
        Ok(s) => s,
        Err(e) => return error!("engine gripper_states subscribe: {e}"),
    };
    let publisher = match backbone::gripper_states::declare_publisher(&runner).await {
        Ok(p) => p,
        Err(e) => return error!("declare gripper_states publisher: {e}"),
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
                error!("engine gripper_states receive: {e}");
                tokio::time::sleep(RECEIVE_ERROR_BACKOFF).await;
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
                if first {
                    first = false;
                    info!("first state relayed to the backbone");
                }
                *relayed.lock().unwrap_or_else(|e| e.into_inner()) = Some(Instant::now());
            }
            Err(e) if !failing => {
                failing = true;
                warn!("gripper_states publish failing, suppressing repeats: {e}");
            }
            Err(_) => {}
        }
    }
}

/// The side encoding shared with the real gripper, so one launcher convention
/// names a side across simulated and real stacks.
const GRIPPER_ID_LEFT: u8 = 0;
const GRIPPER_ID_RIGHT: u8 = 1;

/// Emit the "present, not sensed" motor_health heartbeat: a nominal level and
/// empty reading vectors, because the engine reports no effort or
/// temperature for this limb.
///
/// Held while `relayed` is empty or stale. Nothing is known about the limb
/// before the first engine state, and nothing current is known once states
/// stop arriving, so vouching in either case would report a limb whose
/// physics is absent as a healthy one. A held heartbeat is what lets
/// consumers age the last report out and name this producer dead.
async fn publish_health(
    runner: Arc<NodeRunner>,
    source: String,
    relayed: Arc<Mutex<Option<Instant>>>,
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
        // to call a motor silent.
        let current = relayed
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .is_some_and(|at| at.elapsed() < STATE_STALE_AFTER);
        if !current {
            continue;
        }
        let result = async {
            let ns = peppygen::clock::now_ns().map_err(|e| format!("clock not ready: {e}"))?;
            let msg = motor_health::build_message(
                UNIX_EPOCH + Duration::from_nanos(ns),
                source.clone(),
                vec![0; GRIPPER_MOTORS],
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

    NodeBuilder::new().run(|params: Parameters, node_runner| async move {
        // Names this side on the health wire: same 0 left / 1 right encoding
        // as the real gripper, refused at startup so a launcher typo cannot
        // publish a source no consumer recognises.
        assert!(
            matches!(params.gripper_id, GRIPPER_ID_LEFT | GRIPPER_ID_RIGHT),
            "gripper_id must be {GRIPPER_ID_LEFT} (left) or {GRIPPER_ID_RIGHT} (right), got {}",
            params.gripper_id
        );
        // Health stamps read the daemon-resolved clock (sim time under a
        // simulated clock), like every producer-side stamp in the stack.
        peppygen::clock::init(&node_runner).await?;
        let token = node_runner.cancellation_token().clone();
        // When the engine last relayed a state. Readiness latches on the
        // first (like the real follower's motors-enabled-and-serving gate,
        // and deliberately never unlatches: mid-session recovery is the
        // runtime's restart, not a ready flap); the health heartbeat asks
        // for recency.
        let relayed: Arc<Mutex<Option<Instant>>> = Arc::new(Mutex::new(None));
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
        // Names this side on the health wire the way the real gripper names
        // itself. Exhaustive rather than an else, so a value the assert above
        // does not cover cannot silently publish as the right gripper.
        let source = match params.gripper_id {
            GRIPPER_ID_LEFT => "left gripper",
            GRIPPER_ID_RIGHT => "right gripper",
            other => unreachable!("gripper_id {other} was refused at startup"),
        };
        let health = tokio::spawn(publish_health(
            node_runner.clone(),
            source.to_string(),
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
