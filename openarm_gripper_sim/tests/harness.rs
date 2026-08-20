//! Wire-level proof of the relay under the generated harness, in sim time:
//! the test plays both pairing peers (the backbone and the engine, mocked)
//! and the simulator driving the daemon-clock stand-in, so every decision
//! the node takes on the clock is asserted at an exact virtual instant.

use std::time::{Duration, SystemTime, UNIX_EPOCH};

use control_core::motor_health::STATE_STALE_AFTER;
use peppygen::fixtures::exposed_services::ready::is_ready;
use peppygen::fixtures::harness::{Config, Harness};
use peppygen::mock::pairings::{backbone, engine};

/// Liveness bound on every wire wait; no assertion depends on its value.
const RECV_TIMEOUT: Duration = Duration::from_secs(10);

/// The first sim instant the test drives.
const T1_NS: u64 = 1_000_000_000;

/// Exactly one staleness window past T1: the earliest instant at which a
/// T1-stamped state stops counting as recent (the gate is a strict
/// `age < STATE_STALE_AFTER`).
const T2_NS: u64 = T1_NS + STATE_STALE_AFTER.as_nanos() as u64;

fn instant(ns: u64) -> SystemTime {
    UNIX_EPOCH + Duration::from_nanos(ns)
}

fn gripper_state(stamp: SystemTime) -> engine::gripper_states::Message {
    engine::gripper_states::Message {
        timestamp: stamp,
        opening: 0.5,
        effort: 0.25,
        max_effort: 1.0,
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn sim_time_drives_relay_readiness_and_health_recency() {
    let (mut harness, mut mocks) = Harness::start_with(
        Config {
            use_sim_time: true,
            ..Default::default()
        },
        openarm_gripper_sim::setup,
    )
    .await
    .expect("harness should start in sim time");

    // Not ready before the engine's first state: this limb's physics has
    // not spoken yet.
    let ready = is_ready::poll(&harness, RECV_TIMEOUT)
        .await
        .expect("is_ready should answer");
    assert!(!ready.ready, "ready cannot precede the first engine state");

    // Setpoint leg, before the clock has any time: the relay is a pure
    // passthrough (fields and stamp untouched) with no clock read, so it
    // works while `peppygen::clock::now_ns` still errors.
    let setpoint_stamp = instant(500_000_000);
    mocks
        .pairings
        .backbone
        .gripper_setpoints
        .publish(&backbone::gripper_setpoints::Message {
            timestamp: setpoint_stamp,
            opening: 0.75,
            max_effort: 2.5,
        })
        .await
        .expect("backbone setpoint should publish");
    let relayed =
        tokio::time::timeout(RECV_TIMEOUT, mocks.pairings.engine.gripper_setpoints.next())
            .await
            .expect("the engine should receive the relayed setpoint")
            .expect("engine setpoint should decode")
            .expect("engine setpoint subscription should be open");
    assert_eq!(relayed.timestamp, setpoint_stamp, "stamps pass through unchanged");
    assert_eq!(relayed.opening, 0.75);
    assert_eq!(relayed.max_effort, 2.5);

    // A non-finite setpoint is dropped, not forwarded: the finite marker
    // sent right after it must be the next engine delivery (one ordered
    // publisher, so a forwarded NaN would have arrived first).
    mocks
        .pairings
        .backbone
        .gripper_setpoints
        .publish(&backbone::gripper_setpoints::Message {
            timestamp: setpoint_stamp,
            opening: f64::NAN,
            max_effort: 1.0,
        })
        .await
        .expect("poisoned setpoint should publish");
    mocks
        .pairings
        .backbone
        .gripper_setpoints
        .publish(&backbone::gripper_setpoints::Message {
            timestamp: setpoint_stamp,
            opening: 0.9,
            max_effort: 1.0,
        })
        .await
        .expect("marker setpoint should publish");
    let relayed =
        tokio::time::timeout(RECV_TIMEOUT, mocks.pairings.engine.gripper_setpoints.next())
            .await
            .expect("the engine should receive the marker setpoint")
            .expect("engine setpoint should decode")
            .expect("engine setpoint subscription should be open");
    assert_eq!(
        relayed.opening, 0.9,
        "the NaN setpoint must not reach the engine"
    );

    // State leg at the first driven instant: the engine speaks, stamped
    // with the same sim time the node reads, and readiness latches.
    harness.clock.tick(T1_NS).await.expect("first tick");
    mocks
        .pairings
        .engine
        .gripper_states
        .publish(&gripper_state(instant(T1_NS)))
        .await
        .expect("engine state should publish");
    let relayed_state =
        tokio::time::timeout(RECV_TIMEOUT, mocks.pairings.backbone.gripper_states.next())
            .await
            .expect("the backbone should receive the relayed state")
            .expect("backbone state should decode")
            .expect("backbone state subscription should be open");
    assert_eq!(relayed_state.timestamp, instant(T1_NS));
    assert_eq!(relayed_state.opening, 0.5);
    assert_eq!(relayed_state.effort, 0.25);
    assert_eq!(relayed_state.max_effort, 1.0);
    let ready = is_ready::poll(&harness, RECV_TIMEOUT)
        .await
        .expect("is_ready should answer");
    assert!(ready.ready, "the first relayed state marks the limb live");

    // The heartbeat vouches only now, stamped with the exact instant the
    // test drove, in the "present, not sensed" shape: a nominal level, no
    // readings.
    let health = tokio::time::timeout(
        RECV_TIMEOUT,
        harness.emitted.motor_health_motor_health.next(),
    )
    .await
    .expect("the heartbeat should start after the first state")
    .expect("motor_health should decode")
    .expect("motor_health subscription should be open");
    assert_eq!(
        health.timestamp,
        instant(T1_NS),
        "the heartbeat stamps the driven sim instant"
    );
    assert_eq!(health.level, vec![0u8; 1]);
    assert!(health.effort_fraction_rated.is_empty());
    assert!(health.effort_fraction_rated_sustained.is_empty());
    assert!(health.effort_fraction_peak.is_empty());
    assert!(health.driver_temp_c.is_empty());
    assert!(health.winding_temp_c.is_empty());

    // Recency gate at exactly one window: at T2 the T1 state is a whole
    // STATE_STALE_AFTER old, so the heartbeat holds until the engine speaks
    // again. The stamp stream proves it: a T2-stamped heartbeat can only be
    // published with a T2-fresh state in hand, so every heartbeat delivered
    // up to the first T2-stamped one must still carry T1.
    harness.clock.tick(T2_NS).await.expect("second tick");
    mocks
        .pairings
        .engine
        .gripper_states
        .publish(&gripper_state(instant(T2_NS)))
        .await
        .expect("fresh engine state should publish");
    loop {
        let health = tokio::time::timeout(
            RECV_TIMEOUT,
            harness.emitted.motor_health_motor_health.next(),
        )
        .await
        .expect("the heartbeat should resume on the fresh state")
        .expect("motor_health should decode")
        .expect("motor_health subscription should be open");
        if health.timestamp == instant(T2_NS) {
            break;
        }
        assert_eq!(
            health.timestamp,
            instant(T1_NS),
            "a heartbeat between the ticks may only carry the still-fresh T1"
        );
    }

    harness.shutdown().await.expect("clean shutdown");
}
