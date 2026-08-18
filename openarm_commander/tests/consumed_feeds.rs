//! Consumed-topic flow with one producer bound per zero_or_more contract slot:
//! the backbone mock's `collision_status` must surface as the panel's
//! proximity readout, the alerts mock's alert must render in the alerts list,
//! and both contract slots must report as bound.
//!
//! motor_health is bound too, but the node attributes reports by the producing
//! instance's name (`classify` wants left/right + arm/grip tokens) and the
//! harness pins the generated id `mock-motor_health-0`, so its reports are
//! dropped at the parse boundary; the tail of this test pins that behavior.
//!
//! One booting test per binary: `ui::init_limits` is once-per-process.

mod helpers;

use std::time::{Duration, SystemTime};

use peppygen::consumed_topics::alerts::alerts;
use peppygen::consumed_topics::backbone::collision_status;
use peppygen::consumed_topics::motor_health::motor_health;
use peppygen::fixtures::harness::{Config, Harness};

const PANEL_PORT: u16 = 18633;

fn proximity_msg() -> collision_status::Message {
    collision_status::Message {
        distance: 0.0123,
        link_a: "left_link4".to_string(),
        link_b: "right_link4".to_string(),
        throttled: true,
        stopped: false,
    }
}

fn alert_msg() -> alerts::Message {
    alerts::Message {
        timestamp: SystemTime::now(),
        source: "left arm j2".to_string(),
        kind: "motor_overload".to_string(),
        severity: 2,
        message: "holding 93% of rated torque".to_string(),
    }
}

/// A well-formed 7-motor arm report; only its producer's instance name keeps
/// it off the panel.
fn arm_health_msg() -> motor_health::Message {
    motor_health::Message {
        timestamp: SystemTime::now(),
        level: vec![0; 7],
        effort_fraction_rated: vec![0.4; 7],
        effort_fraction_rated_sustained: vec![0.3; 7],
        effort_fraction_peak: vec![0.2; 7],
        driver_temp_c: vec![41.0; 7],
        winding_temp_c: vec![37.0; 7],
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn consumed_topics_surface_on_the_panel() -> peppygen::Result<()> {
    helpers::set_panel_env(PANEL_PORT);
    let (harness, mocks) = Harness::start_with(
        Config {
            parameters: Some(helpers::test_parameters()),
            motor_health_instances: 1,
            alerts_instances: 1,
            ..Config::default()
        },
        openarm_commander::setup,
    )
    .await?;

    // Slot binding is resolved once at owner start, so the very first
    // snapshot must already report both contract slots as wired.
    let mut ws = helpers::WsClient::connect(PANEL_PORT).await;
    let first = ws
        .snapshot_until(Duration::from_secs(10), "first snapshot", |_| true)
        .await;
    assert_eq!(first["health"]["bound"], true);
    assert_eq!(first["alerts_bound"], true);

    // collision_status -> proximity readout. The readout goes stale 500 ms
    // after the last report, so re-publish each snapshot pass (~100 ms), as
    // the backbone's ~20 Hz stream would.
    let deadline = tokio::time::Instant::now() + Duration::from_secs(10);
    let snapshot = loop {
        mocks
            .deps
            .backbone
            .collision_status
            .publish(&proximity_msg())
            .await?;
        let snapshot = ws
            .snapshot_until(Duration::from_secs(5), "snapshot pass", |_| true)
            .await;
        if !snapshot["proximity"].is_null() {
            break snapshot;
        }
        assert!(
            tokio::time::Instant::now() < deadline,
            "proximity never rendered; last snapshot: {snapshot}"
        );
    };
    let proximity = &snapshot["proximity"];
    assert_eq!(proximity["distance"].as_f64(), Some(0.0123));
    assert_eq!(proximity["link_a"], "left_link4");
    assert_eq!(proximity["link_b"], "right_link4");
    assert_eq!(proximity["throttled"], true);
    assert_eq!(proximity["stopped"], false);

    // alerts -> the alerts list, attributed and severity-tagged. Re-publish
    // with a fresh timestamp per pass so the entry cannot age out mid-poll.
    let deadline = tokio::time::Instant::now() + Duration::from_secs(10);
    let snapshot = loop {
        mocks.deps.alerts[0].alerts.publish(&alert_msg()).await?;
        let snapshot = ws
            .snapshot_until(Duration::from_secs(5), "snapshot pass", |_| true)
            .await;
        if snapshot["alerts"].as_array().is_some_and(|a| !a.is_empty()) {
            break snapshot;
        }
        assert!(
            tokio::time::Instant::now() < deadline,
            "the alert never rendered; last snapshot: {snapshot}"
        );
    };
    let alert = &snapshot["alerts"][0];
    assert_eq!(alert["source"], "left arm j2");
    assert_eq!(alert["severity"], 2);
    assert_eq!(alert["message"], "holding 93% of rated torque");

    // motor_health: bound, but the generated mock id `mock-motor_health-0`
    // names neither a side nor a kind, so the node's classifier drops every
    // report and no side may ever render live rows (bounded check across
    // ~10 snapshot passes while reports keep arriving).
    for _ in 0..10 {
        mocks.deps.motor_health[0]
            .motor_health
            .publish(&arm_health_msg())
            .await?;
        let snapshot = ws
            .snapshot_until(Duration::from_secs(5), "snapshot pass", |_| true)
            .await;
        for side in ["left", "right"] {
            let status = snapshot["health"][side]["status"]
                .as_str()
                .expect("side status");
            assert!(
                status == "pending" || status == "not_reporting",
                "{side} health must never go live off an unclassifiable producer, got {status}"
            );
            assert!(snapshot["health"][side]["motors"]
                .as_array()
                .is_some_and(Vec::is_empty));
        }
    }

    harness.shutdown().await
}
