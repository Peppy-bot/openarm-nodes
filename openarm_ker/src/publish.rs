// Always-on command publisher, the same shape as the commander's: for
// each side, one task streams the arm setpoint at command_rate_hz on
// `arm_joint_commands` and one streams the trigger opening on
// `gripper_commands`, both tagged with their id and governed by the hub before
// anything reaches a follower. A tick publishes nothing when the newest sample
// is missing, stale, or disengaged, so the hub's stream timeouts lapse and the
// robot holds: skipping is the deadman. Re-publishing an unchanged sample every
// tick keeps the hub's stream watchdogs alive between device frames.
//
// Each side+stream runs its own publish task on its own interval, cloning the
// shared per-topic publisher. A single shared loop publishing Left then Right
// would leave Right permanently second (zenoh publish resolves synchronously),
// so independent tasks avoid that bias.

use std::sync::Arc;
use std::time::Duration;

use openarm_description::Side;
use peppygen::NodeRunner;
use peppygen::emitted_topics::openarm_arm_joint_commands::v1::arm_joint_commands;
use peppygen::emitted_topics::openarm_gripper_commands::v1::gripper_commands;
use peppylib::runtime::CancellationToken;
use peppylib::{Payload, TopicPublisher};
use tokio::sync::watch;
use tokio::time::MissedTickBehavior;
use tracing::{error, warn};

use crate::reader::KerSample;

/// The wire id for a side, the same 0 = left / 1 = right encoding every openarm
/// stream uses (`arm_id` and `gripper_id` alike).
fn wire_id(side: Side) -> u8 {
    match side {
        Side::Left => 0,
        Side::Right => 1,
    }
}

fn label(side: Side) -> &'static str {
    match side {
        Side::Left => "left",
        Side::Right => "right",
    }
}

pub async fn run(
    runner: Arc<NodeRunner>,
    rx: watch::Receiver<Option<KerSample>>,
    command_rate_hz: u32,
    stale_timeout: Duration,
    token: CancellationToken,
) {
    // A failed publisher declaration leaves the node connected to the device
    // but unable to command anything, so cancel the node to restart it rather
    // than returning quietly.
    let arm_pub = match arm_joint_commands::declare_publisher(&runner).await {
        Ok(p) => p,
        Err(e) => {
            error!("declare arm_joint_commands publisher: {e}");
            return token.cancel();
        }
    };
    // One shared gripper publisher, cloned per side like the arm publisher;
    // each side's stream tags its own gripper_id, so the hub tells them apart.
    let gripper_pub = match gripper_commands::declare_publisher(&runner).await {
        Ok(p) => p,
        Err(e) => {
            error!("declare gripper_commands publisher: {e}");
            return token.cancel();
        }
    };

    let mut tasks = tokio::task::JoinSet::new();

    for side in [Side::Left, Side::Right] {
        let sample_rx = rx.clone();
        tasks.spawn(stream_setpoints(
            arm_pub.clone(),
            command_rate_hz,
            token.clone(),
            format!("{} arm", label(side)),
            move || {
                let target = streamable(&sample_rx, stale_timeout)?.joints(side);
                Some(
                    arm_joint_commands::build_message(wire_id(side), target)
                        .map_err(|e| e.to_string()),
                )
            },
        ));
        // Gripper: stream the trigger opening (m) while streamable, tagged with
        // the side's id for the hub to demux (mirror of the arm stream above).
        let sample_rx = rx.clone();
        tasks.spawn(stream_setpoints(
            gripper_pub.clone(),
            command_rate_hz,
            token.clone(),
            format!("{} gripper", label(side)),
            move || {
                let opening = streamable(&sample_rx, stale_timeout)?.opening_m(side);
                Some(
                    gripper_commands::build_message(wire_id(side), opening)
                        .map_err(|e| e.to_string()),
                )
            },
        ));
    }
    // join_next surfaces tasks in completion order, so a panicked stream is
    // seen immediately. A dead channel would silently hold its side while the
    // node reports healthy, which is worse than a restart: cancel the node.
    while let Some(result) = tasks.join_next().await {
        if let Err(e) = result {
            error!("command stream task died: {e}; cancelling the node");
            token.cancel();
        }
    }
}

/// The newest sample if it should stream: present, engaged, and fresher than
/// the stale timeout. `None` skips the tick, which is what holds the robot.
fn streamable(
    rx: &watch::Receiver<Option<KerSample>>,
    stale_timeout: Duration,
) -> Option<KerSample> {
    let sample = rx.borrow().clone()?;
    (sample.engaged && sample.received_at.elapsed() < stale_timeout).then_some(sample)
}

// Publish the latest setpoint from `next_message` at command_rate_hz, skipping
// a tick whenever it returns None. Failures latch so a stuck channel warns
// once, not every tick.
async fn stream_setpoints(
    publisher: TopicPublisher,
    command_rate_hz: u32,
    token: CancellationToken,
    label: String,
    mut next_message: impl FnMut() -> Option<Result<Payload, String>>,
) {
    let period = Duration::from_micros(1_000_000 / command_rate_hz as u64);
    // interval (not sleep) so the publish cadence holds at command_rate_hz
    // instead of drifting by the per-tick work time; Delay avoids a catch-up
    // burst after a scheduling hiccup.
    let mut ticker = tokio::time::interval(period);
    ticker.set_missed_tick_behavior(MissedTickBehavior::Delay);
    let mut failing = false;

    loop {
        tokio::select! {
            _ = token.cancelled() => return,
            _ = ticker.tick() => {}
        }

        let Some(built) = next_message() else {
            continue;
        };
        let result = match built {
            Ok(msg) => publisher.publish(msg).await.map_err(|e| e.to_string()),
            Err(e) => Err(e),
        };
        match result {
            Ok(()) => failing = false,
            Err(e) if !failing => {
                failing = true;
                warn!("{label} command publish failing, suppressing repeats: {e}");
            }
            Err(_) => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use std::time::Instant;

    use super::*;

    fn sample(engaged: bool, age: Duration) -> KerSample {
        KerSample {
            left_joints: [0.0; 7],
            right_joints: [0.0; 7],
            left_opening_m: 0.0,
            right_opening_m: 0.0,
            engaged,
            received_at: Instant::now() - age,
        }
    }

    #[test]
    fn streams_only_fresh_engaged_samples() {
        let stale = Duration::from_millis(250);
        let (tx, rx) = watch::channel(None);
        assert!(streamable(&rx, stale).is_none(), "no sample yet");

        tx.send(Some(sample(true, Duration::ZERO))).unwrap();
        assert!(streamable(&rx, stale).is_some());

        tx.send(Some(sample(false, Duration::ZERO))).unwrap();
        assert!(streamable(&rx, stale).is_none(), "disengaged holds");

        tx.send(Some(sample(true, Duration::from_secs(1)))).unwrap();
        assert!(streamable(&rx, stale).is_none(), "stale holds");

        tx.send(None).unwrap();
        assert!(streamable(&rx, stale).is_none(), "device loss holds");
    }
}
