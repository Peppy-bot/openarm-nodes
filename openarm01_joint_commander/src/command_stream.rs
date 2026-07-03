// Always-on command publisher. For each enabled arm and gripper, streams its
// target setpoint at command_rate_hz on that side's pairing slot so the paired
// node follows it; a disabled side emits nothing, so the node's stream timeout
// lapses and it holds. Re-publishing every tick (even an unchanged target)
// keeps the paired node's stream watchdog alive between operator inputs. The
// arm/gripper clamps and rate-limits what it receives, so this only has to
// deliver the latest setpoint. Publishing on an unpaired slot is a legal
// no-op, so an operator enabling a side before its peer is paired is harmless.
//
// Each side+channel runs its own publish task on its own interval, with its
// own slot-scoped publisher. A single shared loop publishing Left then Right
// would leave Right permanently second (zenoh publish resolves synchronously),
// so independent tasks avoid that bias.

use std::sync::Arc;
use std::time::Duration;

use peppygen::NodeRunner;
use peppygen::peers::left_arm::joint_commands as left_joint_commands;
use peppygen::peers::left_gripper::gripper_commands as left_gripper_commands;
use peppygen::peers::right_arm::joint_commands as right_joint_commands;
use peppygen::peers::right_gripper::gripper_commands as right_gripper_commands;
use peppylib::runtime::CancellationToken;
use peppylib::{Payload, TopicPublisher};
use tokio::time::MissedTickBehavior;
use tracing::{error, warn};

use crate::state::{SharedState, Side};

pub async fn run(
    runner: Arc<NodeRunner>,
    state: SharedState,
    command_rate_hz: u32,
    token: CancellationToken,
) {
    // One slot-scoped publisher per side+channel; each stamps its own slot's
    // link_id on the wire, so the two arms' streams stay fully isolated.
    let left_arm_pub = match left_joint_commands::declare_publisher(&runner).await {
        Ok(p) => p,
        Err(e) => return error!("declare left_arm joint_commands publisher: {e}"),
    };
    let right_arm_pub = match right_joint_commands::declare_publisher(&runner).await {
        Ok(p) => p,
        Err(e) => return error!("declare right_arm joint_commands publisher: {e}"),
    };
    let left_gripper_pub = match left_gripper_commands::declare_publisher(&runner).await {
        Ok(p) => p,
        Err(e) => return error!("declare left_gripper gripper_commands publisher: {e}"),
    };
    let right_gripper_pub = match right_gripper_commands::declare_publisher(&runner).await {
        Ok(p) => p,
        Err(e) => return error!("declare right_gripper gripper_commands publisher: {e}"),
    };

    type ArmBuilder = Box<dyn Fn([f64; 7]) -> Result<Payload, String> + Send>;
    type GripperBuilder = Box<dyn Fn(f64) -> Result<Payload, String> + Send>;

    let mut tasks = Vec::new();
    // The four side+channel streams, each with its slot publisher and a
    // builder producing that slot's message (no arm_id / gripper_id: the
    // pairing scopes each stream to its peer).
    let arm_channels: [(Side, TopicPublisher, ArmBuilder); 2] = [
        (
            Side::Left,
            left_arm_pub,
            Box::new(|target| {
                left_joint_commands::build_message(target).map_err(|e| e.to_string())
            }),
        ),
        (
            Side::Right,
            right_arm_pub,
            Box::new(|target| {
                right_joint_commands::build_message(target).map_err(|e| e.to_string())
            }),
        ),
    ];
    for (side, publisher, build) in arm_channels {
        let arm_state = state.clone();
        tasks.push(tokio::spawn(stream_setpoints(
            publisher,
            command_rate_hz,
            token.clone(),
            format!("{} arm", side.label()),
            move || {
                let target = {
                    let s = arm_state.lock().unwrap_or_else(|p| p.into_inner());
                    if !s.enabled(side) {
                        return None;
                    }
                    s.arm(side).joints
                };
                Some(build(target))
            },
        )));
    }
    let gripper_channels: [(Side, TopicPublisher, GripperBuilder); 2] = [
        (
            Side::Left,
            left_gripper_pub,
            Box::new(|target| {
                left_gripper_commands::build_message(target).map_err(|e| e.to_string())
            }),
        ),
        (
            Side::Right,
            right_gripper_pub,
            Box::new(|target| {
                right_gripper_commands::build_message(target).map_err(|e| e.to_string())
            }),
        ),
    ];
    for (side, publisher, build) in gripper_channels {
        let gripper_state = state.clone();
        tasks.push(tokio::spawn(stream_setpoints(
            publisher,
            command_rate_hz,
            token.clone(),
            format!("{} gripper", side.label()),
            move || {
                let target = {
                    let s = gripper_state.lock().unwrap_or_else(|p| p.into_inner());
                    if !s.enabled(side) {
                        return None;
                    }
                    s.gripper(side).position
                };
                Some(build(target))
            },
        )));
    }
    for task in tasks {
        let _ = task.await;
    }
}

// Publish the latest setpoint from `next_message` at command_rate_hz, skipping a
// tick whenever it returns None (the side is disabled). Failures latch so a
// stuck side warns once, not every tick.
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
