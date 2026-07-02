// Always-on command publisher. For each enabled arm and gripper, streams its
// target setpoint at command_rate_hz so the node follows it; a disabled side
// emits nothing, so the node's stream timeout lapses and it holds. Re-publishing
// every tick (even an unchanged target) keeps the node's producer lock alive
// between operator inputs. The arm/gripper clamps and rate-limits what it
// receives, so this only has to deliver the latest setpoint.
//
// Each side+channel runs its own publish task on its own interval, sharing a
// cloneable lock-free publisher per topic (one zenoh session). A single shared
// loop publishing Left then Right would leave Right permanently second (zenoh
// publish resolves synchronously), so independent tasks avoid that bias.

use std::sync::Arc;
use std::time::Duration;

use peppygen::NodeRunner;
use peppygen::emitted_topics::openarm_arm_joint_commands::v1::arm_joint_commands;
use peppygen::emitted_topics::openarm_governor_control::v1::governor_control;
use peppygen::emitted_topics::openarm_gripper_commands::v1::gripper_commands;
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
    // A failed publisher declaration leaves the node serving UI/health but unable to
    // command anything, so cancel the node to restart it rather than returning quietly.
    let arm_pub = match arm_joint_commands::declare_publisher(&runner).await {
        Ok(p) => p,
        Err(e) => {
            error!("declare arm_joint_commands publisher: {e}");
            return token.cancel();
        }
    };
    let gripper_pub = match gripper_commands::declare_publisher(&runner).await {
        Ok(p) => p,
        Err(e) => {
            error!("declare gripper_commands publisher: {e}");
            return token.cancel();
        }
    };
    let governor_pub = match governor_control::declare_publisher(&runner).await {
        Ok(p) => p,
        Err(e) => {
            error!("declare governor_control publisher: {e}");
            return token.cancel();
        }
    };

    let mut tasks = Vec::new();

    // Re-publish the operator's governor controls every tick. Unlike the arm/gripper
    // streams these have no deadman: the hub's governor must always know the
    // operator's intent, and the lossy QoS means a one-shot publish could be
    // dropped, so the latest state is re-sent continuously.
    let governor_state = state.clone();
    tasks.push(tokio::spawn(stream_setpoints(
        governor_pub,
        command_rate_hz,
        token.clone(),
        "governor control".to_string(),
        move || {
            let s = governor_state.lock().unwrap_or_else(|p| p.into_inner());
            Some(
                governor_control::build_message(
                    s.collision_enabled,
                    s.d_stop,
                    s.d_safe,
                    s.max_ee_velocity_m_s,
                )
                .map_err(|e| e.to_string()),
            )
        },
    )));
    for side in [Side::Left, Side::Right] {
        // Arm: stream the 7-joint setpoint while enabled.
        let arm_state = state.clone();
        tasks.push(tokio::spawn(stream_setpoints(
            arm_pub.clone(),
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
                Some(
                    arm_joint_commands::build_message(side.arm_id(), target)
                        .map_err(|e| e.to_string()),
                )
            },
        )));
        // Gripper: stream the opening setpoint while enabled.
        let gripper_state = state.clone();
        tasks.push(tokio::spawn(stream_setpoints(
            gripper_pub.clone(),
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
                Some(
                    gripper_commands::build_message(side.gripper_id(), target)
                        .map_err(|e| e.to_string()),
                )
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
