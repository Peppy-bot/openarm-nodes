// Always-on command publisher. Reads the owner's per-tick `CommandFrame` and streams
// each enabled side's arm setpoint on `arm_joint_commands` and gripper opening on
// `gripper_commands`, plus the operator's governor controls on `governor_control`. Both
// are tagged with their id (arm_id / gripper_id) and go to the backbone, which governs each
// and re-streams the governed value the followers track. A side with `None` in the
// frame has its deadman off, so nothing is published and the backbone holds that side
// at its last governed setpoint. Re-publishing every tick (even an unchanged frame)
// keeps the stream trivially fresh for a backbone that starts or re-pairs mid-session.
//
// The owner is the sole writer of state and the sole jog integrator; these tasks only
// forward the latest frame. Each side+stream runs its own publish task on its own
// interval, cloning the shared per-topic publisher and the frame receiver. A single
// shared loop publishing Left then Right would leave Right permanently second (zenoh
// publish resolves synchronously), so independent tasks avoid that bias.

use std::sync::Arc;
use std::time::Duration;

use peppygen::NodeRunner;
use peppygen::emitted_topics::openarm_commands::v1::arm_joint_commands;
use peppygen::emitted_topics::openarm_commands::v1::gripper_commands;
use peppygen::emitted_topics::openarm_governor_control::v1::governor_control;
use peppylib::runtime::CancellationToken;
use peppylib::{Payload, TopicPublisher};
use tokio::sync::watch;
use tokio::time::MissedTickBehavior;
use tracing::{error, warn};

use crate::owner::CommandFrame;
use crate::state::Side;

pub async fn run(
    runner: Arc<NodeRunner>,
    command_rate_hz: u32,
    token: CancellationToken,
    frame_rx: watch::Receiver<CommandFrame>,
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
    // One shared gripper publisher, cloned per side like the arm publisher; each side's
    // stream tags its own gripper_id, so the backbone tells them apart.
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

    let mut tasks = tokio::task::JoinSet::new();

    // Governor controls: no deadman, so this always publishes the latest frame.
    let governor_rx = frame_rx.clone();
    tasks.spawn(stream_setpoints(
        governor_pub,
        command_rate_hz,
        token.clone(),
        "governor control".to_string(),
        move || {
            let g = governor_rx.borrow().governor;
            Some(
                governor_control::build_message(
                    g.collision_enabled,
                    g.d_stop,
                    g.d_safe,
                    g.max_ee_velocity_m_s,
                )
                .map_err(|e| e.to_string()),
            )
        },
    ));
    for side in [Side::Left, Side::Right] {
        // Arm: stream the 7-joint setpoint while enabled (None in the frame = disabled).
        let arm_rx = frame_rx.clone();
        tasks.spawn(stream_setpoints(
            arm_pub.clone(),
            command_rate_hz,
            token.clone(),
            format!("{} arm", side.label()),
            move || {
                let joints = arm_rx.borrow().arms[side]?;
                Some(
                    arm_joint_commands::build_message(side.arm_id(), joints)
                        .map_err(|e| e.to_string()),
                )
            },
        ));
        // Gripper: stream the opening fraction while enabled, tagged with
        // gripper_id for the backbone to demux (mirror of the arm stream above).
        let gripper_rx = frame_rx.clone();
        tasks.spawn(stream_setpoints(
            gripper_pub.clone(),
            command_rate_hz,
            token.clone(),
            format!("{} gripper", side.label()),
            move || {
                let opening = gripper_rx.borrow().grippers[side]?;
                Some(
                    gripper_commands::build_message(side.gripper_id(), opening)
                        .map_err(|e| e.to_string()),
                )
            },
        ));
    }
    // join_next surfaces tasks in completion order, so a panicked stream is seen
    // immediately. A dead channel would silently hold its side while the node reports
    // healthy, which is worse than a restart: cancel the node.
    while let Some(result) = tasks.join_next().await {
        if let Err(e) = result {
            error!("command stream task died: {e}; cancelling the node");
            token.cancel();
        }
    }
}

// Publish the latest setpoint from `next_message` at command_rate_hz, skipping a tick
// whenever it returns None (the side is disabled). Failures latch so a stuck side warns
// once, not every tick.
async fn stream_setpoints(
    publisher: TopicPublisher,
    command_rate_hz: u32,
    token: CancellationToken,
    label: String,
    mut next_message: impl FnMut() -> Option<Result<Payload, String>>,
) {
    let period = Duration::from_micros(1_000_000 / command_rate_hz as u64);
    // interval (not sleep) so the publish cadence holds at command_rate_hz instead of
    // drifting by the per-tick work time; Delay avoids a catch-up burst after a
    // scheduling hiccup.
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
