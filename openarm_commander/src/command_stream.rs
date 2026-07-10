// Always-on command publisher. For each enabled arm, streams its 7-joint target
// on `arm_joint_commands`; for each enabled gripper, streams its opening (m) on
// `gripper_commands`. Both are tagged with their id (arm_id / gripper_id) and go
// to the hub, which governs each and re-streams the governed value the followers
// track. A disabled side emits nothing, so the hub's stream timeout lapses and it
// holds. Re-publishing every tick (even an unchanged target) keeps the hub's
// stream watchdog alive between operator inputs; the hub clamps and rate-limits
// what it receives, so this only has to deliver the latest setpoint.
//
// Each side+stream runs its own publish task on its own interval, cloning the
// shared per-topic publisher. A single shared loop publishing Left then Right
// would leave Right permanently second (zenoh publish resolves synchronously), so
// independent tasks avoid that bias.

use std::sync::Arc;
use std::time::Duration;

use peppygen::NodeRunner;
use peppygen::emitted_topics::openarm_arm_joint_commands::v1::arm_joint_commands;
use peppygen::emitted_topics::openarm_governor_control::v1::governor_control;
use peppygen::emitted_topics::openarm_gripper_commands::v1::gripper_commands;
use peppylib::runtime::CancellationToken;
use peppylib::{Payload, TopicPublisher};
use tokio::time::MissedTickBehavior;
use tracing::{error, info, warn};

use crate::pose::{ArmModels, JogCaps, JogStep, jog_tick};
use crate::state::{SharedState, Side, UiState};

pub async fn run(
    runner: Arc<NodeRunner>,
    state: SharedState,
    command_rate_hz: u32,
    token: CancellationToken,
    models: ArmModels,
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
    // One shared gripper publisher, cloned per side like the arm publisher; each
    // side's stream tags its own gripper_id, so the hub tells them apart.
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

    // Re-publish the operator's governor controls every tick. Unlike the arm/gripper
    // streams these have no deadman: the hub's governor must always know the
    // operator's intent, and the lossy QoS means a one-shot publish could be
    // dropped, so the latest state is re-sent continuously.
    let governor_state = state.clone();
    tasks.spawn(stream_setpoints(
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
    ));
    // The tick period every stream runs at; jog steps derive from it so a different
    // command_rate_hz changes the step size, never the jog speed.
    let tick_dt_s = 1.0 / command_rate_hz as f64;
    for side in [Side::Left, Side::Right] {
        // Arm: advance any active Cartesian jog one capped step, then stream the
        // 7-joint setpoint while enabled.
        let arm_state = state.clone();
        let arm_models = models.clone();
        tasks.spawn(stream_setpoints(
            arm_pub.clone(),
            command_rate_hz,
            token.clone(),
            format!("{} arm", side.label()),
            move || {
                let target = {
                    let mut s = arm_state.lock().unwrap_or_else(|p| p.into_inner());
                    if !s.enabled[side] {
                        return None;
                    }
                    // Caps re-derive each tick from the operator's live EE speed cap,
                    // so retuning the knob mid-jog changes the jog speed with it.
                    let caps = JogCaps::per_tick(tick_dt_s, s.max_ee_velocity_m_s);
                    advance_jog(&mut s, side, &arm_models, caps);
                    s.arms[side].joints
                };
                Some(
                    arm_joint_commands::build_message(side.arm_id(), target)
                        .map_err(|e| e.to_string()),
                )
            },
        ));
        // Gripper: stream the opening (m) while enabled, tagged with gripper_id
        // for the hub to demux (mirror of the arm stream above).
        let gripper_state = state.clone();
        tasks.spawn(stream_setpoints(
            gripper_pub.clone(),
            command_rate_hz,
            token.clone(),
            format!("{} gripper", side.label()),
            move || {
                let position = {
                    let s = gripper_state.lock().unwrap_or_else(|p| p.into_inner());
                    if !s.enabled[side] {
                        return None;
                    }
                    s.grippers[side].position
                };
                Some(
                    gripper_commands::build_message(side.gripper_id(), position)
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

// Advance one side's Cartesian jog by one tick, if one is armed: step the joint
// target a capped increment toward the desired pose, hold it at the reach boundary,
// and retire the jog once it has converged. Status lines fire only on the
// moving <-> blocked transitions, so a held boundary reports once, not at 100 Hz.
// Called under the UiState lock; jog_tick briefly takes the model lock inside it,
// the same state -> model order as the UI snapshot, so the two cannot deadlock.
fn advance_jog(s: &mut UiState, side: Side, models: &ArmModels, caps: JogCaps) {
    let Some(jog) = s.arms[side].pose_jog else {
        return;
    };
    match jog_tick(
        models,
        side,
        &s.arms[side].joints,
        &jog.desired,
        jog.arm_angle,
        jog.mode,
        caps,
    ) {
        JogStep::Converged => {
            s.arms[side].pose_jog = None;
            s.arms[side].pose_blocked = false;
        }
        JogStep::Stepped(q) => {
            s.arms[side].joints = q;
            if s.arms[side].pose_blocked {
                s.arms[side].pose_blocked = false;
                s.set_status(format!("{}: pose jog moving", side.label()));
                info!(side = side.label(), mode = ?jog.mode, "pose jog resumed");
            }
        }
        JogStep::Blocked => {
            if !s.arms[side].pose_blocked {
                s.arms[side].pose_blocked = true;
                s.set_status(format!("{}: pose at reach limit, holding", side.label()));
                info!(side = side.label(), mode = ?jog.mode, desired = ?jog.desired, "pose jog at reach limit");
            }
        }
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
