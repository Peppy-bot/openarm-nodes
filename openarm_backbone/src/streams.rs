//! Inbound stream plumbing for the backbone: the commander's arm and gripper command
//! streams, both paired arms' measured joint state, both paired grippers'
//! measured aperture, and the runtime governor controls. Each listener holds
//! one subscription and keeps the latest well-formed message in a watch channel
//! the coordinator reads every tick. One held subscription per stream means no
//! re-subscribe gap, so a message is never dropped between receives.

use std::sync::Arc;
use std::time::{Duration, Instant};

use peppygen::NodeRunner;
use peppygen::consumed_topics::{
    collision_ctrl_governor_control, commander_arm_joint_commands, commander_gripper_commands,
};
use peppygen::pairings::{left_arm_link, left_gripper_link, right_arm_link, right_gripper_link};
use tokio::sync::watch;
use tracing::{error, warn};

use crate::{JointVec, Side};

/// At most one unknown-id warning per stream in this window, so a misrouted
/// producer is visible in the log without flooding it at the stream rate.
const UNKNOWN_ARM_WARN_PERIOD: Duration = Duration::from_secs(1);

/// Pause after a receive error before retrying, so a persistently broken
/// subscription cannot spin the listener at full CPU or flood the log at the stream
/// rate. Transient errors still recover; a genuinely dead stream just idles.
const RECEIVE_ERROR_BACKOFF: Duration = Duration::from_millis(100);

/// The latest commander joint setpoint for one arm, kept by the arm command
/// listener and read each tick by the planner's Follow. The command stream is
/// paired to one producer, so the latest message is simply the one to chase.
#[derive(Clone)]
pub struct JointCommand {
    pub positions: JointVec,
}

/// The latest commander opening command for one gripper, fed by the
/// `gripper_commands` stream. The width stays in meters (the wire unit); the
/// coordinator parses it into the governed opening fraction.
#[derive(Clone)]
pub struct GripperCommand {
    pub position_m: f64,
}

/// The latest measured joint position for one arm, fed by the arm_link
/// pairing's `arm_states` back-channel. The backbone anchors trajectories and
/// governor queries on position; the governed velocity feedforward is the
/// commanded step, not a measurement, so the stream's velocities are validated
/// but not retained.
#[derive(Clone, Copy)]
pub struct MeasuredState {
    pub positions: JointVec,
}

/// The latest measured opening of one gripper, parsed at ingestion from the
/// pairing's aperture width into a fraction of the full jaw travel and clamped
/// into `[0, 1]` (0 = closed, 1 = fully open), so downstream consumers never see
/// raw hardware units or an out-of-range placement. Consumed every tick like the
/// measured arm state.
#[derive(Clone, Copy)]
pub struct GripperOpening {
    pub fraction: f64,
}

/// Warn that a message carried an out-of-range id (`field` names it: `arm_id`
/// or `gripper_id`), throttled to at most once per [`UNKNOWN_ARM_WARN_PERIOD`].
fn warn_unknown_id(stream: &str, field: &str, id: u8, last: &mut Option<Instant>) {
    let now = Instant::now();
    if last.is_none_or(|t| now.duration_since(t) >= UNKNOWN_ARM_WARN_PERIOD) {
        warn!("{stream}: dropping message for unknown {field} {id}");
        *last = Some(now);
    }
}

/// Receive `arm_joint_commands` forever, keeping the latest well-formed message
/// per arm in `latest[id]`. Holds one subscription (no re-subscribe gap), backs
/// off on a receive error, and drops an out-of-range or non-finite message rather
/// than driving an arm.
pub async fn run_joint_command_listener(
    runner: Arc<NodeRunner>,
    latest: [watch::Sender<Option<JointCommand>>; 2],
) {
    let mut sub = match commander_arm_joint_commands::subscribe(&runner).await {
        Ok(s) => s,
        Err(e) => return error!("arm_joint_commands: subscribe: {e}"),
    };
    let mut last_unknown_warn: Option<Instant> = None;
    loop {
        let msg = match sub.next().await {
            Ok(Some((_producer, msg))) => msg,
            Ok(None) => return, // subscription closed: node shutting down
            Err(e) => {
                error!("arm_joint_commands: receive: {e}");
                tokio::time::sleep(RECEIVE_ERROR_BACKOFF).await;
                continue;
            }
        };
        let Some(idx) = Side::from_arm_id(msg.arm_id).map(Side::index) else {
            warn_unknown_id(
                "arm_joint_commands",
                "arm_id",
                msg.arm_id,
                &mut last_unknown_warn,
            );
            continue;
        };
        if !msg.positions.iter().all(|v| v.is_finite()) {
            warn!(
                "arm_joint_commands: dropping arm {} message with non-finite positions",
                msg.arm_id
            );
            continue;
        }
        latest[idx].send_replace(Some(JointCommand {
            positions: msg.positions,
        }));
    }
}

/// Receive `gripper_commands` forever, keeping the latest well-formed opening per
/// gripper in `latest[id]`. The 1-DOF mirror of [`run_joint_command_listener`]: an
/// out-of-range or non-finite message is dropped rather than driving the fingers.
pub async fn run_gripper_command_listener(
    runner: Arc<NodeRunner>,
    latest: [watch::Sender<Option<GripperCommand>>; 2],
) {
    let mut sub = match commander_gripper_commands::subscribe(&runner).await {
        Ok(s) => s,
        Err(e) => return error!("gripper_commands: subscribe: {e}"),
    };
    let mut last_unknown_warn: Option<Instant> = None;
    loop {
        let msg = match sub.next().await {
            Ok(Some((_producer, msg))) => msg,
            Ok(None) => return,
            Err(e) => {
                error!("gripper_commands: receive: {e}");
                tokio::time::sleep(RECEIVE_ERROR_BACKOFF).await;
                continue;
            }
        };
        let Some(idx) = Side::from_gripper_id(msg.gripper_id).map(Side::index) else {
            warn_unknown_id(
                "gripper_commands",
                "gripper_id",
                msg.gripper_id,
                &mut last_unknown_warn,
            );
            continue;
        };
        if !msg.position.is_finite() {
            warn!(
                "gripper_commands: dropping gripper {} message with non-finite position",
                msg.gripper_id
            );
            continue;
        }
        latest[idx].send_replace(Some(GripperCommand {
            position_m: msg.position,
        }));
    }
}

/// Receive both paired arms' measured state forever (the arm_link back-channel),
/// keeping the latest per side. The slot IS the side (a pairing delivers only
/// its one peer), so there is no id demux, and the governor anchors only on its
/// exclusive command-loop peers: a stray broadcast producer cannot pose as an
/// arm. Non-finite states are dropped so the coordinator never anchors a
/// trajectory or a governor query on a bad measurement.
pub async fn run_joint_state_listener(
    runner: Arc<NodeRunner>,
    latest: [watch::Sender<Option<MeasuredState>>; 2],
) {
    let (left, right) = tokio::join!(
        left_arm_link::arm_states::subscribe(&runner),
        right_arm_link::arm_states::subscribe(&runner),
    );
    let (mut left, mut right) = match (left, right) {
        (Ok(l), Ok(r)) => (l, r),
        (l, r) => {
            return error!(
                "arm_states subscribe: left {:?}, right {:?}",
                l.err(),
                r.err()
            );
        }
    };
    let parse = |side: Side, positions: JointVec, velocities: JointVec| -> Option<MeasuredState> {
        let finite = positions
            .iter()
            .chain(velocities.iter())
            .all(|v| v.is_finite());
        if !finite {
            warn!(
                "arm_states: dropping {} message with non-finite state",
                side.label()
            );
            return None;
        }
        Some(MeasuredState { positions })
    };
    loop {
        // Whichever slot delivers next wins the select; the other stays queued in
        // its own subscription, so neither side can starve the other.
        let (side, received) = tokio::select! {
            r = left.next() => (Side::Left, r.map(|m| m.map(|(_, msg)| (msg.positions, msg.velocities)))),
            r = right.next() => (Side::Right, r.map(|m| m.map(|(_, msg)| (msg.positions, msg.velocities)))),
        };
        match received {
            Ok(Some((positions, velocities))) => {
                if let Some(state) = parse(side, positions, velocities) {
                    latest[side.index()].send_replace(Some(state));
                }
            }
            Ok(None) => return, // subscription closed: node shutting down
            Err(e) => {
                error!("arm_states receive ({}): {e}", side.label());
                tokio::time::sleep(RECEIVE_ERROR_BACKOFF).await;
            }
        }
    }
}

/// Receive both paired grippers' measured aperture forever (the gripper_link
/// back-channel), keeping the latest opening fraction per side. The slot IS the
/// side (a pairing delivers only its one peer's messages), so there is no id
/// demux, and the backbone's collision model reads finger positions only from its
/// exclusive command-loop peers: a stray broadcast producer cannot spoof the
/// modeled fingers. A non-finite width is dropped so the model never places the
/// fingers on a bad reading; the parsed fraction is clamped into the jaw travel.
pub async fn run_gripper_state_listener(
    runner: Arc<NodeRunner>,
    latest: [watch::Sender<Option<GripperOpening>>; 2],
    jaw_open_m: f64,
) {
    let (left, right) = tokio::join!(
        left_gripper_link::gripper_states::subscribe(&runner),
        right_gripper_link::gripper_states::subscribe(&runner),
    );
    let (mut left, mut right) = match (left, right) {
        (Ok(l), Ok(r)) => (l, r),
        (l, r) => {
            return error!(
                "gripper_states subscribe: left {:?}, right {:?}",
                l.err(),
                r.err()
            );
        }
    };
    let parse = |side: Side, position_m: f64| -> Option<GripperOpening> {
        let fraction = position_m / jaw_open_m;
        if !fraction.is_finite() {
            warn!(
                "gripper_states: dropping {} message with non-finite opening (width {position_m})",
                side.label()
            );
            return None;
        }
        Some(GripperOpening {
            fraction: fraction.clamp(0.0, 1.0),
        })
    };
    loop {
        // Whichever slot delivers next wins the select; the other stays queued in
        // its own subscription, so neither side can starve the other.
        let (side, received) = tokio::select! {
            r = left.next() => (Side::Left, r.map(|m| m.map(|(_, msg)| msg.position))),
            r = right.next() => (Side::Right, r.map(|m| m.map(|(_, msg)| msg.position))),
        };
        match received {
            Ok(Some(position_m)) => {
                if let Some(opening) = parse(side, position_m) {
                    latest[side.index()].send_replace(Some(opening));
                }
            }
            Ok(None) => return, // subscription closed: node shutting down
            Err(e) => {
                error!("gripper_states receive ({}): {e}", side.label());
                tokio::time::sleep(RECEIVE_ERROR_BACKOFF).await;
            }
        }
    }
}

/// The backbone's runtime governor controls from the commander: the on/off toggle plus
/// the live-tunable band and stream speed cap. Raw values; the governor and
/// planners validate them as they apply (an invalid band or speed is ignored,
/// keeping the last good one), so the toggle still takes effect regardless.
#[derive(Clone, Copy)]
pub struct GovernorConfig {
    pub enabled: bool,
    pub d_stop: f64,
    pub d_safe: f64,
    pub max_ee_velocity_m_s: f64,
}

/// Receive the `governor_control` stream forever, mirroring the latest governor
/// config into `config`. The producer re-publishes periodically, so the lossy QoS
/// cannot strand the backbone in a stale state.
pub async fn run_governor_config_listener(
    runner: Arc<NodeRunner>,
    config: watch::Sender<GovernorConfig>,
) {
    let mut sub = match collision_ctrl_governor_control::subscribe(&runner).await {
        Ok(s) => s,
        Err(e) => return error!("governor_control subscribe: {e}"),
    };
    loop {
        match sub.next().await {
            Ok(Some((_, msg))) => {
                config.send_replace(GovernorConfig {
                    enabled: msg.collision_governor_enabled,
                    d_stop: msg.d_stop,
                    d_safe: msg.d_safe,
                    max_ee_velocity_m_s: msg.max_ee_velocity_m_s,
                });
            }
            Ok(None) => return,
            Err(e) => {
                error!("governor_control receive: {e}");
                tokio::time::sleep(RECEIVE_ERROR_BACKOFF).await;
            }
        }
    }
}
