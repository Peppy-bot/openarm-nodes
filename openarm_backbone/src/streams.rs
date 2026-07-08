//! Inbound stream plumbing for the hub: the operator arm and gripper command
//! streams, both arms' measured joint state, both paired grippers' measured
//! aperture, and the runtime governor controls. Each listener holds one
//! subscription and keeps the latest well-formed message in a watch channel the
//! coordinator reads every tick. One held subscription per stream means no
//! re-subscribe gap, so a message is never dropped between receives.

use std::sync::Arc;
use std::time::{Duration, Instant};

use peppygen::NodeRunner;
use peppygen::consumed_topics::{
    arm_states_arm_states, collision_ctrl_governor_control, commander_arm_joint_commands,
    commander_gripper_gripper_commands,
};
use peppygen::pairings::{left_gripper_link, right_gripper_link};
use peppylib::messaging::ProducerRef;
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

/// The latest operator joint setpoint for one arm. `seq` distinguishes a fresh
/// command from one already acted on; `producer` is the source the follow logic
/// locks to; `recv_at` is the arrival time the watchdog uses to tell a live
/// stream from a stale leftover.
#[derive(Clone)]
pub struct JointCommand {
    pub seq: u64,
    pub producer: ProducerRef,
    pub recv_at: Instant,
    pub positions: JointVec,
}

/// The latest operator opening command for one gripper (m), fed by the
/// `gripper_commands` stream. Carries the same follow fields as
/// [`JointCommand`]: `seq` distinguishes a fresh command from one already acted
/// on, `producer` is the source the coordinator's follow locks to (the stream is
/// `from_any`, so without the lock two producers would interleave), and
/// `recv_at` is the arrival time the deadman uses to tell a live stream from a
/// released one. The width stays in meters (the wire unit); the coordinator
/// parses it into the governed opening fraction.
#[derive(Clone)]
pub struct GripperCommand {
    pub seq: u64,
    pub producer: ProducerRef,
    pub recv_at: Instant,
    pub position_m: f64,
}

/// The latest measured joint position for one arm, fed by the `arm_states` stream.
/// The hub anchors trajectories and governor queries on position; the governed
/// velocity feedforward is the commanded step, not a measurement, so the stream's
/// velocities are validated but not retained.
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

/// The id-demuxed inbound listeners share one skeleton: hold the subscription
/// forever, back off on receive errors, demux by the wire id (0 = left,
/// 1 = right; out-of-range dropped with a throttled warn), and keep the latest
/// well-formed message per side in `latest[id]`. Defined once so a fix to the
/// shared behavior cannot drift across copies. `parse` builds the stored value
/// from the running per-side sequence counter, the producer, and the message, or
/// returns `None` to drop it (warning for its own reason).
macro_rules! id_demux_listener {
    (
        $(#[$doc:meta])*
        $name:ident, $topic:path, $stream:literal, $id_field:ident, $value:ty,
        |$seq:ident, $producer:ident, $msg:ident| $parse:expr
    ) => {
        $(#[$doc])*
        pub async fn $name(runner: Arc<NodeRunner>, latest: [watch::Sender<Option<$value>>; 2]) {
            use $topic as topic;
            let mut sub = match topic::subscribe(&runner).await {
                Ok(s) => s,
                Err(e) => return error!("{}: subscribe: {e}", $stream),
            };
            let mut seqs: [u64; 2] = [0, 0];
            let mut last_unknown_warn: Option<Instant> = None;
            loop {
                let (received_producer, received_msg) = match sub.next().await {
                    Ok(Some(received)) => received,
                    Ok(None) => return, // subscription closed: node shutting down
                    Err(e) => {
                        error!("{}: receive: {e}", $stream);
                        tokio::time::sleep(RECEIVE_ERROR_BACKOFF).await;
                        continue;
                    }
                };
                let Some(idx) =
                    Side::from_arm_id(received_msg.$id_field).map(Side::index)
                else {
                    warn_unknown_id(
                        $stream,
                        stringify!($id_field),
                        received_msg.$id_field,
                        &mut last_unknown_warn,
                    );
                    continue;
                };
                seqs[idx] += 1;
                let parse = |$seq: u64, $producer: ProducerRef, $msg: topic::Message| $parse;
                match parse(seqs[idx], received_producer, received_msg) {
                    Some(value) => {
                        latest[idx].send_replace(Some(value));
                    }
                    None => seqs[idx] -= 1, // dropped: the sequence marks stored values only
                }
            }
        }
    };
}

id_demux_listener!(
    /// Receive `arm_joint_commands` forever, keeping the latest well-formed message
    /// per arm. A message with any non-finite position is dropped so a producer gone
    /// bad lets the follow lock time out instead of driving an arm.
    run_joint_command_listener,
    commander_arm_joint_commands,
    "arm_joint_commands",
    arm_id,
    JointCommand,
    |seq, producer, msg| {
        if !msg.positions.iter().all(|v| v.is_finite()) {
            warn!(
                "arm_joint_commands: dropping arm {} message with non-finite positions",
                msg.arm_id
            );
            return None;
        }
        Some(JointCommand {
            seq,
            producer,
            recv_at: Instant::now(),
            positions: msg.positions,
        })
    }
);

id_demux_listener!(
    /// Receive `gripper_commands` forever, keeping the latest well-formed opening
    /// command per gripper. The mirror of `run_joint_command_listener` for the
    /// gripper's 1-DOF opening: a non-finite position is dropped so a producer gone
    /// bad lets the follow lock time out instead of driving the fingers.
    run_gripper_command_listener,
    commander_gripper_gripper_commands,
    "gripper_commands",
    gripper_id,
    GripperCommand,
    |seq, producer, msg| {
        if !msg.position.is_finite() {
            warn!(
                "gripper_commands: dropping gripper {} message with non-finite position",
                msg.gripper_id
            );
            return None;
        }
        Some(GripperCommand {
            seq,
            producer,
            recv_at: Instant::now(),
            position_m: msg.position,
        })
    }
);

id_demux_listener!(
    /// Receive `arm_states` forever, keeping the latest measured state per arm.
    /// Non-finite states are dropped so the coordinator never anchors a trajectory or
    /// a governor query on a bad measurement.
    run_joint_state_listener,
    arm_states_arm_states,
    "arm_states",
    arm_id,
    MeasuredState,
    |_seq, _producer, msg| {
        let finite = msg
            .positions
            .iter()
            .chain(msg.velocities.iter())
            .all(|v| v.is_finite());
        if !finite {
            warn!(
                "arm_states: dropping arm {} message with non-finite state",
                msg.arm_id
            );
            return None;
        }
        Some(MeasuredState {
            positions: msg.positions,
        })
    }
);

/// Receive both paired grippers' measured aperture forever (the gripper_link
/// back-channel), keeping the latest opening fraction per side. The slot IS the
/// side (a pairing delivers only its one peer's messages), so there is no id
/// demux, and the hub's collision model reads finger positions only from its
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

/// The hub's runtime governor controls from the operator: the on/off toggle plus
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
/// cannot strand the hub in a stale state.
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
