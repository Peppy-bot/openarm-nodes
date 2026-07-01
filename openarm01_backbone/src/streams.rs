//! Inbound stream plumbing for the hub: the operator joint-command stream, both
//! arms' measured joint state, and the runtime governor controls. Each listener
//! holds one subscription and keeps the latest well-formed message in a watch
//! channel the coordinator reads every tick. One held subscription per stream
//! means no re-subscribe gap, so a message is never dropped between receives.
//! Arms are told apart by `arm_id` (0 = left, 1 = right); a message for an unknown
//! arm is dropped (and warned, throttled).

use std::sync::Arc;
use std::time::{Duration, Instant};

use peppygen::NodeRunner;
use peppygen::consumed_topics::{
    arm_states_arm_states, collision_ctrl_governor_control, commander_arm_joint_commands,
};
use peppylib::messaging::ProducerRef;
use tokio::sync::watch;
use tracing::{error, warn};

use crate::{JointVec, Side};

/// At most one unknown-`arm_id` warning per stream in this window, so a misrouted
/// producer is visible in the log without flooding it at the stream rate.
const UNKNOWN_ARM_WARN_PERIOD: Duration = Duration::from_secs(1);

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

/// The latest measured joint position for one arm, fed by the `arm_states` stream.
/// The hub anchors trajectories and governor queries on position; the governed
/// velocity feedforward is the commanded step, not a measurement, so the stream's
/// velocities are validated but not retained.
#[derive(Clone, Copy)]
pub struct MeasuredState {
    pub positions: JointVec,
}

/// Warn that a message carried an out-of-range `arm_id`, throttled to at most once
/// per [`UNKNOWN_ARM_WARN_PERIOD`].
fn warn_unknown_arm(stream: &str, arm_id: u8, last: &mut Option<Instant>) {
    let now = Instant::now();
    if last.is_none_or(|t| now.duration_since(t) >= UNKNOWN_ARM_WARN_PERIOD) {
        warn!("{stream}: dropping message for unknown arm_id {arm_id}");
        *last = Some(now);
    }
}

/// Receive `arm_joint_commands` forever, keeping the latest well-formed message per
/// arm in `latest[arm_id]`. A message with any non-finite position is dropped so a
/// producer gone bad lets the follow lock time out instead of driving an arm.
pub async fn run_joint_command_listener(
    runner: Arc<NodeRunner>,
    latest: [watch::Sender<Option<JointCommand>>; 2],
) {
    let mut sub = match commander_arm_joint_commands::subscribe(&runner).await {
        Ok(s) => s,
        Err(e) => return error!("arm_joint_commands subscribe: {e}"),
    };
    let mut seq: [u64; 2] = [0, 0];
    let mut last_unknown_warn: Option<Instant> = None;
    loop {
        let (producer, msg) = match sub.next().await {
            Ok(Some(received)) => received,
            Ok(None) => return, // subscription closed: node shutting down
            Err(e) => {
                error!("arm_joint_commands receive: {e}");
                continue;
            }
        };
        let Some(idx) = Side::from_arm_id(msg.arm_id).map(Side::index) else {
            warn_unknown_arm("arm_joint_commands", msg.arm_id, &mut last_unknown_warn);
            continue;
        };
        if !msg.positions.iter().all(|v| v.is_finite()) {
            warn!(
                "arm_joint_commands: dropping arm {} message with non-finite positions",
                msg.arm_id
            );
            continue;
        }
        seq[idx] += 1;
        latest[idx].send_replace(Some(JointCommand {
            seq: seq[idx],
            producer,
            recv_at: Instant::now(),
            positions: msg.positions,
        }));
    }
}

/// Receive `arm_states` forever, keeping the latest measured state per arm.
/// Non-finite states are dropped so the coordinator never anchors a trajectory or
/// a governor query on a bad measurement.
pub async fn run_joint_state_listener(
    runner: Arc<NodeRunner>,
    latest: [watch::Sender<Option<MeasuredState>>; 2],
) {
    let mut sub = match arm_states_arm_states::subscribe(&runner).await {
        Ok(s) => s,
        Err(e) => return error!("arm_states subscribe: {e}"),
    };
    let mut last_unknown_warn: Option<Instant> = None;
    loop {
        let (_, msg) = match sub.next().await {
            Ok(Some(received)) => received,
            Ok(None) => return,
            Err(e) => {
                error!("arm_states receive: {e}");
                continue;
            }
        };
        let Some(idx) = Side::from_arm_id(msg.arm_id).map(Side::index) else {
            warn_unknown_arm("arm_states", msg.arm_id, &mut last_unknown_warn);
            continue;
        };
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
            continue;
        }
        latest[idx].send_replace(Some(MeasuredState {
            positions: msg.positions,
        }));
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
            Err(e) => error!("governor_control receive: {e}"),
        }
    }
}
