//! Inbound stream plumbing for the backbone: the commander's arm and gripper command
//! streams, both paired arms' measured joint state, both paired grippers'
//! measured aperture, and the runtime governor controls. Each listener holds
//! one subscription and keeps the latest well-formed message in a watch channel
//! the coordinator reads every tick. One held subscription per stream means no
//! re-subscribe gap, so a message is never dropped between receives.

use std::sync::Arc;
use std::time::{Duration, Instant};

use peppygen::NodeRunner;
use peppygen::consumed_topics::collision_ctrl::governor_control as collision_ctrl_governor_control;
use peppygen::consumed_topics::commander::arm_joint_commands as commander_arm_joint_commands;
use peppygen::consumed_topics::commander::gripper_commands as commander_gripper_commands;
use peppygen::paired_topics::{
    left_arm_link, left_gripper_link, right_arm_link, right_gripper_link,
};
use tokio::sync::watch;
use tracing::{error, warn};

use crate::{JointVec, Side};

/// At most one dropped-message warning per stream in this window, so a
/// misrouted or persistently malformed producer is visible in the log
/// without flooding it at the stream rate.
const THROTTLED_WARN_PERIOD: Duration = Duration::from_secs(1);

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
/// `gripper_commands` stream: the raw wire opening fraction, clamped into
/// `[0, 1]` by the coordinator as it applies it, plus the commanded effort
/// cap (`None` when the wire carried no preference).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GripperCommand {
    pub opening: f64,
    pub max_effort: Option<f64>,
}

/// The latest measured joint position for one arm, fed by the joint_link
/// pairing's `joint_states` back-channel. The backbone anchors trajectories and
/// governor queries on position; the governed velocity feedforward is the
/// commanded step, not a measurement, so the stream's velocities are validated
/// but not retained.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MeasuredState {
    pub positions: JointVec,
}

/// The latest measured opening of one gripper, clamped at ingestion into
/// `[0, 1]` (0 = closed, 1 = fully open), so downstream consumers never see an
/// out-of-range placement. Consumed every tick like the measured arm state.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GripperOpening {
    pub fraction: f64,
}

/// Run `emit` at most once per [`THROTTLED_WARN_PERIOD`] per `last` state.
fn warn_throttled(last: &mut Option<Instant>, emit: impl FnOnce()) {
    let now = Instant::now();
    if last.is_none_or(|t| now.duration_since(t) >= THROTTLED_WARN_PERIOD) {
        emit();
        *last = Some(now);
    }
}

/// Warn that a message carried an out-of-range id (`field` names it: `arm_id`
/// or `gripper_id`), throttled to at most once per [`THROTTLED_WARN_PERIOD`].
fn warn_unknown_id(stream: &str, field: &str, id: u8, last: &mut Option<Instant>) {
    warn_throttled(last, || {
        warn!("{stream}: dropping message for unknown {field} {id}");
    });
}

/// Parses one paired arm's inbound joint_states payload. This backbone drives
/// fixed 7-joint arms whose followers always measure velocity, so a message
/// must carry exactly [`crate::ARM_DOF`] positions with matching velocities,
/// all finite; the generic contract's empty-velocities form is deliberately
/// rejected here rather than half-accepted.
fn parse_joint_state(
    positions: Vec<f64>,
    velocities: Vec<f64>,
) -> Result<MeasuredState, &'static str> {
    let finite = positions
        .iter()
        .chain(velocities.iter())
        .all(|v| v.is_finite());
    let velocity_count = velocities.len();
    let Ok(positions) = JointVec::try_from(positions) else {
        return Err("a non-arm joint count");
    };
    if velocity_count != positions.len() {
        return Err("a velocity count that does not match its joints");
    }
    if !finite {
        return Err("non-finite values");
    }
    Ok(MeasuredState { positions })
}

/// Parses one commander gripper command: finite fields, a non-negative effort
/// cap, with the wire's 0 (no preference) parsed to `None`. The opening is
/// clamped later by the coordinator, matching the arm command path.
fn parse_gripper_command(opening: f64, max_effort: f64) -> Result<GripperCommand, &'static str> {
    if !opening.is_finite() || !max_effort.is_finite() {
        return Err("non-finite values");
    }
    if max_effort < 0.0 {
        return Err("a negative max_effort");
    }
    Ok(GripperCommand {
        opening,
        max_effort: (max_effort > 0.0).then_some(max_effort),
    })
}

/// Parses one paired gripper's inbound measured opening: finite, clamped
/// into the contract's `[0, 1]`.
fn parse_gripper_state(opening: f64) -> Result<GripperOpening, &'static str> {
    if !opening.is_finite() {
        return Err("a non-finite opening");
    }
    Ok(GripperOpening {
        fraction: opening.clamp(0.0, 1.0),
    })
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
        match parse_gripper_command(msg.opening, msg.max_effort) {
            Ok(command) => latest[idx].send_replace(Some(command)),
            Err(reason) => {
                warn!(
                    "gripper_commands: dropping gripper {} message with {reason}",
                    msg.gripper_id
                );
                continue;
            }
        };
    }
}

/// Receive both paired arms' measured state forever (the joint_link back-channel),
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
        left_arm_link::joint_states::subscribe(&runner),
        right_arm_link::joint_states::subscribe(&runner),
    );
    let (mut left, mut right) = match (left, right) {
        (Ok(l), Ok(r)) => (l, r),
        (l, r) => {
            return error!(
                "joint_states subscribe: left {:?}, right {:?}",
                l.err(),
                r.err()
            );
        }
    };
    let mut last_reject_warn: Option<Instant> = None;
    loop {
        // Whichever slot delivers next wins the select; the other stays queued in
        // its own subscription, so neither side can starve the other.
        let (side, received) = tokio::select! {
            r = left.next() => (Side::Left, r.map(|m| m.map(|(_, msg)| (msg.positions, msg.velocities)))),
            r = right.next() => (Side::Right, r.map(|m| m.map(|(_, msg)| (msg.positions, msg.velocities)))),
        };
        match received {
            Ok(Some((positions, velocities))) => match parse_joint_state(positions, velocities) {
                Ok(state) => {
                    latest[side.index()].send_replace(Some(state));
                }
                Err(reason) => warn_throttled(&mut last_reject_warn, || {
                    warn!(
                        "joint_states: dropping {} message with {reason}",
                        side.label()
                    );
                }),
            },
            Ok(None) => return, // subscription closed: node shutting down
            Err(e) => {
                error!("joint_states receive ({}): {e}", side.label());
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
/// modeled fingers. A non-finite opening is dropped so the model never places
/// the fingers on a bad reading; the fraction is clamped into `[0, 1]`.
pub async fn run_gripper_state_listener(
    runner: Arc<NodeRunner>,
    latest: [watch::Sender<Option<GripperOpening>>; 2],
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
    let mut last_reject_warn: Option<Instant> = None;
    loop {
        // Whichever slot delivers next wins the select; the other stays queued in
        // its own subscription, so neither side can starve the other.
        let (side, received) = tokio::select! {
            r = left.next() => (Side::Left, r.map(|m| m.map(|(_, msg)| msg.opening))),
            r = right.next() => (Side::Right, r.map(|m| m.map(|(_, msg)| msg.opening))),
        };
        match received {
            Ok(Some(wire_opening)) => match parse_gripper_state(wire_opening) {
                Ok(opening) => {
                    latest[side.index()].send_replace(Some(opening));
                }
                Err(reason) => warn_throttled(&mut last_reject_warn, || {
                    warn!(
                        "gripper_states: dropping {} message with {reason}",
                        side.label()
                    );
                }),
            },
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

#[cfg(test)]
mod tests {
    use super::*;

    use crate::ARM_DOF;

    #[test]
    fn joint_state_requires_arm_dof_positions_with_matching_velocities() {
        let state = parse_joint_state(vec![0.1; ARM_DOF], vec![0.0; ARM_DOF]).unwrap();
        assert_eq!(state.positions, [0.1; ARM_DOF]);
        assert!(parse_joint_state(vec![0.1; ARM_DOF - 1], vec![0.0; ARM_DOF - 1]).is_err());
        assert!(parse_joint_state(vec![0.1; ARM_DOF + 1], vec![0.0; ARM_DOF + 1]).is_err());
        assert!(parse_joint_state(vec![], vec![]).is_err());
        assert!(parse_joint_state(vec![0.1; ARM_DOF], vec![0.0; ARM_DOF - 1]).is_err());
    }

    #[test]
    fn joint_state_rejects_the_contracts_empty_velocities_form() {
        // Deliberate strictness: this backbone's followers always measure
        // velocity, so the generic contract's unmeasured (empty) form is
        // treated as malformed instead of half-accepted.
        assert_eq!(
            parse_joint_state(vec![0.1; ARM_DOF], vec![]),
            Err("a velocity count that does not match its joints")
        );
    }

    #[test]
    fn joint_state_rejects_non_finite_values() {
        let mut positions = vec![0.1; ARM_DOF];
        positions[3] = f64::NAN;
        assert_eq!(
            parse_joint_state(positions, vec![0.0; ARM_DOF]),
            Err("non-finite values")
        );
        let mut velocities = vec![0.0; ARM_DOF];
        velocities[6] = f64::INFINITY;
        assert_eq!(
            parse_joint_state(vec![0.1; ARM_DOF], velocities),
            Err("non-finite values")
        );
    }

    #[test]
    fn gripper_state_clamps_into_the_contract_range() {
        assert_eq!(parse_gripper_state(0.5).unwrap().fraction, 0.5);
        assert_eq!(parse_gripper_state(-0.2).unwrap().fraction, 0.0);
        assert_eq!(parse_gripper_state(1.7).unwrap().fraction, 1.0);
        assert!(parse_gripper_state(f64::NAN).is_err());
    }

    #[test]
    fn gripper_command_parses_the_effort_cap() {
        let command = parse_gripper_command(0.5, 1.5).unwrap();
        assert_eq!(command.opening, 0.5);
        assert_eq!(command.max_effort, Some(1.5));
        // The wire's 0 means no preference, not a zero-force cap.
        assert_eq!(parse_gripper_command(0.5, 0.0).unwrap().max_effort, None);
    }

    #[test]
    fn gripper_command_rejects_malformed_fields() {
        assert_eq!(
            parse_gripper_command(f64::NAN, 0.0),
            Err("non-finite values")
        );
        assert_eq!(
            parse_gripper_command(0.5, f64::INFINITY),
            Err("non-finite values")
        );
        assert_eq!(
            parse_gripper_command(0.5, -1.0),
            Err("a negative max_effort")
        );
    }
}
