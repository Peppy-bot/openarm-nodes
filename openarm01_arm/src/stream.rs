//! Stream plumbing for the real-arm follower: a listener that keeps the latest
//! governed setpoint addressed to this arm (from the hub's `governed_setpoints`)
//! in a watch channel for the control loop, and a publisher that emits the
//! measured joint state on `joint_states` at a fixed rate (the hub consumes it to
//! anchor trajectories and run the governor). All run for the life of the node.

use std::sync::Arc;
use std::time::Duration;

use peppygen::NodeRunner;
use peppygen::consumed_topics::hub_arm_governed_setpoints;
use peppygen::emitted_topics::openarm01_arm_states::v1::arm_states;
use tokio::sync::watch;
use tracing::{error, warn};

use control_core::Pacer;

use crate::JointVec;

/// The latest governed setpoint for this arm: the position/velocity the MIT loop
/// tracks. Produced by the hub, already collision-governed and rate-limited.
#[derive(Clone, Copy)]
pub struct GovernedSetpoint {
    pub q_des: JointVec,
    pub dq_des: JointVec,
}

/// Measured joint state the control loop publishes each tick for the
/// `joint_states` emitter, as wire arrays.
#[derive(Clone, Copy)]
pub struct MeasuredState {
    pub positions: JointVec,
    pub velocities: JointVec,
}

/// The control loop's connections to the stream tasks: the inbound governed
/// setpoint (latest) and the outbound measured-state channel feeding the publisher.
pub struct StreamWiring {
    pub governed: watch::Receiver<Option<GovernedSetpoint>>,
    pub measured: watch::Sender<Option<MeasuredState>>,
}

/// Receive `governed_setpoints` forever, keeping the latest well-formed setpoint
/// addressed to this arm in `latest`. One held subscription, looped: there is no
/// re-subscribe gap between messages, so a setpoint for this arm is never dropped
/// while the other arm's message is in flight. A setpoint with any non-finite
/// value is dropped (the loop then holds), so a producer gone bad cannot drive
/// the arm.
pub async fn run_governed_setpoint_listener(
    runner: Arc<NodeRunner>,
    arm_id: u8,
    latest: watch::Sender<Option<GovernedSetpoint>>,
) {
    let mut sub = match hub_arm_governed_setpoints::subscribe(&runner).await {
        Ok(s) => s,
        Err(e) => return error!("governed_setpoints subscribe: {e}"),
    };
    loop {
        let msg = match sub.next().await {
            Ok(Some((_, msg))) => msg,
            Ok(None) => return, // subscription closed: node shutting down
            Err(e) => {
                error!("governed_setpoints receive: {e}");
                continue;
            }
        };
        if msg.arm_id != arm_id {
            continue;
        }
        let finite = msg.positions.iter().chain(msg.velocities.iter()).all(|v| v.is_finite());
        if !finite {
            warn!("governed_setpoints: dropping message with non-finite values");
            continue;
        }
        latest.send_replace(Some(GovernedSetpoint { q_des: msg.positions, dq_des: msg.velocities }));
    }
}

/// Emit the measured joint state at a fixed rate, forever. The publisher is
/// declared once and reused. The watch starts empty and is first filled by the
/// control loop's first tick, so nothing is published before a real measurement
/// exists. The loop exits if the control task drops the sender, so the stream
/// goes silent rather than republishing a frozen final measurement.
pub async fn run_state_publisher(
    runner: Arc<NodeRunner>,
    arm_id: u8,
    period: Duration,
    mut measured: watch::Receiver<Option<MeasuredState>>,
) {
    let joint_pub = match arm_states::declare_publisher(&runner).await {
        Ok(p) => p,
        Err(e) => return error!("declare joint_states publisher: {e}"),
    };
    if measured.wait_for(Option::is_some).await.is_err() {
        return; // control loop gone: node is shutting down
    }
    let mut pacer = Pacer::new(period).expect("state publish period is non-zero (derives from state_rate_hz)");
    let mut failing = false;
    loop {
        if measured.has_changed().is_err() {
            return;
        }
        let m = (*measured.borrow()).expect("gated on first measurement");
        let result = async {
            let joints = arm_states::build_message(arm_id, m.positions, m.velocities).map_err(|e| e.to_string())?;
            joint_pub.publish(joints).await.map_err(|e| e.to_string())?;
            Ok::<(), String>(())
        }
        .await;
        match result {
            Ok(()) => failing = false,
            Err(e) if !failing => {
                failing = true;
                warn!("state publish failing, suppressing repeats: {e}");
            }
            Err(_) => {}
        }
        pacer.pace().await;
    }
}
