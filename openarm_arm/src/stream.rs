//! Stream plumbing for the real-arm follower: a listener that keeps the latest
//! governed setpoint from the paired hub (the `hub` slot of openarm_arm_link)
//! in a watch channel for the control loop, and a publisher that emits the
//! measured joint state at a fixed rate, both to the paired hub on the
//! pairing's `arm_states` (the exclusive command loop the governor anchors on)
//! and to observers on the broadcast stream (tagged with `arm_id`). All run for
//! the life of the node.

use std::sync::Arc;
use std::time::Duration;

use peppygen::NodeRunner;
use peppygen::emitted_topics::openarm_arm_states::v1::arm_states;
use peppygen::pairings::hub;
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
/// `arm_states` emitter, as wire arrays.
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

/// Receive the paired hub's `arm_setpoints` forever, folding each message into
/// `latest` via [`apply_setpoint`]. The slot delivers only the paired peer's
/// messages, so there is no arm_id filter; subscribing while unpaired is legal
/// (the slot stays silent until a hub pairs). One held subscription, looped: no
/// re-subscribe gap, so a setpoint is never dropped between receives.
pub async fn run_governed_setpoint_listener(
    runner: Arc<NodeRunner>,
    latest: watch::Sender<Option<GovernedSetpoint>>,
) {
    let mut sub = match hub::arm_setpoints::subscribe(&runner).await {
        Ok(s) => s,
        Err(e) => return error!("arm_setpoints subscribe: {e}"),
    };
    loop {
        let msg = match sub.next().await {
            Ok(Some((_, msg))) => msg,
            Ok(None) => return, // subscription closed: node shutting down
            Err(e) => {
                error!("arm_setpoints receive: {e}");
                continue;
            }
        };
        apply_setpoint(&latest, msg.positions, msg.velocities);
    }
}

/// Fold one received setpoint into the watch: clear the target on any
/// non-finite value (so the control loop holds measured pose rather than
/// tracking a stale target), otherwise publish it.
fn apply_setpoint(
    latest: &watch::Sender<Option<GovernedSetpoint>>,
    positions: JointVec,
    velocities: JointVec,
) {
    if !positions
        .iter()
        .chain(velocities.iter())
        .all(|v| v.is_finite())
    {
        warn!("arm_setpoints: clearing target on non-finite values");
        latest.send_replace(None);
        return;
    }
    latest.send_replace(Some(GovernedSetpoint {
        q_des: positions,
        dq_des: velocities,
    }));
}

/// Emit the measured joint state at a fixed rate, forever: to the paired hub on
/// the pairing's `arm_states` (the command loop's state input) and to observers
/// on the broadcast stream (tagged with `arm_id`). The two publishes serve
/// unrelated consumers, so each reports failures independently. The watch starts
/// empty and is first filled by the control loop's first tick, so nothing is
/// published before a real measurement exists. The loop exits if the control
/// task drops the sender, so the stream goes silent rather than republishing a
/// frozen final measurement.
pub async fn run_state_publisher(
    runner: Arc<NodeRunner>,
    arm_id: u8,
    period: Duration,
    mut measured: watch::Receiver<Option<MeasuredState>>,
) {
    let joint_pub = match arm_states::declare_publisher(&runner).await {
        Ok(p) => p,
        Err(e) => return error!("declare arm_states publisher: {e}"),
    };
    let peer_pub = match hub::arm_states::declare_publisher(&runner).await {
        Ok(p) => p,
        Err(e) => return error!("declare paired arm_states publisher: {e}"),
    };
    if measured.wait_for(Option::is_some).await.is_err() {
        return; // control loop gone: node is shutting down
    }
    let mut pacer =
        Pacer::new(period).expect("state publish period is non-zero (derives from state_rate_hz)");
    let mut broadcast_failing = false;
    let mut peer_failing = false;
    loop {
        if measured.has_changed().is_err() {
            return;
        }
        let m = (*measured.borrow()).expect("gated on first measurement");
        let broadcast_result = async {
            let joints = arm_states::build_message(arm_id, m.positions, m.velocities)
                .map_err(|e| e.to_string())?;
            joint_pub.publish(joints).await.map_err(|e| e.to_string())?;
            Ok::<(), String>(())
        }
        .await;
        match broadcast_result {
            Ok(()) => broadcast_failing = false,
            Err(e) if !broadcast_failing => {
                broadcast_failing = true;
                warn!("state publish failing, suppressing repeats: {e}");
            }
            Err(_) => {}
        }
        let peer_result = async {
            let joints = hub::arm_states::build_message(m.positions, m.velocities)
                .map_err(|e| e.to_string())?;
            peer_pub.publish(joints).await.map_err(|e| e.to_string())?;
            Ok::<(), String>(())
        }
        .await;
        match peer_result {
            Ok(()) => peer_failing = false,
            Err(e) if !peer_failing => {
                peer_failing = true;
                warn!("paired state publish failing, suppressing repeats: {e}");
            }
            Err(_) => {}
        }
        pacer.pace().await;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn non_finite_setpoint_clears_the_target() {
        let (tx, rx) = watch::channel::<Option<GovernedSetpoint>>(None);
        apply_setpoint(&tx, [0.1; crate::ARM_DOF], [0.0; crate::ARM_DOF]);
        assert!(rx.borrow().is_some(), "valid setpoint should be published");
        // A non-finite value in either positions or velocities clears the target so
        // the control loop holds; a valid setpoint after a clear republishes.
        let mut bad_pos = [0.0; crate::ARM_DOF];
        bad_pos[0] = f64::NAN;
        apply_setpoint(&tx, bad_pos, [0.0; crate::ARM_DOF]);
        assert!(
            rx.borrow().is_none(),
            "non-finite position must clear the target"
        );

        apply_setpoint(&tx, [0.1; crate::ARM_DOF], [0.0; crate::ARM_DOF]);
        assert!(
            rx.borrow().is_some(),
            "valid setpoint must republish after a clear"
        );
        let mut bad_vel = [0.0; crate::ARM_DOF];
        bad_vel[3] = f64::INFINITY;
        apply_setpoint(&tx, [0.1; crate::ARM_DOF], bad_vel);
        assert!(
            rx.borrow().is_none(),
            "non-finite velocity must clear the target"
        );
    }
}
