//! Plumbing for the paired command loop and the observer stream: a listener
//! task that keeps the latest well-formed `joint_commands` setpoint from the
//! paired commander (the `commander` pairing slot) in a watch channel for the
//! control loop's `Follow` mode, and a publisher task that emits the measured
//! joint state at a fixed rate — back to the paired commander on the pairing's
//! `joint_states` topic and to observers on the broadcast
//! `openarm01_joint_state_source` stream. The arm follows joints only; a
//! task-space commander solves IK upstream and streams joints. All run for the
//! life of the node.

use std::sync::Arc;
use std::time::{Duration, Instant};

use peppygen::NodeRunner;
use peppygen::emitted_topics::openarm01_joint_state_source::v1::joint_states;
use peppygen::peers::commander::joint_commands as peer_joint_commands;
use peppygen::peers::commander::joint_states as peer_joint_states;
use peppylib::messaging::ProducerRef;
use tokio::sync::watch;
use tracing::{error, warn};

use crate::JointVec;
use crate::pacer::Pacer;

/// The latest streamed joint setpoint, as the control loop sees it. `seq`
/// (incremented per accepted message) distinguishes a fresh command from one
/// already acted on; `producer` is the paired commander's wire address (the
/// pairing guarantees a single source, so `Follow`'s lock is only ever held by
/// the peer); `recv_at` is the arrival time the stream watchdog uses to tell a
/// live stream from a stale leftover in the watch channel.
#[derive(Clone)]
pub struct JointCommand {
    pub seq: u64,
    pub producer: ProducerRef,
    pub recv_at: Instant,
    pub positions: JointVec,
}

/// Measured joint state the control loop publishes each tick for the `joint_states`
/// emitter, as wire arrays.
#[derive(Clone, Copy)]
pub struct MeasuredState {
    pub positions: JointVec,
    pub velocities: JointVec,
}

/// The control loop's connections to the stream tasks: the inbound joint-setpoint
/// channel (latest command) and the outbound measured-state channel feeding the
/// publisher. Built in `main`, consumed by the control loop.
pub struct StreamWiring {
    pub joint: watch::Receiver<Option<JointCommand>>,
    pub measured: watch::Sender<Option<MeasuredState>>,
}

/// Receive `joint_commands` from the paired commander forever, keeping only the
/// latest well-formed message in `latest`. Subscribing while unpaired is legal:
/// the subscription stays silent until a commander pairs, and only the paired
/// peer's messages ever surface, so there is no arm_id filter. A message with
/// any non-finite position is dropped and never reaches the control loop, so a
/// commander gone bad lets the stream watchdog time out instead of driving the
/// arm.
pub async fn run_joint_command_listener(
    runner: Arc<NodeRunner>,
    latest: watch::Sender<Option<JointCommand>>,
) {
    let mut subscription = match peer_joint_commands::subscribe(&runner).await {
        Ok(subscription) => subscription,
        Err(e) => {
            error!("joint_commands subscribe: {e}");
            return;
        }
    };
    let mut seq: u64 = 0;
    loop {
        let (producer, msg) = match subscription.next().await {
            Ok(Some(received)) => received,
            Ok(None) => return,
            Err(e) => {
                error!("joint_commands receive: {e}");
                continue;
            }
        };
        if !msg.positions.iter().all(|v| v.is_finite()) {
            warn!("joint_commands: dropping message with non-finite positions");
            continue;
        }
        seq += 1;
        latest.send_replace(Some(JointCommand {
            seq,
            producer,
            recv_at: Instant::now(),
            positions: msg.positions,
        }));
    }
}

/// Emit the measured joint state at a fixed rate, forever — on the pairing's
/// `joint_states` topic for the paired commander (a legal no-op while
/// unpaired) and on the broadcast observer stream (tagged with `arm_id`).
/// Publishers are declared once and reused. The watch starts empty and is
/// first filled by the control loop's first tick, so nothing is published
/// before a real measurement exists. The loop exits if the control task drops
/// the sender (it has died), so the streams go silent rather than
/// republishing a frozen final measurement. Publish failures are logged once
/// per failure streak rather than at the emit rate.
pub async fn run_state_publisher(
    runner: Arc<NodeRunner>,
    arm_id: u8,
    period: Duration,
    mut measured: watch::Receiver<Option<MeasuredState>>,
) {
    let joint_pub = match joint_states::declare_publisher(&runner).await {
        Ok(p) => p,
        Err(e) => return error!("declare joint_states publisher: {e}"),
    };
    let peer_pub = match peer_joint_states::declare_publisher(&runner).await {
        Ok(p) => p,
        Err(e) => return error!("declare paired joint_states publisher: {e}"),
    };
    if measured.wait_for(Option::is_some).await.is_err() {
        return; // control loop gone: node is shutting down
    }
    let mut pacer = Pacer::new(period);
    let mut failing = false;
    loop {
        if measured.has_changed().is_err() {
            return; // control task dropped the sender: stop emitting instead of going stale
        }
        let m = (*measured.borrow()).expect("gated on first measurement");
        let result = async {
            let joints = joint_states::build_message(arm_id, m.positions, m.velocities)
                .map_err(|e| e.to_string())?;
            joint_pub.publish(joints).await.map_err(|e| e.to_string())?;
            let peer_joints = peer_joint_states::build_message(m.positions, m.velocities)
                .map_err(|e| e.to_string())?;
            peer_pub.publish(peer_joints).await.map_err(|e| e.to_string())?;
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
