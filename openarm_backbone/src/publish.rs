//! Every publisher the backbone owns, and the one path that stamps, builds,
//! publishes and reports a message. (Peppy vocabulary throughout: a *publisher*
//! sends on a topic; a pairing *slot* carries one direction of a pairing's two
//! one-way streams to its one *peer*; "wire" below means the encoded message
//! on the transport, nothing else.)
//!
//! Each pairing slot is its own generated module, so two slots carrying the
//! same schema expose two distinct `build_message` items. Holding the builder
//! as a function pointer lets one [`Publisher`] serve both sides of a pairing,
//! and the send path is written once instead of once per slot.
//!
//! Publishing on an unpaired slot is a legal no-op, so the backbone declares
//! every slot at bringup and publishes regardless; a follower simply starts
//! tracking once its pair is established. A slot that cannot be declared at
//! all is a bringup fault and takes the node down.

use std::future::Future;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use peppygen::NodeRunner;
use peppygen::emitted_topics::collision_status;
use peppygen::paired_topics::{
    leader_left_arm, leader_left_gripper, leader_right_arm, leader_right_gripper, left_arm_link,
    left_gripper_link, right_arm_link, right_gripper_link,
};
use peppylib::{Payload, TopicPublisher};
use tracing::{error, warn};

use crate::streams::GripperOpening;
use crate::{ArmPair, JointVec};

/// Pairing stamp from the daemon-resolved clock (sim time under a simulated
/// clock), so consumers age samples on the same timeline they read. Errors
/// until the clock delivers its first tick.
fn pairing_stamp() -> Result<SystemTime, String> {
    let ns = peppygen::clock::now_ns().map_err(|e| format!("clock not ready: {e}"))?;
    Ok(UNIX_EPOCH + Duration::from_nanos(ns))
}

/// A joint vector slot: `joint_setpoints` downstream and `joint_states`
/// upstream share this schema.
type JointBuild = fn(SystemTime, Vec<f64>, Vec<f64>, Vec<f64>) -> peppygen::Result<Payload>;

/// A commanded aperture: the opening fraction and the effort cap to relay.
type OpeningBuild = fn(SystemTime, f64, f64) -> peppygen::Result<Payload>;

/// A measured aperture: the opening fraction, the measured effort, and the
/// follower's effort ceiling.
type ApertureBuild = fn(SystemTime, f64, f64, f64) -> peppygen::Result<Payload>;

/// One outbound slot: its declared publisher, that slot's generated
/// `build_message`, and the phrase naming it in a log line.
pub struct Publisher<Build> {
    publisher: TopicPublisher,
    build: Build,
    what: &'static str,
}

impl<Build> Publisher<Build> {
    async fn declare(
        what: &'static str,
        declaring: impl Future<Output = peppygen::Result<TopicPublisher>>,
        build: Build,
    ) -> peppygen::Result<Self> {
        Ok(Self {
            publisher: declare(what, declaring).await?,
            build,
            what,
        })
    }
}

impl Publisher<JointBuild> {
    /// Publish one limb's joint vector. Efforts ride empty: this backbone
    /// neither commands nor measures them, which the contract spells as an
    /// empty list rather than a vector of zeros.
    pub async fn send(&self, positions: &JointVec, velocities: &JointVec) {
        self.emit(|stamp| (self.build)(stamp, positions.to_vec(), velocities.to_vec(), Vec::new()))
            .await;
    }
}

impl Publisher<OpeningBuild> {
    /// Publish one gripper's governed opening fraction and the effort cap to
    /// relay (`None` rides as the wire's 0: no preference, leaving the
    /// follower's configured ceiling in charge).
    pub async fn send(&self, opening: f64, max_effort: Option<f64>) {
        self.emit(|stamp| (self.build)(stamp, opening, max_effort.unwrap_or(0.0)))
            .await;
    }
}

impl Publisher<ApertureBuild> {
    /// Relay one gripper's measured state as its follower reported it.
    pub async fn send(&self, measured: &GripperOpening) {
        self.emit(|stamp| {
            (self.build)(
                stamp,
                measured.fraction,
                measured.effort,
                measured.max_effort,
            )
        })
        .await;
    }
}

impl<Build> Publisher<Build> {
    /// Stamp, build and publish, naming the slot in either failure. A publish
    /// error is a transient wire condition; a build error (or a clock that has
    /// not ticked) means the message was never formed. Neither is fatal: the
    /// next tick tries again.
    async fn emit(&self, build: impl FnOnce(SystemTime) -> peppygen::Result<Payload>) {
        match pairing_stamp().and_then(|stamp| build(stamp).map_err(|e| e.to_string())) {
            Ok(msg) => {
                if let Err(e) = self.publisher.publish(msg).await {
                    warn!("{} publish: {e}", self.what);
                }
            }
            Err(e) => error!("{} build: {e}", self.what),
        }
    }
}

/// Every publisher the coordination loop owns, declared once at bringup.
pub struct Publishers {
    /// The governed joint setpoints, one per paired arm.
    pub setpoints: ArmPair<Publisher<JointBuild>>,
    /// The governed opening fractions, one per paired gripper.
    pub openings: ArmPair<Publisher<OpeningBuild>>,
    /// Each arm's measured state, relayed up its leader slot so the leading
    /// node sees the same back-channel a follower gives the backbone.
    pub arm_states: ArmPair<Publisher<JointBuild>>,
    /// Each gripper's measured state, relayed up its leader slot.
    pub gripper_states: ArmPair<Publisher<ApertureBuild>>,
    /// The operator readout: an emitted topic rather than a pairing slot, and
    /// the one message with no stamp of its own.
    status: TopicPublisher,
}

impl Publishers {
    pub async fn declare(runner: &NodeRunner) -> peppygen::Result<Self> {
        Ok(Self {
            setpoints: ArmPair::new(
                Publisher::declare(
                    "left joint_setpoints",
                    left_arm_link::joint_setpoints::declare_publisher(runner),
                    left_arm_link::joint_setpoints::build_message as JointBuild,
                )
                .await?,
                Publisher::declare(
                    "right joint_setpoints",
                    right_arm_link::joint_setpoints::declare_publisher(runner),
                    right_arm_link::joint_setpoints::build_message as JointBuild,
                )
                .await?,
            ),
            openings: ArmPair::new(
                Publisher::declare(
                    "left gripper_setpoints",
                    left_gripper_link::gripper_setpoints::declare_publisher(runner),
                    left_gripper_link::gripper_setpoints::build_message as OpeningBuild,
                )
                .await?,
                Publisher::declare(
                    "right gripper_setpoints",
                    right_gripper_link::gripper_setpoints::declare_publisher(runner),
                    right_gripper_link::gripper_setpoints::build_message as OpeningBuild,
                )
                .await?,
            ),
            arm_states: ArmPair::new(
                Publisher::declare(
                    "upstream left joint_states",
                    leader_left_arm::joint_states::declare_publisher(runner),
                    leader_left_arm::joint_states::build_message as JointBuild,
                )
                .await?,
                Publisher::declare(
                    "upstream right joint_states",
                    leader_right_arm::joint_states::declare_publisher(runner),
                    leader_right_arm::joint_states::build_message as JointBuild,
                )
                .await?,
            ),
            gripper_states: ArmPair::new(
                Publisher::declare(
                    "upstream left gripper_states",
                    leader_left_gripper::gripper_states::declare_publisher(runner),
                    leader_left_gripper::gripper_states::build_message as ApertureBuild,
                )
                .await?,
                Publisher::declare(
                    "upstream right gripper_states",
                    leader_right_gripper::gripper_states::declare_publisher(runner),
                    leader_right_gripper::gripper_states::build_message as ApertureBuild,
                )
                .await?,
            ),
            status: declare(
                "collision_status",
                collision_status::declare_publisher(runner),
            )
            .await?,
        })
    }

    /// Publish the operator readout: the nearest checked pair's signed distance
    /// and link names, plus the governor's disposition of the commanded motion.
    pub async fn send_status(
        &self,
        distance: f64,
        link_a: String,
        link_b: String,
        throttling: bool,
        stopped: bool,
    ) {
        match collision_status::build_message(distance, link_a, link_b, throttling, stopped) {
            Ok(msg) => {
                if let Err(e) = self.status.publish(msg).await {
                    warn!("collision_status publish: {e}");
                }
            }
            Err(e) => error!("collision_status build: {e}"),
        }
    }
}

/// Declare one publisher, naming it in the error.
async fn declare(
    what: &str,
    declaring: impl Future<Output = peppygen::Result<TopicPublisher>>,
) -> peppygen::Result<TopicPublisher> {
    declaring
        .await
        .inspect_err(|e| error!("declare {what} publisher: {e}"))
}
