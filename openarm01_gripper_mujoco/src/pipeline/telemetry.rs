// SimBridge pipelines for this gripper side: re-emit gripper_state + ee_pose
// as typed peppygen, partition world contact_forces by finger-body prefix.
// Side suffix lives only on the raw peppylib channel; bus topics are flat.

use std::sync::Arc;

use peppygen::NodeRunner;
use peppygen::emitted_topics::{
    contact_forces_finger1, contact_forces_finger2, ee_pose, gripper_state,
};
use peppylib::TopicPublisher;
use serde::Deserialize;
use sim_bridge_core::{BoxFuture, SimBridge};
use tracing::{error, info, warn};

use crate::config::GripperId;
use crate::state::{GripperStateLatest, SharedState};
use crate::transport::PeppylibTransport;

const ROBOT_NAME: &str = "openarm";

#[derive(Debug, Clone, Deserialize)]
struct GripperStateRaw {
    #[allow(dead_code)]
    robot: String,
    step: u64,
    joint_names: Vec<String>,
    positions: Vec<f64>,
    // applied_forces is emitted by sim_ext_core.GripperStateBridge but not in
    // the gripper's typed contract — accept the field so deserialization
    // doesn't fail, then drop it.
    #[serde(default)]
    #[allow(dead_code)]
    applied_forces: Vec<f64>,
    stamp: f64,
}

#[derive(Debug, Clone, Deserialize)]
struct EePoseRaw {
    #[allow(dead_code)]
    robot: String,
    step: u64,
    position: [f64; 3],
    orientation: [f64; 4],
    stamp: f64,
}

#[derive(Debug, Clone, Deserialize)]
struct ContactRaw {
    body1: String,
    body2: String,
    position: [f64; 3],
    force: [f64; 3],
}

#[derive(Debug, Clone, Deserialize)]
struct ContactForcesRaw {
    #[allow(dead_code)]
    robot: String,
    step: u64,
    contacts: Vec<ContactRaw>,
    stamp: f64,
}

pub async fn run(
    runner: Arc<NodeRunner>,
    gripper_id: GripperId,
    state: Arc<SharedState>,
    transport: Arc<PeppylibTransport>,
) {
    let side = gripper_id.side_word();
    info!(
        "telemetry: starting pipelines (gripper_id={} side={})",
        gripper_id.as_u8(),
        side
    );

    // sim_node = the publisher node name configured in
    // robot_initializer_mujoco's bridge_extension (defaults to "sim").
    let sim_node: Arc<str> = Arc::from("sim");

    // sim_bridge_core takes a tokio_util token; cancel it from an on_shutdown
    // hook so the bridge tears down cleanly before the runtime drops.
    let token = tokio_util::sync::CancellationToken::new();
    {
        let bridge_token = token.clone();
        runner.on_shutdown(async move { bridge_token.cancel() });
    }

    let gripper_state_topic: Arc<str> = Arc::from(format!("gripper_state_{side}"));
    let ee_pose_topic: Arc<str> = Arc::from(format!("ee_pose_{side}"));
    let contact_topic: Arc<str> = Arc::from("contact_forces");

    // Body-name prefixes used to split world-wide contacts into per-finger
    // topics. Matches the openarm MJCF naming: openarm_<side>_right_finger*
    // bodies belong to finger1, openarm_<side>_left_finger* to finger2.
    let finger1_prefix: Arc<str> = Arc::from(format!("openarm_{side}_right_finger").as_str());
    let finger2_prefix: Arc<str> = Arc::from(format!("openarm_{side}_left_finger").as_str());

    // Declare each topic publisher once; the per-message closures clone the
    // lock-free handle and publish on it.
    let gs_pub = match gripper_state::declare_publisher(&runner).await {
        Ok(publisher) => publisher,
        Err(e) => {
            error!("declare gripper_state publisher: {e}");
            return;
        }
    };
    let ee_pub = match ee_pose::declare_publisher(&runner).await {
        Ok(publisher) => publisher,
        Err(e) => {
            error!("declare ee_pose publisher: {e}");
            return;
        }
    };
    let f1_pub = match contact_forces_finger1::declare_publisher(&runner).await {
        Ok(publisher) => publisher,
        Err(e) => {
            error!("declare contact_forces_finger1 publisher: {e}");
            return;
        }
    };
    let f2_pub = match contact_forces_finger2::declare_publisher(&runner).await {
        Ok(publisher) => publisher,
        Err(e) => {
            error!("declare contact_forces_finger2 publisher: {e}");
            return;
        }
    };

    let state_for_gs = state.clone();

    let bridge = SimBridge::new(runner.clone(), transport, token, sim_node)
        .sim_to_os(
            gripper_state_topic,
            move |_runner, msg: GripperStateRaw| -> BoxFuture<std::result::Result<(), String>> {
                let state = state_for_gs.clone();
                let publisher = gs_pub.clone();
                Box::pin(async move {
                    // Cache for the action handler's feedback loop.
                    {
                        let mut latest = state
                            .gripper_state
                            .lock()
                            .unwrap_or_else(|p| p.into_inner());
                        *latest = Some(GripperStateLatest {
                            step: msg.step,
                            positions: msg.positions.clone(),
                            stamp: msg.stamp,
                        });
                    }
                    emit_gripper_state(&publisher, &msg).await
                })
            },
        )
        .sim_to_os(
            ee_pose_topic,
            move |_runner, msg: EePoseRaw| -> BoxFuture<std::result::Result<(), String>> {
                let publisher = ee_pub.clone();
                Box::pin(async move { emit_ee_pose(&publisher, &msg).await })
            },
        )
        .sim_to_os(
            contact_topic,
            move |_runner, msg: ContactForcesRaw| -> BoxFuture<std::result::Result<(), String>> {
                let f1_prefix = finger1_prefix.clone();
                let f2_prefix = finger2_prefix.clone();
                let f1_pub = f1_pub.clone();
                let f2_pub = f2_pub.clone();
                Box::pin(async move {
                    emit_contact_forces(&f1_pub, &f2_pub, &msg, &f1_prefix, &f2_prefix).await
                })
            },
        );

    bridge.run().await;
    info!("telemetry: pipelines exited");
}

fn stamp_now_secs() -> f64 {
    match peppygen::clock::now_ns() {
        Ok(ns) => ns as f64 / 1e9,
        Err(e) => {
            warn!("peppygen::clock::now_ns failed ({e}); stamping with 0.0");
            0.0
        }
    }
}

async fn emit_gripper_state(
    publisher: &TopicPublisher,
    msg: &GripperStateRaw,
) -> std::result::Result<(), String> {
    let payload = gripper_state::build_message(
        ROBOT_NAME.into(),
        msg.step,
        msg.joint_names.clone(),
        msg.positions.clone(),
        stamp_now_secs(),
    )
    .map_err(|e| e.to_string())?;
    publisher.publish(payload).await.map_err(|e| e.to_string())
}

async fn emit_ee_pose(
    publisher: &TopicPublisher,
    msg: &EePoseRaw,
) -> std::result::Result<(), String> {
    let payload = ee_pose::build_message(
        ROBOT_NAME.into(),
        msg.step,
        msg.position,
        msg.orientation,
        stamp_now_secs(),
    )
    .map_err(|e| e.to_string())?;
    publisher.publish(payload).await.map_err(|e| e.to_string())
}

async fn emit_contact_forces(
    f1_pub: &TopicPublisher,
    f2_pub: &TopicPublisher,
    msg: &ContactForcesRaw,
    finger1_prefix: &str,
    finger2_prefix: &str,
) -> std::result::Result<(), String> {
    let (f1, f2) = partition_contacts(&msg.contacts, finger1_prefix, finger2_prefix);

    // Skip empty payloads on hot telemetry ticks — at 50-1000 Hz × 2 sides ×
    // 2 fingers, unconditional emit would flood subscribers with "contacts: []".
    let stamp = stamp_now_secs();
    if !f1.is_empty() {
        let payload =
            contact_forces_finger1::build_message(ROBOT_NAME.into(), msg.step, to_f1(&f1), stamp)
                .map_err(|e| e.to_string())?;
        f1_pub.publish(payload).await.map_err(|e| e.to_string())?;
    }
    if !f2.is_empty() {
        let payload =
            contact_forces_finger2::build_message(ROBOT_NAME.into(), msg.step, to_f2(&f2), stamp)
                .map_err(|e| e.to_string())?;
        f2_pub.publish(payload).await.map_err(|e| e.to_string())?;
    }
    Ok(())
}

fn partition_contacts(
    all: &[ContactRaw],
    finger1_prefix: &str,
    finger2_prefix: &str,
) -> (Vec<ContactRaw>, Vec<ContactRaw>) {
    let mut f1 = Vec::new();
    let mut f2 = Vec::new();
    for c in all {
        let touches_finger1 =
            c.body1.starts_with(finger1_prefix) || c.body2.starts_with(finger1_prefix);
        let touches_finger2 =
            c.body1.starts_with(finger2_prefix) || c.body2.starts_with(finger2_prefix);
        if touches_finger1 {
            f1.push(c.clone());
        }
        if touches_finger2 {
            f2.push(c.clone());
        }
    }
    (f1, f2)
}

// MessageContactsItem is a per-topic type; the two `to_*` helpers exist so
// the partition logic above stays codec-agnostic and the per-topic codec
// layout stays one-to-one with the contract.
fn to_f1(snaps: &[ContactRaw]) -> Vec<contact_forces_finger1::MessageContactsItem> {
    snaps
        .iter()
        .map(|c| contact_forces_finger1::MessageContactsItem {
            body1: c.body1.clone(),
            body2: c.body2.clone(),
            position: c.position,
            force: c.force,
        })
        .collect()
}

fn to_f2(snaps: &[ContactRaw]) -> Vec<contact_forces_finger2::MessageContactsItem> {
    snaps
        .iter()
        .map(|c| contact_forces_finger2::MessageContactsItem {
            body1: c.body1.clone(),
            body2: c.body2.clone(),
            position: c.position,
            force: c.force,
        })
        .collect()
}
