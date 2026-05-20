use std::sync::Arc;
use std::time::Duration;

use peppygen::NodeRunner;
use peppygen::emitted_topics::{
    contact_forces_left_finger1, contact_forces_left_finger2,
    contact_forces_right_finger1, contact_forces_right_finger2,
    ee_pose_left, ee_pose_right,
    gripper_state_left, gripper_state_right,
};
use tracing::warn;

use crate::config::{GripperId, ResolvedSide};
use crate::drivers::mjdata_bus::{ContactSnap, MjDataBus};

const ROBOT_NAME: &str = "openarm";
const TELEMETRY_HZ: u32 = 50;

pub async fn run(
    runner: Arc<NodeRunner>,
    bus: Arc<MjDataBus>,
    side: Arc<ResolvedSide>,
    gripper_id: GripperId,
) {
    let period = Duration::from_micros(1_000_000 / TELEMETRY_HZ as u64);
    let mut step: u64 = 0;
    let all_finger_geoms = side.all_finger_geom_ids();

    loop {
        let snap = match bus.snapshot(&side.finger_qpos_addrs, side.ee_body_id, &all_finger_geoms) {
            Ok(s) => s,
            Err(e) => {
                warn!("telemetry snapshot: {e}");
                tokio::time::sleep(period).await;
                continue;
            }
        };

        let stamp = snap.sim_time;
        let (f1, f2) = partition_contacts(
            &snap.contacts,
            &side.finger1_geom_ids,
            &side.finger2_geom_ids,
        );

        if gripper_id.0 == 0 {
            emit_left(&runner, step, stamp, &snap, side.joint_names.clone(), &f1, &f2).await;
        } else {
            emit_right(&runner, step, stamp, &snap, side.joint_names.clone(), &f1, &f2).await;
        }

        step += 1;
        tokio::time::sleep(period).await;
    }
}

fn partition_contacts(
    all: &[ContactSnap],
    f1_geoms: &[u32],
    f2_geoms: &[u32],
) -> (Vec<ContactSnap>, Vec<ContactSnap>) {
    let mut f1 = Vec::new();
    let mut f2 = Vec::new();
    for c in all {
        if f1_geoms.contains(&c.geom1_id) || f1_geoms.contains(&c.geom2_id) {
            f1.push(c.clone());
        }
        if f2_geoms.contains(&c.geom1_id) || f2_geoms.contains(&c.geom2_id) {
            f2.push(c.clone());
        }
    }
    (f1, f2)
}

async fn emit_left(
    runner: &Arc<NodeRunner>,
    step: u64,
    stamp: f64,
    snap: &crate::drivers::mjdata_bus::Snapshot,
    joint_names: Vec<String>,
    f1: &[ContactSnap],
    f2: &[ContactSnap],
) {
    if let Err(e) =
        ee_pose_left::emit(runner, ROBOT_NAME.into(), step, snap.xpos, snap.xquat, stamp).await
    {
        warn!("ee_pose_left: {e}");
    }
    let positions = snap.qpos.clone();
    let applied_forces = vec![0.0_f64; positions.len()];
    if let Err(e) = gripper_state_left::emit(
        runner, ROBOT_NAME.into(), step, joint_names,
        positions, applied_forces, stamp,
    ).await {
        warn!("gripper_state_left: {e}");
    }
    if let Err(e) = contact_forces_left_finger1::emit(
        runner, ROBOT_NAME.into(), step, to_left_f1(f1), stamp,
    ).await {
        warn!("contact_forces_left_finger1: {e}");
    }
    if let Err(e) = contact_forces_left_finger2::emit(
        runner, ROBOT_NAME.into(), step, to_left_f2(f2), stamp,
    ).await {
        warn!("contact_forces_left_finger2: {e}");
    }
}

async fn emit_right(
    runner: &Arc<NodeRunner>,
    step: u64,
    stamp: f64,
    snap: &crate::drivers::mjdata_bus::Snapshot,
    joint_names: Vec<String>,
    f1: &[ContactSnap],
    f2: &[ContactSnap],
) {
    if let Err(e) =
        ee_pose_right::emit(runner, ROBOT_NAME.into(), step, snap.xpos, snap.xquat, stamp).await
    {
        warn!("ee_pose_right: {e}");
    }
    let positions = snap.qpos.clone();
    let applied_forces = vec![0.0_f64; positions.len()];
    if let Err(e) = gripper_state_right::emit(
        runner, ROBOT_NAME.into(), step, joint_names,
        positions, applied_forces, stamp,
    ).await {
        warn!("gripper_state_right: {e}");
    }
    if let Err(e) = contact_forces_right_finger1::emit(
        runner, ROBOT_NAME.into(), step, to_right_f1(f1), stamp,
    ).await {
        warn!("contact_forces_right_finger1: {e}");
    }
    if let Err(e) = contact_forces_right_finger2::emit(
        runner, ROBOT_NAME.into(), step, to_right_f2(f2), stamp,
    ).await {
        warn!("contact_forces_right_finger2: {e}");
    }
}

// MessageContactsItem is a per-topic type; struct literal keeps us robust to
// peppygen field-order regeneration across versions.
fn to_left_f1(snaps: &[ContactSnap]) -> Vec<contact_forces_left_finger1::MessageContactsItem> {
    snaps.iter().map(|c| contact_forces_left_finger1::MessageContactsItem {
        body1: c.body1_id.to_string(),
        body2: c.body2_id.to_string(),
        position: c.pos,
        force: c.force,
    }).collect()
}

fn to_left_f2(snaps: &[ContactSnap]) -> Vec<contact_forces_left_finger2::MessageContactsItem> {
    snaps.iter().map(|c| contact_forces_left_finger2::MessageContactsItem {
        body1: c.body1_id.to_string(),
        body2: c.body2_id.to_string(),
        position: c.pos,
        force: c.force,
    }).collect()
}

fn to_right_f1(snaps: &[ContactSnap]) -> Vec<contact_forces_right_finger1::MessageContactsItem> {
    snaps.iter().map(|c| contact_forces_right_finger1::MessageContactsItem {
        body1: c.body1_id.to_string(),
        body2: c.body2_id.to_string(),
        position: c.pos,
        force: c.force,
    }).collect()
}

fn to_right_f2(snaps: &[ContactSnap]) -> Vec<contact_forces_right_finger2::MessageContactsItem> {
    snaps.iter().map(|c| contact_forces_right_finger2::MessageContactsItem {
        body1: c.body1_id.to_string(),
        body2: c.body2_id.to_string(),
        position: c.pos,
        force: c.force,
    }).collect()
}
