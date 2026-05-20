use std::collections::HashMap;

use crate::drivers::mjdata_bus::MjDataBus;
use crate::error::{Error, Result};

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct GripperId(pub u8);

impl GripperId {
    pub fn side_word(self) -> &'static str {
        match self.0 {
            0 => "left",
            1 => "right",
            _ => "unknown",
        }
    }

    pub fn instance_id(self) -> &'static str {
        match self.0 {
            0 => "left_gripper",
            1 => "right_gripper",
            _ => "unknown",
        }
    }
}

// Resolved indices/IDs the gripper variant needs from MuJoCo, looked up in
// meta.json at startup. After this struct is built the gripper never deals in
// joint names again — it just reads addresses.
#[derive(Debug, Clone)]
pub struct ResolvedSide {
    pub finger_qpos_addrs: Vec<usize>,  // qpos addresses for our 2 fingers
    pub finger_ctrl_ids: Vec<usize>,    // ctrl ids for our 2 finger actuators
    pub ee_body_id: usize,              // body id of hand_tcp on this side
    pub finger1_geom_ids: Vec<u32>,     // collision geoms for "finger1"
    pub finger2_geom_ids: Vec<u32>,     // collision geoms for "finger2"
    pub joint_names: Vec<String>,       // for gripper_state.joint_names
    pub body_names: HashMap<usize, String>, // body_id → name, for contact telemetry
}

impl ResolvedSide {
    pub fn resolve(bus: &MjDataBus, gripper_id: GripperId) -> Result<Self> {
        let side = gripper_id.side_word();
        // Joints in the openarm MJCF are named openarm_<side>_finger_joint{1,2}.
        let joint_names: Vec<String> = vec![
            format!("openarm_{side}_finger_joint1"),
            format!("openarm_{side}_finger_joint2"),
        ];

        let mut finger_qpos_addrs = Vec::with_capacity(2);
        for jn in &joint_names {
            finger_qpos_addrs.push(bus.joint_qpos_addr(jn).map_err(Error::from)?);
        }

        // Actuators use a different scheme from joints: <side>_finger{1,2}_ctrl.
        let actuator_names = [
            format!("{side}_finger1_ctrl"),
            format!("{side}_finger2_ctrl"),
        ];
        let mut finger_ctrl_ids = Vec::with_capacity(2);
        for an in &actuator_names {
            finger_ctrl_ids.push(bus.actuator_ctrl_id(an).map_err(Error::from)?);
        }

        let ee_body_id = bus
            .body_id(&format!("openarm_{side}_hand_tcp"))
            .map_err(Error::from)?;

        // Finger geoms: scan meta for geom names beginning with the side prefix.
        // Topic convention (see root peppy.json5): finger1 = right-side finger of
        // the gripper, finger2 = left-side finger of the gripper. This mapping
        // holds for both left and right grippers — e.g. on the left gripper,
        // `openarm_left_right_finger*` geoms produce contact_forces_left_finger1.
        let f1_prefix = format!("openarm_{side}_right_finger");
        let f2_prefix = format!("openarm_{side}_left_finger");
        let mut finger1_geom_ids = Vec::new();
        let mut finger2_geom_ids = Vec::new();
        for (name, entry) in &bus.meta.geoms {
            if !name.contains("_finger_collision") {
                continue;
            }
            if name.starts_with(&f1_prefix) {
                finger1_geom_ids.push(entry.id as u32);
            } else if name.starts_with(&f2_prefix) {
                finger2_geom_ids.push(entry.id as u32);
            }
        }

        let body_names: HashMap<usize, String> =
            bus.meta.bodies.iter().map(|(name, b)| (b.id, name.clone())).collect();

        Ok(Self {
            finger_qpos_addrs,
            finger_ctrl_ids,
            ee_body_id,
            finger1_geom_ids,
            finger2_geom_ids,
            joint_names,
            body_names,
        })
    }

    pub fn all_finger_geom_ids(&self) -> Vec<u32> {
        let mut v = self.finger1_geom_ids.clone();
        v.extend_from_slice(&self.finger2_geom_ids);
        v
    }
}
