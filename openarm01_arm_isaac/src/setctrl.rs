// Shared set_ctrl publishing for the sim arm. Both the move_arm_joints action
// and the follow loop stream (q_des, dq_des) to the sim on the same topic; the
// sim-side actuator plugin owns the MIT gains and turns them into motor torque.

use std::collections::HashMap;

use peppylib::config::QoSProfile;
use peppylib::messaging::SenderTarget;
use peppylib::{MessengerHandle, Payload, TopicMessenger, TopicPublisher};
use serde::Serialize;
use sim_bridge_core::DaemonState;

use crate::trajectory::{ARM_DOF as DOF, JointVec};

const ARM_NODE_NAME: &str = "openarm01_arm";

#[derive(Serialize)]
struct SetCtrlPayload<'a> {
    actuator_values: HashMap<&'a str, f64>,
    velocity_values: HashMap<&'a str, f64>,
}

pub fn actuator_names(side: &str) -> [String; DOF] {
    std::array::from_fn(|i| format!("openarm_{side}_joint{}", i + 1))
}

// One publisher per side, shared by the move action and the follow loop. The
// instance_id keeps concurrent left+right arms from colliding on the registry.
pub async fn declare_publisher(
    handle: &MessengerHandle,
    daemon: &DaemonState,
    side: &str,
) -> Result<TopicPublisher, String> {
    let topic = format!("set_ctrl_arm_{side}");
    let instance_id = format!("openarm01_arm_{side}_setctrl_pub");
    let target = SenderTarget::node(ARM_NODE_NAME, "v1").map_err(|e| e.to_string())?;
    TopicMessenger::declare_publisher(
        handle,
        &daemon.core_node_name,
        &instance_id,
        target,
        None,
        &topic,
        QoSProfile::Standard,
    )
    .await
    .map_err(|e| e.to_string())
}

pub async fn publish(
    publisher: &TopicPublisher,
    actuator_names: &[String; DOF],
    q_des: &JointVec,
    dq_des: &JointVec,
) -> Result<(), String> {
    let mut actuator_values: HashMap<&str, f64> = HashMap::with_capacity(DOF);
    let mut velocity_values: HashMap<&str, f64> = HashMap::with_capacity(DOF);
    for i in 0..DOF {
        actuator_values.insert(actuator_names[i].as_str(), q_des[i]);
        velocity_values.insert(actuator_names[i].as_str(), dq_des[i]);
    }
    let payload = SetCtrlPayload {
        actuator_values,
        velocity_values,
    };
    let bytes = serde_json::to_vec(&payload).map_err(|e| e.to_string())?;
    publisher
        .publish(Payload::from(bytes))
        .await
        .map_err(|e| e.to_string())
}
