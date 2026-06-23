// Shared set_ctrl_gripper publishing for the sim gripper. Both the move_gripper
// action and the follow loop stream the per-finger target to the sim on the same
// topic; the sim-side actuator plugin drives the finger joints to it.

use std::collections::HashMap;

use peppylib::config::QoSProfile;
use peppylib::messaging::SenderTarget;
use peppylib::{MessengerHandle, Payload, TopicMessenger, TopicPublisher};
use serde::Serialize;
use sim_bridge_core::DaemonState;

const GRIPPER_NODE_NAME: &str = "openarm01_gripper";

#[derive(Serialize)]
struct SetCtrlPayload<'a> {
    actuator_values: HashMap<&'a str, f64>,
}

// The two finger joints driven for one gripper side.
pub fn actuator_names(side: &str) -> [String; 2] {
    [
        format!("openarm_{side}_finger_joint1"),
        format!("openarm_{side}_finger_joint2"),
    ]
}

// One publisher per side, shared by the move action and the follow loop. The
// instance_id keeps concurrent left+right grippers from colliding on the registry.
pub async fn declare_publisher(
    handle: &MessengerHandle,
    daemon: &DaemonState,
    side: &str,
) -> Result<TopicPublisher, String> {
    let topic = format!("set_ctrl_gripper_{side}");
    let instance_id = format!("openarm01_gripper_{side}_setctrl_pub");
    let target = SenderTarget::node(GRIPPER_NODE_NAME, "v1").map_err(|e| e.to_string())?;
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

// Publish the per-finger target (both fingers hold the same displacement, so
// the aperture is twice this value).
pub async fn publish(
    publisher: &TopicPublisher,
    actuator_names: &[String; 2],
    per_finger: f64,
) -> Result<(), String> {
    let mut values: HashMap<&str, f64> = HashMap::with_capacity(2);
    values.insert(actuator_names[0].as_str(), per_finger);
    values.insert(actuator_names[1].as_str(), per_finger);
    let payload = SetCtrlPayload {
        actuator_values: values,
    };
    let bytes = serde_json::to_vec(&payload).map_err(|e| e.to_string())?;
    publisher
        .publish(Payload::from(bytes))
        .await
        .map_err(|e| e.to_string())
}
