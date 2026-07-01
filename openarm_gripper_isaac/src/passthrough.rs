// Emit the resolved gripper aperture to the sim on the typed
// gripper_sim_passthrough topic. Shared by the follow loop and the move action
// so a single publisher drives the sim. The opening (m) is mapped to the
// per-version passthrough value ([`ApertureMap`]); the sim splits it across the
// two finger joints.

use peppygen::NodeRunner;
use peppygen::emitted_topics::openarm_gripper_sim_passthrough::v1::gripper_sim_passthrough;
use peppylib::TopicPublisher;

use crate::config::ApertureMap;

pub async fn declare_publisher(runner: &NodeRunner) -> Result<TopicPublisher, String> {
    gripper_sim_passthrough::declare_publisher(runner)
        .await
        .map_err(|e| e.to_string())
}

pub async fn publish(
    publisher: &TopicPublisher,
    gripper_id: u8,
    map: &ApertureMap,
    opening_m: f64,
) -> Result<(), String> {
    let value = map.to_wire(opening_m);
    let payload =
        gripper_sim_passthrough::build_message(gripper_id, value).map_err(|e| e.to_string())?;
    publisher.publish(payload).await.map_err(|e| e.to_string())
}
