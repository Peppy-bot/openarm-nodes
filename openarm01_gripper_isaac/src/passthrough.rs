// Emit the resolved gripper aperture to the sim on the typed
// gripper_sim_passthrough topic. Shared by the follow loop and the move action
// so a single publisher drives the sim. The sim splits the opening across the
// two finger joints.

use peppygen::NodeRunner;
use peppygen::emitted_topics::openarm01_gripper_sim_passthrough::v1::gripper_sim_passthrough;
use peppylib::TopicPublisher;

pub async fn declare_publisher(runner: &NodeRunner) -> Result<TopicPublisher, String> {
    gripper_sim_passthrough::declare_publisher(runner)
        .await
        .map_err(|e| e.to_string())
}

pub async fn publish(
    publisher: &TopicPublisher,
    gripper_id: u8,
    opening: f64,
) -> Result<(), String> {
    let payload = gripper_sim_passthrough::build_message(gripper_id, opening).map_err(|e| e.to_string())?;
    publisher.publish(payload).await.map_err(|e| e.to_string())
}
