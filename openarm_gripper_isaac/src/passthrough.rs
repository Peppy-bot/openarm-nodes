// Emit the resolved gripper aperture (m) to the sim on the typed
// gripper_sim_passthrough topic. The sim bridge maps the aperture onto its
// finger joints' own travel.

use peppygen::NodeRunner;
use peppygen::emitted_topics::openarm_gripper_sim_passthrough::v1::gripper_sim_passthrough;
use peppylib::TopicPublisher;

pub async fn declare_publisher(runner: &NodeRunner) -> Result<TopicPublisher, String> {
    gripper_sim_passthrough::declare_publisher(runner)
        .await
        .map_err(|e| e.to_string())
}

pub async fn publish(
    publisher: &TopicPublisher,
    gripper_id: u8,
    opening_m: f64,
) -> Result<(), String> {
    let payload =
        gripper_sim_passthrough::build_message(gripper_id, opening_m).map_err(|e| e.to_string())?;
    publisher.publish(payload).await.map_err(|e| e.to_string())
}
