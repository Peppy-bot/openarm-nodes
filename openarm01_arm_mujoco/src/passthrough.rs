// Emit the resolved joint setpoint (q_des, dq_des) to the sim on the typed
// arm_sim_passthrough topic. Shared by the follow loop and the move action so a
// single publisher drives the sim. The arm sends ordered arrays (j1..j7); the
// sim maps array order onto its actuator names and applies the MIT gains.

use peppygen::NodeRunner;
use peppygen::emitted_topics::openarm01_arm_sim_passthrough::v1::arm_sim_passthrough;
use peppylib::TopicPublisher;

use crate::trajectory::JointVec;

pub async fn declare_publisher(runner: &NodeRunner) -> Result<TopicPublisher, String> {
    arm_sim_passthrough::declare_publisher(runner)
        .await
        .map_err(|e| e.to_string())
}

pub async fn publish(
    publisher: &TopicPublisher,
    arm_id: u8,
    q_des: &JointVec,
    dq_des: &JointVec,
) -> Result<(), String> {
    let payload =
        arm_sim_passthrough::build_message(arm_id, *q_des, *dq_des).map_err(|e| e.to_string())?;
    publisher.publish(payload).await.map_err(|e| e.to_string())
}
