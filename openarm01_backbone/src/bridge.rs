use std::sync::Arc;

use peppygen::NodeRunner;
use peppylib::runtime::CancellationToken;
use sim_bridge_core::{
    BridgeConfig, DaemonState, SimBridge,
    types::messages::{
        ClockMsg, ContactForcesMsg, EePoseMsg, GripperStateMsg, ImuMsg, JointStatesMsg,
        OdometryMsg, TfTreeMsg, WrenchMsg,
    },
};

pub fn build(
    runner: Arc<NodeRunner>,
    daemon: DaemonState,
    token: CancellationToken,
    sim_node: Arc<str>,
    config: &BridgeConfig,
) -> SimBridge<NodeRunner> {
    let mut bridge = SimBridge::new(runner, daemon, token, sim_node);

    for pub_cfg in &config.publishers {
        let topic: Arc<str> = Arc::from(pub_cfg.topic.as_str());
        bridge = match (pub_cfg.type_name.as_str(), pub_cfg.topic.as_str()) {
            ("clock", _) => bridge.sim_to_os::<ClockMsg, _>(topic, |runner, msg| {
                Box::pin(async move {
                    use peppygen::emitted_topics::clock;
                    clock::emit(&runner, msg.step, msg.sim_time, msg.stamp)
                        .await
                        .map_err(|e| e.to_string())
                })
            }),
            ("joint_states", _) => bridge.sim_to_os::<JointStatesMsg, _>(topic, |runner, msg| {
                Box::pin(async move {
                    use peppygen::emitted_topics::joint_states;
                    joint_states::emit(
                        &runner,
                        msg.robot,
                        msg.step,
                        msg.positions,
                        msg.velocities,
                        msg.stamp,
                    )
                    .await
                    .map_err(|e| e.to_string())
                })
            }),
            ("imu", "imu_left") => bridge.sim_to_os::<ImuMsg, _>(topic, |runner, msg| {
                Box::pin(async move {
                    use peppygen::emitted_topics::imu_left;
                    imu_left::emit(
                        &runner,
                        msg.robot,
                        msg.step,
                        msg.orientation,
                        msg.angular_velocity,
                        msg.linear_acceleration,
                        msg.stamp,
                    )
                    .await
                    .map_err(|e| e.to_string())
                })
            }),
            ("imu", "imu_right") => bridge.sim_to_os::<ImuMsg, _>(topic, |runner, msg| {
                Box::pin(async move {
                    use peppygen::emitted_topics::imu_right;
                    imu_right::emit(
                        &runner,
                        msg.robot,
                        msg.step,
                        msg.orientation,
                        msg.angular_velocity,
                        msg.linear_acceleration,
                        msg.stamp,
                    )
                    .await
                    .map_err(|e| e.to_string())
                })
            }),
            ("ee_pose", "ee_pose_left") => bridge.sim_to_os::<EePoseMsg, _>(topic, |runner, msg| {
                Box::pin(async move {
                    use peppygen::emitted_topics::ee_pose_left;
                    ee_pose_left::emit(
                        &runner,
                        msg.robot,
                        msg.step,
                        msg.position,
                        msg.orientation,
                        msg.stamp,
                    )
                    .await
                    .map_err(|e| e.to_string())
                })
            }),
            ("ee_pose", "ee_pose_right") => {
                bridge.sim_to_os::<EePoseMsg, _>(topic, |runner, msg| {
                    Box::pin(async move {
                        use peppygen::emitted_topics::ee_pose_right;
                        ee_pose_right::emit(
                            &runner,
                            msg.robot,
                            msg.step,
                            msg.position,
                            msg.orientation,
                            msg.stamp,
                        )
                        .await
                        .map_err(|e| e.to_string())
                    })
                })
            }
            ("tf_tree", _) => bridge.sim_to_os::<TfTreeMsg, _>(topic, |runner, msg| {
                Box::pin(async move {
                    use peppygen::emitted_topics::tf_tree;
                    tf_tree::emit(&runner, msg.robot, msg.step, msg.frames, msg.stamp)
                        .await
                        .map_err(|e| e.to_string())
                })
            }),
            ("odometry", _) => bridge.sim_to_os::<OdometryMsg, _>(topic, |runner, msg| {
                Box::pin(async move {
                    use peppygen::emitted_topics::odometry;
                    odometry::emit(
                        &runner,
                        msg.robot,
                        msg.step,
                        msg.position,
                        msg.orientation,
                        msg.linear_velocity,
                        msg.angular_velocity,
                        msg.stamp,
                    )
                    .await
                    .map_err(|e| e.to_string())
                })
            }),
            ("wrench", _) => bridge.sim_to_os::<WrenchMsg, _>(topic, |runner, msg| {
                Box::pin(async move {
                    use peppygen::emitted_topics::wrench;
                    wrench::emit(&runner, msg.robot, msg.step, msg.force, msg.torque, msg.stamp)
                        .await
                        .map_err(|e| e.to_string())
                })
            }),
            ("contact_forces", "contact_forces_left_finger1") => {
                bridge.sim_to_os::<ContactForcesMsg, _>(topic, |runner, msg| {
                    Box::pin(async move {
                        use peppygen::emitted_topics::contact_forces_left_finger1;
                        contact_forces_left_finger1::emit(
                            &runner,
                            msg.robot,
                            msg.step,
                            msg.contacts,
                            msg.stamp,
                        )
                        .await
                        .map_err(|e| e.to_string())
                    })
                })
            }
            ("contact_forces", "contact_forces_left_finger2") => {
                bridge.sim_to_os::<ContactForcesMsg, _>(topic, |runner, msg| {
                    Box::pin(async move {
                        use peppygen::emitted_topics::contact_forces_left_finger2;
                        contact_forces_left_finger2::emit(
                            &runner,
                            msg.robot,
                            msg.step,
                            msg.contacts,
                            msg.stamp,
                        )
                        .await
                        .map_err(|e| e.to_string())
                    })
                })
            }
            ("contact_forces", "contact_forces_right_finger1") => {
                bridge.sim_to_os::<ContactForcesMsg, _>(topic, |runner, msg| {
                    Box::pin(async move {
                        use peppygen::emitted_topics::contact_forces_right_finger1;
                        contact_forces_right_finger1::emit(
                            &runner,
                            msg.robot,
                            msg.step,
                            msg.contacts,
                            msg.stamp,
                        )
                        .await
                        .map_err(|e| e.to_string())
                    })
                })
            }
            ("contact_forces", "contact_forces_right_finger2") => {
                bridge.sim_to_os::<ContactForcesMsg, _>(topic, |runner, msg| {
                    Box::pin(async move {
                        use peppygen::emitted_topics::contact_forces_right_finger2;
                        contact_forces_right_finger2::emit(
                            &runner,
                            msg.robot,
                            msg.step,
                            msg.contacts,
                            msg.stamp,
                        )
                        .await
                        .map_err(|e| e.to_string())
                    })
                })
            }
            ("gripper_state", "gripper_state_left") => {
                bridge.sim_to_os::<GripperStateMsg, _>(topic, |runner, msg| {
                    Box::pin(async move {
                        use peppygen::emitted_topics::gripper_state_left;
                        gripper_state_left::emit(
                            &runner,
                            msg.robot,
                            msg.step,
                            msg.joint_names,
                            msg.positions,
                            msg.applied_forces,
                            msg.stamp,
                        )
                        .await
                        .map_err(|e| e.to_string())
                    })
                })
            }
            ("gripper_state", "gripper_state_right") => {
                bridge.sim_to_os::<GripperStateMsg, _>(topic, |runner, msg| {
                    Box::pin(async move {
                        use peppygen::emitted_topics::gripper_state_right;
                        gripper_state_right::emit(
                            &runner,
                            msg.robot,
                            msg.step,
                            msg.joint_names,
                            msg.positions,
                            msg.applied_forces,
                            msg.stamp,
                        )
                        .await
                        .map_err(|e| e.to_string())
                    })
                })
            }
            (t, n) => {
                tracing::warn!(type_name = t, topic = n, "unknown publisher in config — skipped");
                bridge
            }
        };
    }

    bridge
}
