use std::sync::Arc;
use std::time::Duration;

use peppygen::consumed_actions::arm_move_arm_joints::{ActionHandle, GoalRequest, ResultOutcome};
use peppygen::consumed_services::{arm_get_arm_id, arm_get_joint_positions, ik_get_ik};
use peppygen::{NodeBuilder, NodeRunner, Parameters, QoSProfile, Result};
use tracing::{error, info};

const SERVICE_TIMEOUT: Duration = Duration::from_secs(5);

fn main() -> Result<()> {
    tracing_subscriber::fmt().with_max_level(tracing::Level::INFO).init();

    NodeBuilder::new().run(|params: Parameters, node_runner| async move {
        tokio::spawn(async move {
            if let Err(e) = run(params, node_runner).await {
                error!("{e}");
                std::process::exit(1);
            }
        });
        Ok(())
    })
}

async fn run(params: Parameters, node_runner: Arc<NodeRunner>) -> Result<()> {
    let id = arm_get_arm_id::poll(&node_runner, SERVICE_TIMEOUT).await?;
    info!(
        "get_arm_id -> id={} (instance={}, core_node={})",
        id.data.arm_id, id.instance_id, id.core_node
    );

    // Give the arm a moment to publish a fresh state before reading it.
    tokio::time::sleep(Duration::from_secs(3)).await;

    let state = arm_get_joint_positions::poll(&node_runner, SERVICE_TIMEOUT).await?;
    let start = state.data.joint_positions;
    info!("joint_positions: {:.4?}", start);

    if !params.motion_enabled {
        info!("motion_enabled=false: connectivity confirmed; set motion_enabled=true to move");
        return Ok(());
    }

    // Resolve the workspace (Cartesian) target to joint angles via IK, seeded with
    // the current pose so the solver picks the branch nearest where we are.
    let target_position = [params.target_x, params.target_y, params.target_z];
    let target_orientation = [
        params.target_qx,
        params.target_qy,
        params.target_qz,
        params.target_qw,
    ];
    info!(
        "get_ik: pos={:.4?} quat_xyzw={:.4?} policy={} arm_angle={:.4}",
        target_position, target_orientation, params.arm_angle_policy, params.arm_angle
    );
    let ik = ik_get_ik::poll(
        &node_runner,
        Duration::from_secs_f64(params.ik_timeout_s),
        ik_get_ik::Request {
            target_position,
            target_orientation,
            seed: start.to_vec(),
            arm_angle_policy: params.arm_angle_policy.clone(),
            arm_angle: params.arm_angle,
        },
    )
    .await?;

    if !ik.data.success {
        error!("IK failed: {}", ik.data.message);
        std::process::exit(1);
    }
    if ik.data.joint_positions.len() != 7 {
        error!("IK returned {} joints, expected 7", ik.data.joint_positions.len());
        std::process::exit(1);
    }
    let target: [f64; 7] = std::array::from_fn(|i| ik.data.joint_positions[i]);
    info!("IK solution: {:.4?} (arm_angle={:.4})", target, ik.data.arm_angle);

    move_arm(&node_runner, &params, target).await
}

/// Fire one `move_arm_joints` goal to `joint_positions`, stream feedback, and log
/// the outcome.
async fn move_arm(
    node_runner: &Arc<NodeRunner>,
    params: &Parameters,
    joint_positions: [f64; 7],
) -> Result<()> {
    info!("move to {:.4?}", joint_positions);
    let mut handle = ActionHandle::fire_goal(
        node_runner,
        Duration::from_secs_f64(params.goal_timeout_s),
        GoalRequest {
            feedback_frequency: params.feedback_frequency,
            joint_positions,
        },
        QoSProfile::default(),
    )
    .await?;

    if !handle.data.accepted {
        error!("goal rejected: {:?}", handle.data.error_message);
        std::process::exit(1);
    }
    info!("goal accepted, awaiting result");

    while let Ok(fb) = handle.on_next_feedback_message().await {
        info!("t={:.2}s joints={:.4?}", fb.action_time, fb.joint_positions);
    }

    let result = handle
        .get_result(Duration::from_secs_f64(params.result_timeout_s))
        .await?;
    match result.outcome {
        ResultOutcome::Completed(d) => info!(
            "completed: success={} message={:?} final={:.4?} t={:.3}s",
            d.success, d.message, d.final_joint_positions, d.action_time,
        ),
        ResultOutcome::Cancelled(d) => info!("cancelled: {:?}", d.message),
        ResultOutcome::Abandoned => error!("goal abandoned"),
        ResultOutcome::Expired => error!("result wait expired"),
    }
    Ok(())
}
