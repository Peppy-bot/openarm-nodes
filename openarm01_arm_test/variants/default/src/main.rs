use std::sync::Arc;
use std::time::Duration;

use peppygen::consumed_actions::arm_move_arm_joints::{ActionHandle, GoalRequest};
use peppygen::consumed_services::{arm_get_arm_id, arm_get_joint_positions};
use peppygen::{NodeBuilder, NodeRunner, Parameters, QoSProfile, Result};
use tracing::{error, info, warn};

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
    let id_response = arm_get_arm_id::poll(&node_runner, SERVICE_TIMEOUT, None, None).await?;
    info!(
        "get_arm_id -> id={} (instance={}, core_node={})",
        id_response.data.arm_id, id_response.instance_id, id_response.core_node
    );

    tokio::time::sleep(Duration::from_secs(3)).await;

    let joints = arm_get_joint_positions::poll(
        &node_runner,
        SERVICE_TIMEOUT,
        None,
        Some(&id_response.instance_id),
    )
    .await?;
    info!("joint_positions: {:.4?}", joints.data.joint_positions);

    let instance_id = id_response.instance_id.clone();
    info!("moving to zero");
    move_arm(&node_runner, &instance_id, &params, vec![0.0; 7]).await?;

    if !params.motion_enabled {
        info!("motion_enabled=false — connectivity confirmed; set motion_enabled=true to cycle joints");
        return Ok(());
    }

    let enabled = [
        params.joint_1_enabled,
        params.joint_2_enabled,
        params.joint_3_enabled,
        params.joint_4_enabled,
        params.joint_5_enabled,
        params.joint_6_enabled,
        params.joint_7_enabled,
    ];
    let limits_1 = [
        params.joint_1_limit_1,
        params.joint_2_limit_1,
        params.joint_3_limit_1,
        params.joint_4_limit_1,
        params.joint_5_limit_1,
        params.joint_6_limit_1,
        params.joint_7_limit_1,
    ];
    let limits_2 = [
        params.joint_1_limit_2,
        params.joint_2_limit_2,
        params.joint_3_limit_2,
        params.joint_4_limit_2,
        params.joint_5_limit_2,
        params.joint_6_limit_2,
        params.joint_7_limit_2,
    ];

    let n_enabled = enabled.iter().filter(|&&e| e).count();
    if n_enabled == 0 {
        warn!("motion_enabled=true but no joints enabled — set joint_N_enabled=true to cycle joints");
        return Ok(());
    }
    info!("cycling {n_enabled} joint(s): enabled={enabled:?}");

    let startup = joints.data.joint_positions;

    loop {
        for (label, limits) in [("limit_1", &limits_1), ("limit_2", &limits_2)] {
            let target: Vec<f64> = (0..7)
                .map(|i| if enabled[i] { limits[i] } else { startup.get(i).copied().unwrap_or(0.0) })
                .collect();
            info!("move to {label}: {:.4?}", target);
            move_arm(&node_runner, &instance_id, &params, target).await?;
            tokio::time::sleep(Duration::from_secs_f64(params.dwell_s)).await;
        }
    }
}

async fn move_arm(
    node_runner: &Arc<NodeRunner>,
    instance_id: &str,
    params: &Parameters,
    joint_positions: Vec<f64>,
) -> Result<()> {
    let mut handle = ActionHandle::fire_goal(
        node_runner,
        Duration::from_secs_f64(params.goal_timeout_s),
        None,
        Some(instance_id),
        GoalRequest {
            feedback_frequency: params.feedback_frequency,
            joint_positions,
        },
        QoSProfile::default(),
    )
    .await?;

    if !handle.data.accepted {
        error!("goal rejected by arm");
        std::process::exit(1);
    }
    info!("goal accepted, awaiting result");

    loop {
        match handle.on_next_feedback_message().await {
            Ok(fb) => info!("t={:.2}s joints={:.4?}", fb.action_time, fb.joint_positions),
            Err(_) => break,
        }
    }

    let result = handle.get_result(Duration::from_secs_f64(params.result_timeout_s)).await?;
    info!(
        "result: success={} message={:?} final_joints={:.4?} t={:.3}s",
        result.data.success,
        result.data.message,
        result.data.final_joint_positions,
        result.data.action_time,
    );
    Ok(())
}
