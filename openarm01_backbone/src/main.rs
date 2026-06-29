//! openarm01_backbone - bimanual coordination hub. It owns all arm motion: it
//! consumes the operator joint stream and exposes the joint / Cartesian move
//! actions, generates the trajectories, runs the self-collision governor over
//! both arms together, and streams the governed per-arm setpoints the arms
//! follow. Grippers are unchanged: it still relays move_gripper to the per-side
//! gripper instances. The governor is URDF-based, so it runs identically for the
//! sim and the real arms.

mod actions;
mod arm_pair;
mod chase;
mod coordinator;
mod governor;
mod openarm_v10;
mod pacer;
mod planner;
mod startup;
mod streams;
mod trajectory;
mod types;

pub(crate) use arm_pair::ArmPair;
pub(crate) use types::{ARM_DOF, ARM_ID_LEFT, ARM_ID_RIGHT, JointVec, Setpoint, side_index};

use std::sync::Arc;
use std::sync::atomic::AtomicBool;
use std::time::Duration;

use peppygen::{NodeBuilder, Parameters, Result};
use tokio::sync::{mpsc, watch};
use tokio::task::JoinSet;
use tracing::{error, info};

use coordinator::ArmChannels;
use planner::{PlanConfig, Planner};

fn side_label(idx: usize) -> &'static str {
    if idx == 0 { "left" } else { "right" }
}

fn main() -> Result<()> {
    tracing_subscriber::fmt().with_max_level(tracing::Level::INFO).init();

    NodeBuilder::new().run(|params: Parameters, node_runner| async move {
        assert!(params.control_rate_hz > 0, "control_rate_hz must be > 0");
        assert!(
            params.stream_timeout_s.is_finite() && params.stream_timeout_s > 0.0,
            "stream_timeout_s must be a positive finite number"
        );
        let max_joint_velocity_rad_s: JointVec = [
            params.max_joint_velocity_rad_s_1,
            params.max_joint_velocity_rad_s_2,
            params.max_joint_velocity_rad_s_3,
            params.max_joint_velocity_rad_s_4,
            params.max_joint_velocity_rad_s_5,
            params.max_joint_velocity_rad_s_6,
            params.max_joint_velocity_rad_s_7,
        ];
        assert!(
            max_joint_velocity_rad_s.iter().all(|v| v.is_finite() && *v > 0.0),
            "all max_joint_velocity_rad_s_N must be finite and > 0"
        );
        assert!(
            params.max_ee_velocity_m_s.is_finite() && params.max_ee_velocity_m_s > 0.0,
            "max_ee_velocity_m_s must be a positive finite number"
        );

        let cycle_period = Duration::from_micros(1_000_000 / params.control_rate_hz as u64);
        let stream_timeout = Duration::from_secs_f64(params.stream_timeout_s);

        // Two arm models (FK/IK/Jacobian/limits) and the bimanual collision model,
        // all from the one URDF. A bad URDF / base link / mesh dir aborts bringup.
        let left_model = srs_model::Arm::from_urdf_file(&params.urdf_path, &params.left_base)
            .unwrap_or_else(|e| panic!("build left arm model from base '{}': {e}", params.left_base));
        let right_model = srs_model::Arm::from_urdf_file(&params.urdf_path, &params.right_base)
            .unwrap_or_else(|e| panic!("build right arm model from base '{}': {e}", params.right_base));
        info!("arm models loaded (urdf '{}', left '{}', right '{}')", params.urdf_path, params.left_base, params.right_base);

        let governor = governor::Governor::build(
            &params.urdf_path,
            &params.meshes_dir,
            &params.left_base,
            &params.right_base,
            params.d_stop,
            params.d_safe,
            params.collision_enabled_default,
        )
        .unwrap_or_else(|e| panic!("build self-collision governor: {e}"));
        info!(
            "self-collision governor ready (d_stop={} d_safe={} default {})",
            params.d_stop,
            params.d_safe,
            if params.collision_enabled_default { "ENABLED" } else { "DISABLED" },
        );

        let left_limits = left_model.limits();
        let right_limits = right_model.limits();
        let plan_cfg = |limits| PlanConfig {
            cycle_period,
            max_joint_velocity_rad_s,
            max_ee_velocity_m_s: params.max_ee_velocity_m_s,
            limits,
            stream_timeout,
        };
        let planners = ArmPair::new(
            Planner::new(side_label(0), left_model, plan_cfg(left_limits), [0.0; ARM_DOF]),
            Planner::new(side_label(1), right_model, plan_cfg(right_limits), [0.0; ARM_DOF]),
        );

        // Per-arm channels. Listeners fill the watch slots; action handlers send
        // accepted goals; the coordinator reads all of it.
        let (cmd_tx0, cmd_rx0) = watch::channel(None);
        let (cmd_tx1, cmd_rx1) = watch::channel(None);
        let (meas_tx0, meas_rx0) = watch::channel(None);
        let (meas_tx1, meas_rx1) = watch::channel(None);
        let (goal_tx0, goal_rx0) = mpsc::channel(1);
        let (goal_tx1, goal_rx1) = mpsc::channel(1);
        let busy = [Arc::new(AtomicBool::new(false)), Arc::new(AtomicBool::new(false))];
        let (config_tx, config_rx) = watch::channel(streams::GovernorConfig {
            enabled: params.collision_enabled_default,
            d_stop: params.d_stop,
            d_safe: params.d_safe,
            max_ee_velocity_m_s: params.max_ee_velocity_m_s,
        });

        let channels = ArmPair::new(
            ArmChannels { command: cmd_rx0, measured: meas_rx0, goals: goal_rx0, busy: busy[0].clone() },
            ArmChannels { command: cmd_rx1, measured: meas_rx1, goals: goal_rx1, busy: busy[1].clone() },
        );

        // Always-on inbound listeners (they only buffer the latest message).
        tokio::spawn(streams::run_joint_command_listener(node_runner.clone(), [cmd_tx0, cmd_tx1]));
        tokio::spawn(streams::run_joint_state_listener(node_runner.clone(), [meas_tx0, meas_tx1]));
        tokio::spawn(streams::run_governor_config_listener(node_runner.clone(), config_tx));

        // Gate exposing actions + streaming on the robot being ready, in a spawned
        // task so this setup closure returns promptly for the health probe.
        let runner = node_runner.clone();
        let token = node_runner.cancellation_token().clone();
        let goal_busy = [busy[0].clone(), busy[1].clone()];
        tokio::spawn(async move {
            startup::wait_until_ready(&runner, &token).await;

            // The coordination loop owns the governor, both planners, and the
            // channels; it streams governed setpoints once both arms report.
            tokio::spawn(coordinator::run(runner.clone(), governor, planners, channels, config_rx, cycle_period));

            let mut set = JoinSet::new();
            set.spawn(actions::arm::run_move_arm_joints(
                runner.clone(),
                [goal_tx0.clone(), goal_tx1.clone()],
                [goal_busy[0].clone(), goal_busy[1].clone()],
                [left_limits, right_limits],
            ));
            set.spawn(actions::arm::run_move_arm(
                runner.clone(),
                [goal_tx0, goal_tx1],
                [goal_busy[0].clone(), goal_busy[1].clone()],
            ));
            set.spawn(actions::gripper::run(runner.clone(), token.clone()));
            while let Some(joined) = set.join_next().await {
                match joined {
                    Ok(Ok(())) => info!("backbone handler exited cleanly"),
                    Ok(Err(e)) => error!(error = %e, "backbone handler returned Err"),
                    Err(e) if e.is_panic() => error!(error = %e, "backbone handler panicked"),
                    Err(e) => error!(error = %e, "backbone handler join failed"),
                }
            }
        });

        Ok(())
    })
}
