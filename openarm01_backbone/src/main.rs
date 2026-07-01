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
mod planner;
mod startup;
mod streams;
mod trajectory;
mod types;

pub(crate) use arm_pair::ArmPair;
pub(crate) use types::{ARM_DOF, JointVec, Side};

use std::sync::Arc;
use std::sync::atomic::AtomicBool;
use std::time::Duration;

use peppygen::{NodeBuilder, Parameters, Result};
use tokio::sync::{mpsc, watch};
use tokio::task::JoinSet;
use tracing::{error, info};

use coordinator::ArmChannels;
use planner::{PlanConfig, Planner};

/// Spawn a never-returning inbound listener into the hub's supervised task set,
/// adapting its `()` output to the set's `Result` so its exit trips the
/// fatal-first-exit like any other hub task.
fn spawn_listener<F>(set: &mut JoinSet<Result<()>>, listener: F)
where
    F: std::future::Future<Output = ()> + Send + 'static,
{
    set.spawn(async move {
        listener.await;
        Ok(())
    });
}

fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::INFO)
        .init();

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
            max_joint_velocity_rad_s
                .iter()
                .all(|v| v.is_finite() && *v > 0.0),
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
            .unwrap_or_else(|e| {
                panic!("build left arm model from base '{}': {e}", params.left_base)
            });
        let right_model = srs_model::Arm::from_urdf_file(&params.urdf_path, &params.right_base)
            .unwrap_or_else(|e| {
                panic!(
                    "build right arm model from base '{}': {e}",
                    params.right_base
                )
            });
        info!(
            "arm models loaded (urdf '{}', left '{}', right '{}')",
            params.urdf_path, params.left_base, params.right_base
        );

        let governor = governor::Governor::build(
            &params.urdf_path,
            &params.meshes_dir,
            &params.left_base,
            &params.right_base,
            params.d_stop,
            params.d_safe,
            params.collision_governor_enabled,
        )
        .unwrap_or_else(|e| panic!("build self-collision governor: {e}"));
        info!(
            "self-collision governor ready (d_stop={} d_safe={} default {})",
            params.d_stop,
            params.d_safe,
            if params.collision_governor_enabled {
                "ENABLED"
            } else {
                "DISABLED"
            },
        );

        let left_limits = left_model.limits();
        let right_limits = right_model.limits();
        // The chase clamps every streamed/planned target into these limits with
        // `f64::clamp`, which is total only for finite, well-ordered bounds. Assert
        // it here so a malformed URDF aborts at bringup, not mid-tick.
        assert!(
            left_limits
                .iter()
                .chain(right_limits.iter())
                .all(|l| l.lo.is_finite() && l.hi.is_finite() && l.lo <= l.hi),
            "joint position limits must be finite and well-ordered (lo <= hi)"
        );
        let plan_cfg = |limits| PlanConfig {
            cycle_period,
            max_joint_velocity_rad_s,
            max_ee_velocity_m_s: params.max_ee_velocity_m_s,
            limits,
            stream_timeout,
        };
        let planners = ArmPair::new(
            Planner::new(Side::Left, left_model, plan_cfg(left_limits)),
            Planner::new(Side::Right, right_model, plan_cfg(right_limits)),
        );

        // Per-arm channels. Listeners fill the watch slots; action handlers send
        // accepted goals; the coordinator reads all of it.
        let (cmd_tx0, cmd_rx0) = watch::channel(None);
        let (cmd_tx1, cmd_rx1) = watch::channel(None);
        let (meas_tx0, meas_rx0) = watch::channel(None);
        let (meas_tx1, meas_rx1) = watch::channel(None);
        let (goal_tx0, goal_rx0) = mpsc::channel(1);
        let (goal_tx1, goal_rx1) = mpsc::channel(1);
        let busy = [
            Arc::new(AtomicBool::new(false)),
            Arc::new(AtomicBool::new(false)),
        ];
        let (config_tx, config_rx) = watch::channel(streams::GovernorConfig {
            enabled: params.collision_governor_enabled,
            d_stop: params.d_stop,
            d_safe: params.d_safe,
            max_ee_velocity_m_s: params.max_ee_velocity_m_s,
        });

        let channels = ArmPair::new(
            ArmChannels {
                command: cmd_rx0,
                measured: meas_rx0,
                goals: goal_rx0,
                busy: busy[0].clone(),
            },
            ArmChannels {
                command: cmd_rx1,
                measured: meas_rx1,
                goals: goal_rx1,
                busy: busy[1].clone(),
            },
        );

        // Gate exposing actions + streaming on the robot being ready, in a spawned
        // task so this setup closure returns promptly for the health probe.
        let runner = node_runner.clone();
        let token = node_runner.cancellation_token().clone();
        let goal_busy = [busy[0].clone(), busy[1].clone()];
        tokio::spawn(async move {
            startup::wait_until_ready(&runner, &token).await;

            // The coordination loop (owns the governor, both planners, the channels;
            // streams governed setpoints once both arms report) and the action
            // handlers are all meant to run for the life of the node.
            let mut set = JoinSet::new();
            set.spawn(coordinator::run(
                runner.clone(),
                governor,
                planners,
                channels,
                config_rx,
                cycle_period,
                token.clone(),
            ));
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

            // Inbound listeners buffer the latest message into the watch slots. They
            // run under the same fatal-first-exit supervision as the rest of the hub,
            // so a listener that dies takes the node down instead of leaving the
            // coordinator streaming on stale measured state or governor controls while
            // the node still reports healthy.
            spawn_listener(
                &mut set,
                streams::run_joint_command_listener(runner.clone(), [cmd_tx0, cmd_tx1]),
            );
            spawn_listener(
                &mut set,
                streams::run_joint_state_listener(runner.clone(), [meas_tx0, meas_tx1]),
            );
            spawn_listener(
                &mut set,
                streams::run_governor_config_listener(runner.clone(), config_tx),
            );

            // The first task to finish is fatal: cancel the node so the daemon
            // restarts a clean process rather than running on with a dead
            // coordination loop or a missing action handler while reporting healthy.
            if let Some(joined) = set.join_next().await {
                match joined {
                    Ok(Ok(())) => error!("backbone task exited; shutting node down"),
                    Ok(Err(e)) => error!(error = %e, "backbone task failed; shutting node down"),
                    Err(e) if e.is_panic() => {
                        error!(error = %e, "backbone task panicked; shutting node down")
                    }
                    Err(e) => error!(error = %e, "backbone task join failed; shutting node down"),
                }
            }
            token.cancel();
            set.shutdown().await;
        });

        Ok(())
    })
}
