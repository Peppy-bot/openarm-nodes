mod control;
mod trajectory;

use openarm_can::v10;
use control::{ControlConfig, run_move_arm_joints};
use peppygen::exposed_actions::move_arm;
use peppygen::exposed_services::{get_arm_id, get_joint_positions};
use peppygen::{NodeBuilder, Parameters, Result};

use std::sync::{Arc, Mutex};
use std::time::Duration;
use tokio::signal::unix::{SignalKind, signal};
use tracing::{error, info};

fn main() -> Result<()> {
    tracing_subscriber::fmt().with_max_level(tracing::Level::INFO).init();

    NodeBuilder::new().run(|params: Parameters, node_runner| async move {
        let arm_id = params.arm_id;
        let can_interface = params.can_interface.clone();

        assert!(params.control_rate_hz > 0, "control_rate_hz must be > 0");
        let max_joint_velocity_rad_s = [
            params.max_joint_velocity_rad_s_1,
            params.max_joint_velocity_rad_s_2,
            params.max_joint_velocity_rad_s_3,
            params.max_joint_velocity_rad_s_4,
            params.max_joint_velocity_rad_s_5,
            params.max_joint_velocity_rad_s_6,
            params.max_joint_velocity_rad_s_7,
        ];
        assert!(
            max_joint_velocity_rad_s.iter().all(|v| *v > 0.0),
            "all max_joint_velocity_rad_s_N must be > 0"
        );
        assert!(params.min_motion_time_s >= 0.0, "min_motion_time_s must be >= 0");

        let cfg = ControlConfig {
            kp: [
                params.kp1, params.kp2, params.kp3, params.kp4,
                params.kp5, params.kp6, params.kp7,
            ],
            kd: [
                params.kd1, params.kd2, params.kd3, params.kd4,
                params.kd5, params.kd6, params.kd7,
            ],
            cycle_period: Duration::from_micros(1_000_000 / params.control_rate_hz as u64),
            recv_timeout_us: params.recv_timeout_us as i32,
            position_tolerance_rad: params.position_tolerance_rad,
            motion_timeout: Duration::from_secs_f64(params.motion_timeout_s),
            max_joint_velocity_rad_s,
            min_motion_time_s: params.min_motion_time_s,
        };

        // Instance lock — check if another instance with the same arm_id is running.
        let lock_path = format!("/tmp/openarm_arm_{arm_id}.lock");
        std::fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&lock_path)
            .unwrap_or_else(|e| panic!("instance lock {lock_path} held: {e}"));

        // CAN is stubbed out on this branch. `state` is a shared sim of joint positions:
        // get_joint_positions reads it; the move_arm_joints control loop writes the
        // trajectory's commanded position into it each cycle, so readbacks track q_des.
        info!("CAN stubbed (can_interface={can_interface} ignored) — running without hardware");
        let state: Arc<Mutex<v10::JointVec>> = Arc::new(Mutex::new([0.0f64; v10::ARM_DOF]));

        // Shutdown task: releases the instance lock on SIGINT/SIGTERM.
        {
            let lock_path = lock_path.clone();
            tokio::spawn(async move {
                let mut sigint = signal(SignalKind::interrupt()).expect("sigint");
                let mut sigterm = signal(SignalKind::terminate()).expect("sigterm");
                tokio::select! {
                    _ = sigint.recv() => {},
                    _ = sigterm.recv() => {},
                }
                info!("shutdown");
                let _ = std::fs::remove_file(&lock_path);
                std::process::exit(0);
            });
        }

        // get_arm_id service.
        {
            let runner = node_runner.clone();
            tokio::spawn(async move {
                loop {
                    if let Err(e) = get_arm_id::handle_next_request(&runner, |_req| {
                        Ok(get_arm_id::Response::new(arm_id))
                    })
                    .await
                    {
                        error!("get_arm_id: {e}");
                    }
                }
            });
        }

        // get_joint_positions service.
        {
            let runner = node_runner.clone();
            let state = state.clone();
            tokio::spawn(async move {
                loop {
                    if let Err(e) = get_joint_positions::handle_next_request(&runner, |_req| {
                        let s = state.lock().unwrap_or_else(|e| e.into_inner());
                        Ok(get_joint_positions::Response::new(s.to_vec()))
                    })
                    .await
                    {
                        error!("get_joint_positions: {e}");
                    }
                }
            });
        }

        // TODO: move_arm (Cartesian) always rejects. To implement: generate a minimum-jerk
        // Cartesian trajectory, run IK at each control cycle to get joint targets, then MIT
        // control — requires an embedded IK solver running at control rate.
        {
            let runner = node_runner.clone();
            tokio::spawn(async move {
                let mut handle = move_arm::ActionHandle::expose(&runner)
                    .await
                    .expect("expose move_arm");
                loop {
                    if let Err(e) = handle
                        .handle_goal_next_request(|_req| Ok(move_arm::GoalResponse::new(false)))
                        .await
                    {
                        error!("move_arm goal: {e}");
                    }
                }
            });
        }

        // move_arm_joints: trajectory-tracking control loop (CAN stubbed — writes q_des into shared state).
        tokio::spawn(run_move_arm_joints(node_runner.clone(), state.clone(), cfg.clone()));

        Ok(())
    })
}
