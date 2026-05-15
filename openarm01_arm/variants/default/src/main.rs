mod control;
mod trajectory;

use openarm_can::{ArmCan, CallbackMode, v10};
use control::{ControlConfig, run_move_arm_joints};
use peppygen::exposed_actions::move_arm;
use peppygen::exposed_services::{get_arm_id, get_joint_positions};
use peppygen::{NodeBuilder, Parameters, Result};

use std::sync::{Arc, Mutex};
use std::time::Duration;
use tokio::signal::unix::{SignalKind, signal};
use tracing::{error, info};

// Sleep durations chosen to match ROS2 enactic/openarm_ros2 v10_simple_hardware behaviour.
const POST_ENABLE_SLEEP: Duration = Duration::from_millis(100);
const POST_DISABLE_SLEEP: Duration = Duration::from_millis(100);
const BRINGUP_RECV_US: i32 = 500;
const ENABLE_FD: bool = true;

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
            recv_timeout_us: i32::try_from(params.recv_timeout_us)
                .unwrap_or_else(|_| panic!("recv_timeout_us ({}) exceeds i32::MAX", params.recv_timeout_us)),
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

        // Hardware bringup — sequence mirrors ROS2 v10_simple_hardware on_init/on_activate.
        info!("opening CAN interface {can_interface} (FD={ENABLE_FD})");
        let mut arm = ArmCan::new(&can_interface, ENABLE_FD).expect("ArmCan::new");
        arm.init_motors(&v10::ARM_MOTOR_TYPES, &v10::ARM_SEND_IDS, &v10::ARM_RECV_IDS);
        arm.set_callback_mode(CallbackMode::Ignore);
        arm.enable_all();
        tokio::time::sleep(POST_ENABLE_SLEEP).await;
        arm.recv_all(BRINGUP_RECV_US);
        arm.set_callback_mode(CallbackMode::State);
        // recv_all in State mode populates initial joint state. ROS2 gets this implicitly
        // via the recv inside return_to_zero(); without it get_state() returns all zeros.
        arm.recv_all(BRINGUP_RECV_US);
        info!("arm ready");

        let arm = Arc::new(Mutex::new(arm));

        // Shutdown task: disables motors and releases lock on SIGINT/SIGTERM.
        {
            let arm = arm.clone();
            let lock_path = lock_path.clone();
            tokio::spawn(async move {
                let mut sigint = signal(SignalKind::interrupt()).expect("sigint");
                let mut sigterm = signal(SignalKind::terminate()).expect("sigterm");
                tokio::select! {
                    _ = sigint.recv() => {},
                    _ = sigterm.recv() => {},
                }
                info!("shutdown: disabling motors");
                // unwrap_or_else: recover even if the lock is poisoned (panic in control loop)
                // so disable_all() always runs and motors don't stay energised.
                arm.lock().unwrap_or_else(|e| e.into_inner()).disable_all();
                // ROS2 reference: sleep before recv to give motors time to acknowledge.
                // Drop the guard across the await so other tasks aren't blocked.
                tokio::time::sleep(POST_DISABLE_SLEEP).await;
                arm.lock().unwrap_or_else(|e| e.into_inner()).recv_all(BRINGUP_RECV_US);
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
            let arm = arm.clone();
            tokio::spawn(async move {
                loop {
                    if let Err(e) = get_joint_positions::handle_next_request(&runner, |_req| {
                        let a = arm.lock().unwrap_or_else(|e| e.into_inner());
                        Ok(get_joint_positions::Response::new(a.get_state().positions))
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

        // move_arm_joints: trajectory-tracking control loop.
        tokio::spawn(run_move_arm_joints(node_runner.clone(), arm.clone(), cfg.clone()));

        Ok(())
    })
}
