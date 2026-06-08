mod control;
mod friction;
mod trajectory;

use openarm_can::{ArmCan, CallbackMode, v10};
use control::ControlConfig;
use peppygen::exposed_services::openarm01_arm::v1::{get_arm_id, get_joint_positions};
use peppygen::{NodeBuilder, Parameters, Result};
use srs_model::nalgebra::Isometry3;

use std::sync::{Arc, Mutex};
use std::time::Duration;
use tokio::signal::unix::{SignalKind, signal};
use tracing::{error, info, warn};

/// Degrees of freedom of the arm.
pub const ARM_DOF: usize = 7;
/// One joint-space vector (positions, velocities, or torques), j1..j7.
pub type JointVec = [f64; ARM_DOF];

/// `arm_id` values (0 = left, 1 = right). The geometry and joint limits come from
/// the URDF via `base_link`; `arm_id` is the robot-side identity for the service
/// contract and the log label.
const ARM_ID_LEFT: u8 = 0;
const ARM_ID_RIGHT: u8 = 1;

/// Human-readable side for the given `arm_id`, panicking on an unknown value so a
/// misconfigured run fails loudly at startup.
fn side_label(arm_id: u8) -> &'static str {
    match arm_id {
        ARM_ID_LEFT => "left",
        ARM_ID_RIGHT => "right",
        other => panic!("arm_id must be {ARM_ID_LEFT} (left) or {ARM_ID_RIGHT} (right), got {other}"),
    }
}

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
        let side = side_label(arm_id);

        // Build the srs_model arm from the URDF: forward kinematics for the
        // in-process gravity and Coriolis feedforward, plus joint limits off the
        // same parsed chain (one source of truth, the URDF). A non-SRS or short
        // chain from base_link errors here.
        let model = srs_model::Arm::from_urdf_file(&params.urdf_path, &params.base_link)
            .unwrap_or_else(|e| panic!("build arm model from base '{}': {e}", params.base_link));
        info!("model loaded (urdf '{}', base '{}')", params.urdf_path, params.base_link);

        // Gravity acts along world -Z, so it is only correct if the URDF carries the
        // mount tree above base_link to orient that frame. We do not force one (a
        // base-rooted URDF legitimately evaluates gravity in the base frame), so log
        // which frame is in play: identity mount means base_link is the URDF root.
        if model.base_from_world() == Isometry3::identity() {
            warn!(
                "no world->base mount tree above '{}': gravity/Coriolis evaluated in the \
                 base frame (correct only if base_link's frame is gravity-aligned)",
                params.base_link
            );
        } else {
            info!("mount tree resolved: gravity/Coriolis evaluated in the world frame");
        }

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
            limits: model.limits(),
        };

        // Echo the resolved config so every run records exactly what it ran with.
        info!(
            "config: arm_id={arm_id} ({side}) rate={}Hz recv_timeout={}us",
            params.control_rate_hz,
            cfg.recv_timeout_us,
        );
        info!("config: kp={:?} kd={:?}", cfg.kp, cfg.kd);

        // Instance lock: check if another instance with the same arm_id is running.
        let lock_path = format!("/tmp/openarm_arm_{arm_id}.lock");
        std::fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&lock_path)
            .unwrap_or_else(|e| panic!("instance lock {lock_path} held: {e}"));

        // Hardware bringup: sequence mirrors ROS2 v10_simple_hardware on_init/on_activate.
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

        // Shutdown task: disables motors and releases the lock on shutdown.
        // `peppy node stop` shuts a daemon node down in-band over messaging by
        // cancelling the runtime's cancellation token, not by sending a unix
        // signal, so the SIGINT/SIGTERM arms alone never fire on a stop and the
        // motors would stay energised. Observing the token closes that gap and
        // also lets the process exit promptly instead of being force-killed.
        {
            let arm = arm.clone();
            let node_runner = node_runner.clone();
            let cancel = node_runner.cancellation_token().clone();
            let lock_path = lock_path.clone();
            tokio::spawn(async move {
                let mut sigint = signal(SignalKind::interrupt()).expect("sigint");
                let mut sigterm = signal(SignalKind::terminate()).expect("sigterm");
                tokio::select! {
                    _ = sigint.recv() => {},
                    _ = sigterm.recv() => {},
                    _ = cancel.cancelled() => {},
                }
                info!("shutdown: disabling motors, releasing lock");
                // unwrap_or_else: recover even if poisoned (panic in control loop)
                // so disable_all() always runs and motors don't stay energised.
                // Hold the lock from here through process exit so an in-flight
                // control loop can neither command mid-shutdown nor re-command the
                // motors after they have been disabled.
                let mut a = arm.lock().unwrap_or_else(|e| e.into_inner());
                a.disable_all();
                // ROS2 reference: sleep before recv to let motors acknowledge.
                std::thread::sleep(POST_DISABLE_SLEEP);
                a.recv_all(BRINGUP_RECV_US);
                if let Err(e) = std::fs::remove_file(&lock_path) {
                    warn!("failed to remove lock {lock_path}: {e}");
                }
                // exit() does not run destructors, so the guard is never released;
                // the motors stay disabled as the process dies.
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

        // Single control task (the only motor writer): holds its setpoint with
        // gravity-comp + PD at startup and between moves, trajectory tracking while a
        // move_arm_joints goal runs, back to holding the goal after. It spawns its
        // own action handler, which only admits goals and hands them over.
        control::spawn(node_runner.clone(), arm.clone(), cfg, model);

        Ok(())
    })
}
