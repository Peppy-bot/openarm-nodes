mod actions;
mod control;
mod friction;
mod pacer;
mod stream;
mod trajectory;

use openarm_can::{ArmCan, CallbackMode, v10};
use control::ControlConfig;
use peppygen::exposed_services::openarm01_arm::v1::get_arm_id;
use peppygen::{NodeBuilder, Parameters, Result};
use peppylib::datastore::{self, Encoding};
use srs_model::nalgebra::Isometry3;

use std::sync::{Arc, Mutex};
use std::time::Duration;
use tokio::sync::{oneshot, watch};
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
const BRINGUP_RECV_US: i32 = 500;
const ENABLE_FD: bool = true;
const DATASTORE_TIMEOUT: Duration = Duration::from_secs(3);
/// Tighter bound for shutdown lock removal so the return-to-ready park
/// (`control::PARK_DURATION_S`) + motor disable + removal stays inside the
/// default 5 s shutdown grace window.
const LOCK_REMOVE_TIMEOUT: Duration = Duration::from_secs(1);

fn main() -> Result<()> {
    tracing_subscriber::fmt().with_max_level(tracing::Level::INFO).init();

    NodeBuilder::new().run(|params: Parameters, node_runner| async move {
        let arm_id = params.arm_id;
        let can_interface = params.can_interface.clone();

        // Rates feed `Duration::from_micros(1_000_000 / rate)`, so a rate above
        // 1 MHz would round to a 0 µs period; no real deployment approaches that,
        // so just guard against zero.
        assert!(params.control_rate_hz > 0, "control_rate_hz must be > 0");
        assert!(params.state_rate_hz > 0, "state_rate_hz must be > 0");
        assert!(
            params.stream_timeout_s.is_finite() && params.stream_timeout_s > 0.0,
            "stream_timeout_s must be a positive finite number"
        );
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
            max_joint_velocity_rad_s.iter().all(|v| v.is_finite() && *v > 0.0),
            "all max_joint_velocity_rad_s_N must be finite and > 0"
        );
        assert!(
            params.max_ee_velocity_m_s.is_finite() && params.max_ee_velocity_m_s > 0.0,
            "max_ee_velocity_m_s must be a positive finite number"
        );
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
            max_joint_velocity_rad_s,
            max_ee_velocity_m_s: params.max_ee_velocity_m_s,
            limits: model.limits(),
            stream_timeout: Duration::from_secs_f64(params.stream_timeout_s),
        };

        // Echo the resolved config so every run records exactly what it ran with.
        info!(
            "config: arm_id={arm_id} ({side}) rate={}Hz recv_timeout={}us",
            params.control_rate_hz,
            cfg.recv_timeout_us,
        );
        info!("config: kp={:?} kd={:?}", cfg.kp, cfg.kd);
        info!(
            "config: max_ee_velocity={} m/s, stream_timeout={}s",
            params.max_ee_velocity_m_s, params.stream_timeout_s,
        );

        // Validate the static ready pose against the parsed joint limits here,
        // during bringup, so a misconfigured pose aborts the whole process before
        // any hardware is touched instead of panicking inside the spawned control
        // task (which would leave a zombie node with motors enabled but no loop).
        control::assert_ready_pose_in_limits(&cfg.limits);

        // Instance lock: crash if another instance with the same arm_id is
        // running. Held in the core-node datastore (released from the on_shutdown
        // hook below), so a lock leaked by a hard crash clears with the stack
        // instead of lingering like a /tmp file. get-then-store is not atomic; two
        // simultaneous starts can race (single-writer in practice).
        let lock_key = format!("openarm01_arm_{arm_id}_instance_lock");
        if let Some(held) = datastore::get(&node_runner, lock_key.as_str(), DATASTORE_TIMEOUT).await? {
            panic!("instance lock {lock_key} held by {}", held.last_modified_by);
        }
        datastore::store(
            &node_runner,
            lock_key.as_str(),
            b"locked".to_vec(),
            Encoding::TEXT_PLAIN,
            DATASTORE_TIMEOUT,
        )
        .await?;

        // Shutdown: register the lock-release hook right after acquiring the lock,
        // so a panic during bringup still releases the key (dropping `shutdown_tx`
        // completes `shutdown_rx`, so the hook runs). On a normal stop the control
        // task eases to the ready pose and disables the motors (the sole motor
        // writer), signalling `shutdown_tx` when done; this hook waits for that,
        // then removes the datastore lock. The runtime fires it on every stop path
        // with the messenger connected and awaits it before exit.
        let (shutdown_tx, shutdown_rx) = oneshot::channel::<()>();
        {
            let runner = node_runner.clone();
            let lock_key = lock_key.clone();
            node_runner.on_shutdown(async move {
                let _ = shutdown_rx.await;
                if let Err(e) = datastore::remove(&runner, lock_key.as_str(), LOCK_REMOVE_TIMEOUT).await {
                    warn!("failed to remove lock {lock_key}: {e}");
                }
            });
        }

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

        // Bidirectional stream plumbing: the listener keeps the latest well-formed
        // `joint_commands` setpoint for the control loop, and the publisher emits
        // the measured joint state and end-effector pose on `joint_states` /
        // `cartesian_states`, always on, at its own rate.
        let (joint_command_tx, joint_command_rx) = watch::channel(None);
        let (measured_tx, measured_rx) = watch::channel(None);
        tokio::spawn(stream::run_joint_command_listener(node_runner.clone(), arm_id, joint_command_tx));
        tokio::spawn(stream::run_state_publisher(
            node_runner.clone(),
            arm_id,
            Duration::from_micros(1_000_000 / params.state_rate_hz as u64),
            measured_rx,
        ));
        let wiring = stream::StreamWiring { joint: joint_command_rx, measured: measured_tx };

        // Single control task (the only motor writer): eases to the ready pose on
        // startup, then follows the active command stream (or holds) between
        // moves, preempting into a joint or Cartesian trajectory while a move goal
        // runs. Its action handlers only admit goals and hand them over; both
        // actions are exposed before anything spawns, so a failed registration
        // fails bringup here.
        control::spawn(&node_runner, arm.clone(), cfg, model, wiring, shutdown_tx).await?;

        Ok(())
    })
}
