//! One arm of the openarm01 robot (real hardware); instantiate twice, one per
//! side, with a distinct `arm_id`. A follower of the bimanual hub: it owns the
//! hardware control loop (gravity/Coriolis/friction feedforward from the
//! in-process srs_model, plus MIT control) and tracks the hub's governed
//! setpoint, reporting measured state on the always-on `arm_states` stream. The
//! hub (openarm01_backbone) owns all trajectory generation, stream following, and
//! self-collision governing, so this node carries no motion logic of its own; on
//! shutdown it disables the motors and lets the arm go limp.

mod control;
mod friction;
mod stream;

use control::ControlConfig;
use openarm_can::{ArmCan, CallbackMode, v10};
use peppygen::exposed_services::openarm01_hardware_ready::v1::is_ready;
use peppygen::{NodeBuilder, Parameters, Result};
use peppylib::datastore::{self, Encoding};
use srs_model::nalgebra::Isometry3;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;
use tokio::sync::{oneshot, watch};
use tracing::{error, info, warn};

/// Degrees of freedom of the arm.
pub const ARM_DOF: usize = 7;
/// One joint-space vector (positions, velocities, or torques), j1..j7.
pub type JointVec = [f64; ARM_DOF];

/// `arm_id` values (0 = left, 1 = right). Geometry and joint limits come from the
/// URDF via `base_link`; `arm_id` selects which governed setpoints to follow and
/// which arm the measured state is tagged with.
const ARM_ID_LEFT: u8 = 0;
const ARM_ID_RIGHT: u8 = 1;

/// Human-readable side for the given `arm_id`, panicking on an unknown value so a
/// misconfigured run fails loudly at startup.
fn side_label(arm_id: u8) -> &'static str {
    match arm_id {
        ARM_ID_LEFT => "left",
        ARM_ID_RIGHT => "right",
        other => {
            panic!("arm_id must be {ARM_ID_LEFT} (left) or {ARM_ID_RIGHT} (right), got {other}")
        }
    }
}

// Sleep durations chosen to match ROS2 enactic/openarm_ros2 v10_simple_hardware behaviour.
const POST_ENABLE_SLEEP: Duration = Duration::from_millis(100);
const BRINGUP_RECV_US: i32 = 500;
const ENABLE_FD: bool = true;
const DATASTORE_TIMEOUT: Duration = Duration::from_secs(3);
/// Tighter bound for shutdown lock removal so motor disable + lock removal stays
/// inside the default 5 s shutdown grace window.
const LOCK_REMOVE_TIMEOUT: Duration = Duration::from_secs(1);

fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::INFO)
        .init();

    NodeBuilder::new().run(|params: Parameters, node_runner| async move {
        let arm_id = params.arm_id;
        let can_interface = params.can_interface.clone();

        // Rates feed `Duration::from_micros(1_000_000 / rate)`, so a rate above
        // 1 MHz would round to a 0 µs period; no real deployment approaches that,
        // so just guard against zero.
        assert!(params.control_rate_hz > 0, "control_rate_hz must be > 0");
        assert!(params.state_rate_hz > 0, "state_rate_hz must be > 0");
        let side = side_label(arm_id);

        // Build the srs_model arm from the embedded OpenArm description: forward
        // kinematics for the in-process gravity/Coriolis feedforward, plus joint limits
        // off the same parsed chain. The elbow singularity margin is a control policy the
        // description exports as a constant; apply it so limits() carries it. A non-SRS
        // or short chain from base_link errors here.
        let model = srs_model::Arm::from_urdf(openarm_description::urdf(), &params.base_link)
            .map(|arm| {
                arm.with_lower_floor(
                    openarm_description::ELBOW_JOINT_INDEX,
                    openarm_description::ELBOW_SINGULARITY_FLOOR_RAD,
                )
            })
            .unwrap_or_else(|e| panic!("build arm model from base '{}': {e}", params.base_link));
        info!("model loaded (base '{}')", params.base_link);

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
                params.kp1, params.kp2, params.kp3, params.kp4, params.kp5, params.kp6, params.kp7,
            ],
            kd: [
                params.kd1, params.kd2, params.kd3, params.kd4, params.kd5, params.kd6, params.kd7,
            ],
            cycle_period: Duration::from_micros(1_000_000 / params.control_rate_hz as u64),
            recv_timeout_us: i32::try_from(params.recv_timeout_us).unwrap_or_else(|_| {
                panic!(
                    "recv_timeout_us ({}) exceeds i32::MAX",
                    params.recv_timeout_us
                )
            }),
            limits: model.limits(),
        };

        info!(
            "config: arm_id={arm_id} ({side}) rate={}Hz recv_timeout={}us",
            params.control_rate_hz, cfg.recv_timeout_us
        );
        info!("config: kp={:?} kd={:?}", cfg.kp, cfg.kd);

        // Instance lock: crash if another instance with the same arm_id is
        // running. Held in the core-node datastore (released from the on_shutdown
        // hook below), so a lock leaked by a hard crash clears with the stack
        // instead of lingering like a /tmp file. get-then-store is not atomic; two
        // simultaneous starts can race (single-writer in practice).
        let lock_key = format!("openarm01_arm_{arm_id}_instance_lock");
        if let Some(held) =
            datastore::get(&node_runner, lock_key.as_str(), DATASTORE_TIMEOUT).await?
        {
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
        // task disables the motors (the sole motor writer) and signals
        // `shutdown_tx` when done; this hook waits for that,
        // then removes the datastore lock. The runtime fires it on every stop path
        // with the messenger connected and awaits it before exit.
        let (shutdown_tx, shutdown_rx) = oneshot::channel::<()>();
        {
            let runner = node_runner.clone();
            let lock_key = lock_key.clone();
            node_runner.on_shutdown(async move {
                let _ = shutdown_rx.await;
                if let Err(e) =
                    datastore::remove(&runner, lock_key.as_str(), LOCK_REMOVE_TIMEOUT).await
                {
                    warn!("failed to remove lock {lock_key}: {e}");
                }
            });
        }

        // Hardware bringup: sequence mirrors ROS2 v10_simple_hardware on_init/on_activate.
        info!("opening CAN interface {can_interface} (FD={ENABLE_FD})");
        let mut arm = ArmCan::new(&can_interface, ENABLE_FD).expect("ArmCan::new");
        arm.init_motors(
            &v10::ARM_MOTOR_TYPES,
            &v10::ARM_SEND_IDS,
            &v10::ARM_RECV_IDS,
        );
        arm.set_callback_mode(CallbackMode::Ignore);
        arm.enable_all();
        tokio::time::sleep(POST_ENABLE_SLEEP).await;
        arm.recv_all(BRINGUP_RECV_US);
        arm.set_callback_mode(CallbackMode::State);
        // recv_all in State mode populates initial joint state; without it get_state() returns zeros.
        arm.recv_all(BRINGUP_RECV_US);
        info!("arm ready");

        let arm = Arc::new(Mutex::new(arm));

        // is_ready service: false until bringup and control wiring complete, then
        // true. The real robot_initializer polls this (openarm01_hardware_ready) to
        // gate the whole robot.
        let ready = Arc::new(AtomicBool::new(false));
        {
            let runner = node_runner.clone();
            let ready = ready.clone();
            tokio::spawn(async move {
                loop {
                    if let Err(e) = is_ready::handle_next_request(&runner, |_req| {
                        Ok(is_ready::Response::new(ready.load(Ordering::SeqCst)))
                    })
                    .await
                    {
                        error!("is_ready: {e}");
                    }
                }
            });
        }

        // Stream plumbing: the listener keeps the latest governed setpoint for the
        // control loop, and the publisher emits the measured joint state at its
        // own rate (the hub consumes it).
        let (governed_tx, governed_rx) = watch::channel(None);
        let (measured_tx, measured_rx) = watch::channel(None);
        tokio::spawn(stream::run_governed_setpoint_listener(
            node_runner.clone(),
            arm_id,
            governed_tx,
        ));
        tokio::spawn(stream::run_state_publisher(
            node_runner.clone(),
            arm_id,
            Duration::from_micros(1_000_000 / params.state_rate_hz as u64),
            measured_rx,
        ));
        let wiring = stream::StreamWiring {
            governed: governed_rx,
            measured: measured_tx,
        };

        // Single control task (the only motor writer): follows the governed
        // setpoint with in-process feedforward and a final limit clamp, and on
        // shutdown disables the motors.
        control::spawn(&node_runner, arm.clone(), cfg, model, wiring, shutdown_tx).await?;

        // Motors enabled, initial state populated, control loop running: report
        // ready so the robot_initializer can release the gate.
        ready.store(true, Ordering::SeqCst);

        Ok(())
    })
}
