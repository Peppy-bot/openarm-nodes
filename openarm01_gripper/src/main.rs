mod control;
mod joint_limits;

use openarm_can::{CallbackMode, GripperCan, v10};
use control::{ControlConfig, run_move_gripper};
use peppygen::exposed_services::openarm01_gripper::v1::get_gripper_id;
use peppygen::{NodeBuilder, Parameters, Result};
use peppylib::datastore::{self, Encoding};

use std::sync::{Arc, Mutex};
use std::time::Duration;
use tokio::signal::unix::{SignalKind, signal};
use tracing::{error, info, warn};

// Mirrors ROS2 v10_simple_hardware on_activate / on_deactivate sleep durations.
const POST_ENABLE_SLEEP: Duration = Duration::from_millis(100);
const POST_DISABLE_SLEEP: Duration = Duration::from_millis(100);
const BRINGUP_RECV_US: i32 = 2000;
const ENABLE_FD: bool = true;

fn main() -> Result<()> {
    tracing_subscriber::fmt().with_max_level(tracing::Level::INFO).init();

    NodeBuilder::new().run(|params: Parameters, node_runner| async move {
        let gripper_id = params.gripper_id;
        let can_interface = params.can_interface.clone();

        assert!(params.control_rate_hz > 0, "control_rate_hz must be > 0");

        let cfg = ControlConfig {
            cycle_period: Duration::from_micros(1_000_000 / params.control_rate_hz as u64),
            recv_timeout_us: i32::try_from(params.recv_timeout_us)
                .unwrap_or_else(|_| panic!("recv_timeout_us ({}) exceeds i32::MAX", params.recv_timeout_us)),
            position_tolerance_m: params.position_tolerance,
            motion_timeout: Duration::from_secs_f64(params.motion_timeout_s),
        };

        // Instance lock in the datastore (shared across instances on the bound core
        // node), so a second instance with the same gripper_id refuses to start.
        // Get-then-store is non-atomic, so two *simultaneous* starts could both
        // pass; the realistic case (a leftover instance, then a new run) is
        // sequential and is caught. Released best-effort on shutdown below; a
        // hard-killed instance leaves a stale key the next start must clear.
        let lock_key = format!("openarm_gripper_lock_{gripper_id}");
        let instance_id = node_runner.processor().bound_instance_id().to_string();
        match datastore::get(&node_runner, lock_key.clone(), Some(Duration::from_secs(2))).await {
            Ok(Some(held)) => {
                panic!("instance lock '{lock_key}' already held by '{}'", held.last_modified_by)
            }
            Ok(None) => {}
            Err(e) => panic!("datastore get for lock '{lock_key}': {e}"),
        }
        datastore::store(
            &node_runner,
            lock_key.clone(),
            instance_id,
            Encoding::TEXT_PLAIN,
            Some(Duration::from_secs(2)),
        )
        .await
        .unwrap_or_else(|e| panic!("datastore store for lock '{lock_key}': {e}"));

        // Hardware bringup — mirrors ROS2 v10_simple_hardware on_init / on_configure / on_activate.
        info!("opening CAN interface {can_interface} (FD={ENABLE_FD})");
        let mut gripper = GripperCan::new(&can_interface, ENABLE_FD).expect("GripperCan::new");
        gripper.init_motor(v10::GRIPPER_MOTOR_TYPE, v10::GRIPPER_SEND_ID, v10::GRIPPER_RECV_ID);

        // IGNORE during enable so ACK frames aren't processed as state updates,
        // then switch to STATE before the control loop (matches demo.cpp bringup pattern).
        gripper.set_callback_mode(CallbackMode::Ignore);
        gripper.enable_all();
        tokio::time::sleep(POST_ENABLE_SLEEP).await;
        gripper.recv_all(BRINGUP_RECV_US);

        gripper.set_callback_mode(CallbackMode::State);

        // Return to closed (motor angle = 0.0 rad) before serving requests.
        info!("returning to zero");
        gripper.mit_control(control::KP, control::KD, 0.0, 0.0, 0.0);
        gripper.recv_all(BRINGUP_RECV_US);
        info!("gripper ready");

        let gripper = Arc::new(Mutex::new(gripper));

        // Shutdown task: disables the motor and releases the lock. `peppy node
        // stop` cancels in-band via the runtime cancellation token (not a unix
        // signal), so observe that too or the motor would stay energised on a
        // daemon stop.
        {
            let gripper = gripper.clone();
            let node_runner = node_runner.clone();
            let cancel = node_runner.cancellation_token().clone();
            let lock_key = lock_key.clone();
            tokio::spawn(async move {
                let mut sigint = signal(SignalKind::interrupt()).expect("sigint");
                let mut sigterm = signal(SignalKind::terminate()).expect("sigterm");
                tokio::select! {
                    _ = sigint.recv() => {},
                    _ = sigterm.recv() => {},
                    _ = cancel.cancelled() => {},
                }
                info!("shutdown: disabling motor, releasing lock");
                {
                    // unwrap_or_else: recover even if poisoned (panic in control loop)
                    // so disable_all() always runs and the motor doesn't stay energised.
                    let mut g = gripper.lock().unwrap_or_else(|e| e.into_inner());
                    g.disable_all();
                    std::thread::sleep(POST_DISABLE_SLEEP);
                    g.recv_all(BRINGUP_RECV_US);
                }
                // Best-effort: on a `node stop` the messaging session may already be
                // closing, so this can fail and leave a stale key (cleared on a later
                // start). Short timeout so motor-disable isn't held up.
                if let Err(e) =
                    datastore::remove(&node_runner, lock_key.clone(), Some(Duration::from_millis(500)))
                        .await
                {
                    warn!("failed to release datastore lock '{lock_key}': {e}");
                }
                // process::exit: peppylib runtime has no clean shutdown path; the
                // motor is already disabled above so this is safe.
                std::process::exit(0);
            });
        }

        // get_gripper_id service.
        {
            let runner = node_runner.clone();
            tokio::spawn(async move {
                loop {
                    if let Err(e) = get_gripper_id::handle_next_request(&runner, |_req| {
                        Ok(get_gripper_id::Response::new(gripper_id))
                    })
                    .await
                    {
                        error!("get_gripper_id: {e}");
                    }
                }
            });
        }

        // move_gripper: direct-setpoint control loop.
        tokio::spawn(run_move_gripper(node_runner, gripper, cfg));

        Ok(())
    })
}
