mod control;
mod geometry;
mod stream;

use openarm_can::{CallbackMode, GripperCan, v10};
use control::{ControlConfig, run_move_gripper};
use peppygen::exposed_services::openarm01_gripper::v1::get_gripper_id;
use peppygen::{NodeBuilder, Parameters, Result};

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

        // Rates feed `Duration::from_micros(1_000_000 / rate)`, so a rate above
        // 1 MHz would round to a 0 µs period; no real deployment approaches that,
        // so just guard against zero.
        assert!(params.control_rate_hz > 0, "control_rate_hz must be > 0");
        assert!(params.state_rate_hz > 0, "state_rate_hz must be > 0");

        let cfg = ControlConfig {
            cycle_period: Duration::from_micros(1_000_000 / params.control_rate_hz as u64),
            recv_timeout_us: i32::try_from(params.recv_timeout_us)
                .unwrap_or_else(|_| panic!("recv_timeout_us ({}) exceeds i32::MAX", params.recv_timeout_us)),
            position_tolerance_m: params.position_tolerance,
            motion_timeout: Duration::from_secs_f64(params.motion_timeout_s),
        };

        // Instance lock: crash if another instance with the same gripper_id is running.
        let lock_path = format!("/tmp/openarm_gripper_{gripper_id}.lock");
        std::fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&lock_path)
            .unwrap_or_else(|e| panic!("instance lock {lock_path} held: {e}"));

        // Hardware bringup: mirrors ROS2 v10_simple_hardware on_init / on_configure / on_activate.
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

        // Always-on gripper_states publisher: reads the motor's cached state at
        // state_rate_hz and emits the opening. It issues no CAN traffic of its
        // own, so it never contends with the move control loop for the bus.
        tokio::spawn(stream::run(
            node_runner.clone(),
            gripper_id,
            params.state_rate_hz,
            gripper.clone(),
            node_runner.cancellation_token().clone(),
        ));

        // Shutdown task: disables the motor and releases the lock. `peppy node
        // stop` cancels in-band via the runtime cancellation token (not a unix
        // signal), so observe that too or the motor would stay energised on a
        // daemon stop.
        {
            let gripper = gripper.clone();
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
                info!("shutdown: disabling motor, releasing lock");
                {
                    // unwrap_or_else: recover even if poisoned (panic in control loop)
                    // so disable_all() always runs and the motor doesn't stay energised.
                    let mut g = gripper.lock().unwrap_or_else(|e| e.into_inner());
                    g.disable_all();
                    std::thread::sleep(POST_DISABLE_SLEEP);
                    g.recv_all(BRINGUP_RECV_US);
                }
                if let Err(e) = std::fs::remove_file(&lock_path) {
                    warn!("failed to remove lock {lock_path}: {e}");
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
