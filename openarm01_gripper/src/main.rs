mod command_stream;
mod control;
mod follow;
mod geometry;
mod stream;

use openarm_can::{CallbackMode, GripperCan, v10};
use control::{ControlConfig, run_move_gripper};
use peppygen::exposed_services::openarm01_gripper::v1::get_gripper_id;
use peppygen::{NodeBuilder, Parameters, Result};
use peppylib::datastore::{self, Encoding};

use std::sync::atomic::AtomicBool;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use tokio::sync::watch;
use tracing::{error, info, warn};

// Mirrors ROS2 v10_simple_hardware on_activate / on_deactivate sleep durations.
const POST_ENABLE_SLEEP: Duration = Duration::from_millis(100);
const POST_DISABLE_SLEEP: Duration = Duration::from_millis(100);
const BRINGUP_RECV_US: i32 = 2000;
const ENABLE_FD: bool = true;
const DATASTORE_TIMEOUT: Duration = Duration::from_secs(3);
/// Tighter bound for shutdown lock removal so disable + drain + removal stays
/// inside the default 5 s shutdown grace window.
const LOCK_REMOVE_TIMEOUT: Duration = Duration::from_secs(1);

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
        assert!(
            params.stream_timeout_s.is_finite() && params.stream_timeout_s > 0.0,
            "stream_timeout_s must be a positive finite number"
        );

        let cfg = ControlConfig {
            cycle_period: Duration::from_micros(1_000_000 / params.control_rate_hz as u64),
            recv_timeout_us: i32::try_from(params.recv_timeout_us)
                .unwrap_or_else(|_| panic!("recv_timeout_us ({}) exceeds i32::MAX", params.recv_timeout_us)),
            position_tolerance_m: params.position_tolerance,
            motion_timeout: Duration::from_secs_f64(params.motion_timeout_s),
            stream_timeout: Duration::from_secs_f64(params.stream_timeout_s),
        };

        // Instance lock: crash if another instance with the same gripper_id is
        // running. Held in the core-node datastore (released from the on_shutdown
        // hook below), so a lock leaked by a hard crash clears with the stack
        // instead of lingering like a /tmp file. get-then-store is not atomic; two
        // simultaneous starts can race (single-writer in practice). Same scheme as
        // openarm01_arm.
        let lock_key = format!("openarm01_gripper_{gripper_id}_instance_lock");
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

        // Lock-release hook, registered first so it runs last (after the
        // motor-disable hook below). The runtime fires it on every stop path with
        // the messenger still connected, so the key never outlives the process.
        {
            let runner = node_runner.clone();
            let lock_key = lock_key.clone();
            node_runner.on_shutdown(async move {
                if let Err(e) = datastore::remove(&runner, lock_key.as_str(), LOCK_REMOVE_TIMEOUT).await {
                    warn!("failed to remove lock {lock_key}: {e}");
                }
            });
        }

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

        // Motor-disable hook, registered second so it runs first at shutdown
        // (before the lock-release hook above). The runtime fires it on every stop
        // path (signals, `peppy node stop`, daemon loss) and awaits it before
        // exiting, so the motor never stays energised.
        {
            let gripper = gripper.clone();
            node_runner.on_shutdown(async move {
                info!("shutdown: disabling motor");
                // Hold the lock across the whole disable -> settle -> drain so a
                // still-live follow/move loop can't interleave CAN traffic before
                // the disable ACKs are drained. Blocking sleep (not tokio) keeps the
                // guard held, which it could not be across an await.
                // unwrap_or_else: recover even if poisoned (panic in control loop)
                // so disable_all() always runs and the motor doesn't stay energised.
                let mut g = gripper.lock().unwrap_or_else(|e| e.into_inner());
                g.disable_all();
                std::thread::sleep(POST_DISABLE_SLEEP);
                g.recv_all(BRINGUP_RECV_US);
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

        // One busy gate, shared by the move action and the follow loop so only one
        // drives the CAN bus at a time.
        let busy = Arc::new(AtomicBool::new(false));

        // Stream listener -> follow loop: the listener keeps the latest streamed
        // opening addressed to this gripper, the follow loop drives the motor
        // toward it between moves.
        let (cmd_tx, cmd_rx) = watch::channel(None);
        tokio::spawn(command_stream::run(
            node_runner.clone(),
            gripper_id,
            cmd_tx,
            node_runner.cancellation_token().clone(),
        ));
        tokio::spawn(follow::run(
            gripper.clone(),
            busy.clone(),
            cmd_rx,
            cfg.clone(),
            node_runner.cancellation_token().clone(),
        ));

        // move_gripper: direct-setpoint control loop.
        tokio::spawn(run_move_gripper(node_runner, gripper, cfg, busy));

        Ok(())
    })
}
