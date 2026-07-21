mod command_stream;
mod follow;
mod geometry;
mod stream;

use follow::ControlConfig;
use openarm_can::{CallbackMode, GripperCan, v20};
use peppygen::exposed_services::ready::is_ready;
use peppygen::{NodeBuilder, Parameters, Result};
use peppylib::datastore::{self, Encoding};

use std::sync::atomic::{AtomicBool, Ordering};
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
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::INFO)
        .init();

    NodeBuilder::new().run(|params: Parameters, node_runner| async move {
        // Pairing stamps read the daemon-resolved clock (sim time under a
        // simulated clock), so state consumers age samples on one timeline.
        peppygen::clock::init(&node_runner).await?;

        let gripper_id = params.gripper_id;
        // Resolves this side's signed opening direction (the two v2 grippers are
        // mechanically mirrored) and rejects an out-of-range id at bringup.
        let motor_geometry = geometry::Geometry::from_gripper_id(gripper_id).unwrap_or_else(|| {
            panic!("gripper_id must be 0 (left) or 1 (right), got {gripper_id}")
        });
        let can_interface = params.can_interface.clone();

        // Rates feed `Duration::from_micros(1_000_000 / rate)`, so a rate above
        // 1 MHz would round to a 0 µs period; no real deployment approaches that,
        // so just guard against zero.
        assert!(params.control_rate_hz > 0, "control_rate_hz must be > 0");
        assert!(params.state_rate_hz > 0, "state_rate_hz must be > 0");
        assert!(
            params.speed_rad_s.is_finite() && params.speed_rad_s > 0.0,
            "speed_rad_s must be a positive finite number"
        );
        assert!(
            params.force_limit_pu.is_finite() && (0.0..=1.0).contains(&params.force_limit_pu),
            "force_limit_pu must be in [0, 1]"
        );

        let cfg = ControlConfig {
            cycle_period: Duration::from_micros(1_000_000 / params.control_rate_hz as u64),
            recv_timeout_us: i32::try_from(params.recv_timeout_us).unwrap_or_else(|_| {
                panic!(
                    "recv_timeout_us ({}) exceeds i32::MAX",
                    params.recv_timeout_us
                )
            }),
            geometry: motor_geometry,
            speed_rad_s: params.speed_rad_s,
            force_limit_pu: params.force_limit_pu,
        };

        // Instance lock: crash if another instance with the same gripper_id is
        // running. Held in the core-node datastore (released from the on_shutdown
        // hook below), so a lock leaked by a hard crash clears with the stack
        // instead of lingering like a /tmp file. get-then-store is not atomic; two
        // simultaneous starts can race (single-writer in practice). Same scheme as
        // openarm_arm.
        let lock_key = format!("openarm_gripper_v2_{gripper_id}_instance_lock");
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

        // Lock-release hook, registered first so it runs last (after the
        // motor-disable hook below). The runtime fires it on every stop path with
        // the messenger still connected, so the key never outlives the process.
        {
            let runner = node_runner.clone();
            let lock_key = lock_key.clone();
            node_runner.on_shutdown(async move {
                if let Err(e) =
                    datastore::remove(&runner, lock_key.as_str(), LOCK_REMOVE_TIMEOUT).await
                {
                    warn!("failed to remove lock {lock_key}: {e}");
                }
            });
        }

        // Hardware bringup: the v2 pinch gripper runs in POS_FORCE mode, so init the motor
        // with that control mode (mirrors enactic's test_gripper_posforce init sequence).
        info!("opening CAN interface {can_interface} (FD={ENABLE_FD})");
        let mut gripper = GripperCan::new(&can_interface, ENABLE_FD).expect("GripperCan::new");
        gripper.init_motor_pos_force(
            v20::GRIPPER_MOTOR_TYPE,
            v20::GRIPPER_SEND_ID,
            v20::GRIPPER_RECV_ID,
        );

        // IGNORE during enable so ACK frames aren't processed as state updates,
        // then switch to STATE before the control loop (matches demo.cpp bringup pattern).
        gripper.set_callback_mode(CallbackMode::Ignore);
        gripper.enable_all();
        tokio::time::sleep(POST_ENABLE_SLEEP).await;
        gripper.recv_all(BRINGUP_RECV_US);

        gripper.set_callback_mode(CallbackMode::State);

        // Return to closed (motor angle = 0.0 rad) before serving requests.
        info!("returning to zero");
        gripper.set_position(0.0, cfg.speed_rad_s, cfg.force_limit_pu);
        gripper.recv_all(BRINGUP_RECV_US);
        info!("gripper ready");

        let gripper = Arc::new(Mutex::new(gripper));

        // Always-on gripper_states publisher: reads the motor's cached state at
        // state_rate_hz and emits the opening. It issues no CAN traffic of its
        // own, so it never contends with the follow loop for the bus.
        tokio::spawn(stream::run(
            node_runner.clone(),
            gripper_id,
            params.state_rate_hz,
            motor_geometry,
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
                // still-live follow loop can't interleave CAN traffic before
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

        // is_ready service: false until bringup and control wiring complete, then
        // true. The real robot_initializer polls this (openarm_hardware_ready) to
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

        // Stream listener -> follow loop: the listener keeps the latest streamed
        // opening addressed to this gripper, the follow loop drives the motor
        // toward it.
        let (cmd_tx, cmd_rx) = watch::channel(None);
        // Supervised: if the command consumer ever exits, whether a clean close
        // on shutdown or an unexpected error, streamed openings are dead, so
        // cancel the node to restart it rather than leaving it healthy but inert.
        {
            let runner = node_runner.clone();
            let token = node_runner.cancellation_token().clone();
            tokio::spawn(async move {
                command_stream::run(runner, cmd_tx, token.clone()).await;
                token.cancel();
            });
        }
        tokio::spawn(follow::run(
            gripper,
            cmd_rx,
            cfg,
            node_runner.cancellation_token().clone(),
        ));

        // Motor enabled and follow loop running: report ready so the
        // robot_initializer can release the gate.
        ready.store(true, Ordering::SeqCst);

        Ok(())
    })
}
