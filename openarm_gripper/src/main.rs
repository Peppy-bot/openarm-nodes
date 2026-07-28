mod command_stream;
mod follow;
mod geometry;
mod stream;

use follow::ControlConfig;
use openarm_can::{GripperCan, Mit, v10};
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
const BRINGUP_RECV_US: u32 = 2000;
const ENABLE_FD: bool = true;
const DATASTORE_TIMEOUT: Duration = Duration::from_secs(3);
/// Tighter bound for shutdown lock removal so disable + drain + removal stays
/// inside the default 5 s shutdown grace window.
const LOCK_REMOVE_TIMEOUT: Duration = Duration::from_secs(1);

/// Adapts a CAN failure into the runtime error type so bring-up failures
/// return through the runtime's error path, which runs the shutdown hooks.
/// A panic would skip them, leaving the motor energised and the instance
/// lock held. Repeated per node because peppygen is generated per node; no
/// shared crate can name its Error type.
fn can_err(e: openarm_can::CanError) -> peppygen::Error {
    peppygen::Error::Io(std::io::Error::other(e))
}

fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::INFO)
        .init();

    NodeBuilder::new().run(|params: Parameters, node_runner| async move {
        // Pairing stamps read the daemon-resolved clock (sim time under a
        // simulated clock), so state consumers age samples on one timeline.
        peppygen::clock::init(&node_runner).await?;

        let gripper_id = params.gripper_id;
        let can_interface = params.can_interface.clone();

        // Rates feed `Duration::from_micros(1_000_000 / rate)`, so a rate above
        // 1 MHz would round to a 0 µs period; no real deployment approaches that,
        // so just guard against zero.
        assert!(params.control_rate_hz > 0, "control_rate_hz must be > 0");
        assert!(params.state_rate_hz > 0, "state_rate_hz must be > 0");
        // Bounded so a config typo cannot park recv_all in a near-eternal
        // ppoll while it holds the CAN mutex the shutdown hook needs.
        assert!(
            params.recv_timeout_us <= 1_000_000,
            "recv_timeout_us must be at most 1_000_000 (1s), got {}",
            params.recv_timeout_us
        );

        let cfg = ControlConfig {
            cycle_period: Duration::from_micros(1_000_000 / params.control_rate_hz as u64),
            recv_timeout_us: params.recv_timeout_us,
        };

        // Instance lock: crash if another instance with the same gripper_id is
        // running. Held in the core-node datastore (released from the on_shutdown
        // hook below), so a lock leaked by a hard crash clears with the stack
        // instead of lingering like a /tmp file. get-then-store is not atomic; two
        // simultaneous starts can race (single-writer in practice). Same scheme as
        // openarm_arm.
        let lock_key = format!("openarm_gripper_{gripper_id}_instance_lock");
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

        // Hardware bringup: mirrors ROS2 v10_simple_hardware on_init / on_configure / on_activate.
        info!("opening CAN interface {can_interface} (FD={ENABLE_FD})");
        let gripper: GripperCan<Mit> = GripperCan::open_mit(
            &can_interface,
            ENABLE_FD,
            v10::GRIPPER_MOTOR_TYPE,
            v10::GRIPPER_SEND_ID,
            v10::GRIPPER_RECV_ID,
        )
        .map_err(can_err)?;
        let gripper = Arc::new(Mutex::new(gripper));

        // Motor-disable hook, registered before the motor is ever enabled and
        // second overall so it runs first at shutdown (before the lock-release
        // hook above). The runtime fires it on every stop path and on a
        // bring-up error return, so the motor never stays energised.
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
                if let Err(e) = g.disable_all() {
                    error!("disable motor: {e}");
                }
                std::thread::sleep(POST_DISABLE_SLEEP);
                if let Err(e) = g.recv_all(BRINGUP_RECV_US) {
                    warn!("drain disable replies: {e}");
                }
            });
        }

        // Enable, settle, drain the enable replies so bring-up ACK frames
        // aren't decoded as state, then return to closed (motor angle =
        // 0.0 rad) before serving requests. Errors return through the
        // runtime so the hooks above disable the motor and release the lock.
        gripper
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .enable_all()
            .map_err(can_err)?;
        tokio::time::sleep(POST_ENABLE_SLEEP).await;
        {
            let mut g = gripper.lock().unwrap_or_else(|e| e.into_inner());
            g.drain(BRINGUP_RECV_US).map_err(can_err)?;
            info!("returning to zero");
            g.mit_control(follow::KP, follow::KD, 0.0, 0.0, 0.0)
                .map_err(can_err)?;
            g.recv_all(BRINGUP_RECV_US).map_err(can_err)?;
        }
        info!("gripper ready");

        // Always-on gripper_states publisher: reads the motor's cached state at
        // state_rate_hz and emits the opening. It issues no CAN traffic of its
        // own, so it never contends with the follow loop for the bus.
        tokio::spawn(stream::run(
            node_runner.clone(),
            params.state_rate_hz,
            gripper.clone(),
            node_runner.cancellation_token().clone(),
        ));

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
                        // A latched fault revokes readiness immediately:
                        // services stay reachable while shutdown hooks run,
                        // and the gate must not see ready during a disable.
                        Ok(is_ready::Response::new(
                            ready.load(Ordering::SeqCst)
                                && !follow::HARD_FAULT.load(Ordering::SeqCst),
                        ))
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
                if !token.is_cancelled() {
                    error!("command stream exited unexpectedly");
                    follow::HARD_FAULT.store(true, Ordering::SeqCst);
                }
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
    })?;

    // The runtime has returned, so the shutdown hooks (motor disable, lock
    // release) have already run; exiting non-zero here makes the daemon
    // record a hard CAN fault as failed instead of finished.
    if follow::HARD_FAULT.load(Ordering::SeqCst) {
        return Err(peppygen::Error::Io(std::io::Error::other(
            "persistent CAN fault stopped the follow loop",
        )));
    }
    Ok(())
}
