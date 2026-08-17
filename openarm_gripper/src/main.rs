mod command_stream;
mod drive;
mod follow;
mod geometry;
mod health;
mod stream;

use follow::ControlConfig;
use openarm_can::{GripperCan, Mit, v10};
use peppygen::exposed_services::ready::is_ready;
use peppygen::{NodeBuilder, Parameters, Result};
use peppylib::datastore::{self, Encoding};
use peppylib::runtime::CancellationToken;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;
use tokio::sync::{oneshot, watch};
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
/// Bound on the shutdown hook that awaits the health task's final flush.
/// Hooks share one grace window and this one runs before the motor-disable
/// hook, so an unbounded wait on a stalled publish would hold the motors
/// energised until the force-kill deadline. Sized well inside even the
/// minimum 1 s grace window so the disable hook keeps most of the budget.
const HEALTH_FLUSH_TIMEOUT: Duration = Duration::from_millis(300);

/// Adapts a CAN failure into the runtime error type so bring-up failures
/// return through the runtime's error path, which runs the shutdown hooks.
/// A panic would skip them, leaving the motor energised and the instance
/// lock held. Repeated per node because peppygen is generated per node; no
/// shared crate can name its Error type.
fn can_err(e: openarm_can::CanError) -> peppygen::Error {
    peppygen::Error::Io(std::io::Error::other(e))
}

/// The tick period for a whole-hertz rate, refused at startup when it
/// rounds to zero, so a bad rate fails before the motor is energised and
/// with its own name in the message.
fn period_from_hz(rate_hz: u32, name: &str) -> Duration {
    assert!(rate_hz > 0, "{name} must be > 0");
    let period = Duration::from_micros(1_000_000 / u64::from(rate_hz));
    assert!(
        !period.is_zero(),
        "{name} {rate_hz} rounds to a zero-microsecond period"
    );
    period
}

/// Cancel the node when a supervised task exits while it should still be
/// running. An unsupervised dead loop leaves the node reachable but inert;
/// a dead follow loop is worse, because the health task would keep judging
/// a frozen state cache. Watching the JoinHandle means a panic lands here
/// too, not only a clean return.
fn supervise(task: tokio::task::JoinHandle<()>, what: &'static str, token: CancellationToken) {
    tokio::spawn(async move {
        let _ = task.await;
        if !token.is_cancelled() {
            error!("{what} exited unexpectedly");
            follow::HARD_FAULT.store(true, Ordering::SeqCst);
        }
        token.cancel();
    });
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
        // Names this gripper on the alert wire. Exhaustive so a value
        // outside the convention is refused rather than published as the
        // wrong side.
        let alert_source = match gripper_id {
            0 => "left gripper",
            1 => "right gripper",
            other => {
                return Err(peppygen::Error::Io(std::io::Error::other(format!(
                    "gripper_id must be 0 (left) or 1 (right), got {other}"
                ))));
            }
        };
        let can_interface = params.can_interface.clone();

        let cycle_period = period_from_hz(params.control_rate_hz, "control_rate_hz");
        let state_period = period_from_hz(params.state_rate_hz, "state_rate_hz");
        // Bounded so a config typo cannot park recv_all in a long ppoll
        // while it holds the CAN mutex the shutdown hooks need: 100 ms keeps
        // the whole hook sequence inside even the minimum 1 s grace window,
        // and real configs run around 1 ms.
        assert!(
            params.recv_timeout_us <= 100_000,
            "recv_timeout_us must be at most 100_000 (100 ms), got {}",
            params.recv_timeout_us
        );

        let cfg = ControlConfig {
            cycle_period,
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

        // Datasheet ratings resolve before the motor is energised: an error
        // return here still runs the hooks above, and no panic can strand an
        // enabled motor. The DM4310's configured trip derates above its
        // datasheet peak, so the registers would resolve to the datasheet
        // anyway.
        let ratings = v10::GRIPPER_MOTOR_TYPE.ratings().ok_or_else(|| {
            peppygen::Error::Io(std::io::Error::other(
                "no datasheet ratings for the gripper motor type",
            ))
        })?;

        // Enable and verify the motor acknowledges torque authority, then
        // return to closed (motor angle = 0.0 rad) before serving requests.
        // Errors return through the runtime so the hooks above disable the
        // motor and release the lock.
        gripper
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .enable_and_confirm(
                openarm_can::ENABLE_ATTEMPTS,
                POST_ENABLE_SLEEP,
                BRINGUP_RECV_US,
            )
            .map_err(|e| peppygen::Error::Io(std::io::Error::other(e)))?;
        {
            let mut g = gripper.lock().unwrap_or_else(|e| e.into_inner());
            info!("returning to zero");
            g.mit_control(follow::KP, follow::KD, 0.0, 0.0, 0.0)
                .map_err(can_err)?;
            g.recv_all(BRINGUP_RECV_US).map_err(can_err)?;
        }
        info!("gripper ready (motor confirmed enabled)");

        // Always-on gripper_states publisher: reads the motor's cached state at
        // state_rate_hz and emits the opening. It issues no CAN traffic of its
        // own, so it never contends with the follow loop for the bus.
        let publisher = tokio::spawn(stream::run(
            node_runner.clone(),
            state_period,
            gripper.clone(),
            node_runner.cancellation_token().clone(),
        ));
        supervise(
            publisher,
            "state publisher",
            node_runner.cancellation_token().clone(),
        );

        // Motor condition telemetry and operator alerts, off the same cached
        // state. The final flush round after cancellation is only awaited if
        // a shutdown hook waits for it: the runtime awaits hooks, not plain
        // tasks racing the token. Dropping the sender (return or panic)
        // completes the receiver, so the hook cannot hang on a dead task.
        let (health_done_tx, health_done_rx) = oneshot::channel::<()>();
        let health_task = tokio::spawn({
            let runner = node_runner.clone();
            let alert_source = alert_source.to_string();
            let gripper = gripper.clone();
            let cycle_period = cfg.cycle_period;
            let token = node_runner.cancellation_token().clone();
            async move {
                let _done = health_done_tx;
                health::run(runner, alert_source, gripper, ratings, cycle_period, token).await;
            }
        });
        node_runner.on_shutdown(async move {
            if tokio::time::timeout(HEALTH_FLUSH_TIMEOUT, health_done_rx)
                .await
                .is_err()
            {
                warn!("health flush missed its shutdown budget; disabling without it");
            }
        });
        supervise(
            health_task,
            "health publisher",
            node_runner.cancellation_token().clone(),
        );

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
        let commands = tokio::spawn(command_stream::run(
            node_runner.clone(),
            cmd_tx,
            node_runner.cancellation_token().clone(),
        ));
        supervise(
            commands,
            "command stream",
            node_runner.cancellation_token().clone(),
        );
        let (follow_started_tx, follow_started_rx) = oneshot::channel::<()>();
        let follower = tokio::spawn(follow::run(
            gripper,
            cmd_rx,
            cfg,
            node_runner.cancellation_token().clone(),
            follow_started_tx,
        ));
        supervise(
            follower,
            "follow loop",
            node_runner.cancellation_token().clone(),
        );

        // The ack arrives once the follow loop is actually running; a task
        // that dies first drops the sender and the error return runs the
        // disable hooks. Reporting ready any earlier would let the robot
        // gate open on a spawned-but-dead controller.
        follow_started_rx.await.map_err(|_| {
            peppygen::Error::Io(std::io::Error::other("the follow loop never started"))
        })?;
        ready.store(true, Ordering::SeqCst);

        Ok(())
    })?;

    // The runtime has returned, so the shutdown hooks (motor disable, lock
    // release) have already run; exiting non-zero here makes the daemon
    // record a hard CAN fault as failed instead of finished.
    if follow::HARD_FAULT.load(Ordering::SeqCst) {
        return Err(peppygen::Error::Io(std::io::Error::other(
            "hard fault stopped this node; the log names the failing component",
        )));
    }
    Ok(())
}
