// Node composition: parses every parameter up front, builds the srs_model
// arm, takes the per-arm instance lock, brings up the CAN hardware, and wires
// the control loop to the stream tasks. All control, health, and stream logic
// lives in the sibling modules; this is only the assembly.

use crate::control::{self, ControlConfig};
use crate::health;
use crate::stream;
use openarm_can::ArmCan;
use openarm_description::{HardwareVersion, Side};
use peppygen::exposed_services::ready::is_ready;
use peppygen::{NodeRunner, Parameters, Result};
use peppylib::datastore::{self, Encoding};
use srs_model::nalgebra::Isometry3;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;
use tokio::sync::{oneshot, watch};
use tracing::{error, info, warn};

/// `arm_id` values (0 = left, 1 = right). The chain base link and joint limits come
/// from the embedded description keyed by [`Side`] (the command loop itself is
/// scoped by the joint_link pairing, no id needed).
const ARM_ID_LEFT: u8 = 0;
const ARM_ID_RIGHT: u8 = 1;

/// Microseconds in a second: the resolution every loop period here is
/// expressed in.
const MICROS_PER_SECOND: u32 = 1_000_000;

/// Highest loop rate with a non-zero microsecond period, which is one tick of
/// that resolution.
const MAX_RATE_HZ: u32 = MICROS_PER_SECOND;

/// Ceiling on the per-cycle CAN read deadline; see its use for the rationale.
const MAX_RECV_TIMEOUT_US: u32 = 100_000;

/// The arm side for the given `arm_id`, refused on an unknown value so a
/// misconfigured run fails at startup with the value in the message.
fn side_for(arm_id: u8) -> std::result::Result<Side, String> {
    match arm_id {
        ARM_ID_LEFT => Ok(Side::Left),
        ARM_ID_RIGHT => Ok(Side::Right),
        other => Err(format!(
            "arm_id must be {ARM_ID_LEFT} (left) or {ARM_ID_RIGHT} (right), got {other}"
        )),
    }
}

/// The tick period for a whole-hertz rate, refused at startup outside the
/// range that yields a non-zero microsecond period, so a bad rate fails before
/// the motors are energised and with its own name in the message. A rate that
/// does not divide a second evenly truncates to the microsecond below.
fn period_from_hz(rate_hz: u32, name: &str) -> std::result::Result<Duration, String> {
    if !(1..=MAX_RATE_HZ).contains(&rate_hz) {
        return Err(format!(
            "{name} must be in 1..={MAX_RATE_HZ} Hz, got {rate_hz}"
        ));
    }
    Ok(Duration::from_micros(u64::from(
        MICROS_PER_SECOND / rate_hz,
    )))
}

// Sleep durations chosen to match ROS2 enactic/openarm_ros2 v10_simple_hardware behaviour.
const POST_ENABLE_SLEEP: Duration = Duration::from_millis(100);
const BRINGUP_RECV_US: u32 = 500;
const ENABLE_FD: bool = true;
const DATASTORE_TIMEOUT: Duration = Duration::from_secs(3);
/// Tighter bound for shutdown lock removal so motor disable + lock removal stays
/// inside the default 5 s shutdown grace window.
const LOCK_REMOVE_TIMEOUT: Duration = Duration::from_secs(1);
/// Bound on the shutdown hook that awaits the health task's final flush.
/// Hooks share one grace window and this one runs before the motor-disable
/// hook, so an unbounded wait on a stalled publish would hold the motors
/// energised until the force-kill deadline. Sized well inside even the
/// minimum 1 s grace window so the disable hook keeps most of the budget.
const HEALTH_FLUSH_TIMEOUT: Duration = Duration::from_millis(300);

/// Adapts any failure this node can describe into the runtime error type, so
/// it returns through the runtime's error path instead of panicking. That path
/// runs the shutdown hooks, and once the motors are open a panic would skip
/// them and leave them energised with the instance lock held; before then it is
/// what makes the daemon record a named failure rather than a backtrace.
/// Repeated per node because peppygen is generated per node; no shared crate
/// can name its Error type.
fn node_err(e: impl Into<Box<dyn std::error::Error + Send + Sync>>) -> peppygen::Error {
    peppygen::Error::Io(std::io::Error::other(e))
}

/// True once the control loop has latched a hard CAN fault; read by `main`
/// after the runtime returns (the `control` module stays private to this
/// crate).
pub fn hard_fault_latched() -> bool {
    control::HARD_FAULT.load(Ordering::SeqCst)
}

pub async fn setup(params: Parameters, node_runner: Arc<NodeRunner>) -> Result<()> {
    // Pairing timestamps read the daemon-resolved clock (sim time under a
    // simulated clock), so state consumers age samples on one timeline.
    peppygen::clock::init(&node_runner).await?;

    let arm_id = params.arm_id;
    let can_interface = params.can_interface.clone();

    let cycle_period =
        period_from_hz(params.control_rate_hz, "control_rate_hz").map_err(node_err)?;
    let state_period = period_from_hz(params.state_rate_hz, "state_rate_hz").map_err(node_err)?;
    // Bounded above so a config typo cannot park recv_all in a long ppoll
    // while it holds the CAN mutex the shutdown hooks need: 100 ms keeps
    // the whole hook sequence inside even the minimum 1 s grace window,
    // and real configs run around 1 ms. Bounded below because the driver
    // refuses a zero wait on its other receive passes, and a receive that
    // never waits turns every quiet tick into a silence pass.
    if !(1..=MAX_RECV_TIMEOUT_US).contains(&params.recv_timeout_us) {
        return Err(node_err(format!(
            "recv_timeout_us must be in 1..={MAX_RECV_TIMEOUT_US} (100 ms), got {}",
            params.recv_timeout_us
        )));
    }

    let side = side_for(arm_id).map_err(node_err)?;

    // Which OpenArm generation this arm drives; selects the embedded description.
    let hardware_version: HardwareVersion = params.hardware_version.parse().map_err(node_err)?;

    // The chain base link this side's SRS model is walked out from: a fact of the
    // generation's URDF, resolved from the description rather than configured, so a
    // v2 arm can't be launched with a v1 base-link name.
    let base_link = hardware_version.base_link(side);

    // Build the srs_model arm from this generation's embedded OpenArm description:
    // forward kinematics for the in-process gravity/Coriolis feedforward, plus joint
    // limits off the same parsed chain. The elbow singularity margin is a control
    // policy the description exports per generation; apply it so limits() carries it.
    // A non-SRS or short chain from base_link errors here.
    let model = srs_model::Arm::from_urdf(hardware_version.urdf(), base_link)
        .map(|arm| {
            arm.with_lower_floor(
                hardware_version.elbow_joint_index(),
                hardware_version.elbow_singularity_floor_rad(),
            )
        })
        .map_err(|e| {
            node_err(format!(
                "build {hardware_version} arm model from base '{base_link}': {e}"
            ))
        })?;
    info!("model loaded ({hardware_version}, base '{base_link}')");

    // Gravity acts along world -Z, so it is only correct if the URDF carries the
    // mount tree above base_link to orient that frame. We do not force one (a
    // base-rooted URDF legitimately evaluates gravity in the base frame), so log
    // which frame is in play: identity mount means base_link is the URDF root.
    if model.base_from_world() == Isometry3::identity() {
        warn!(
            "no world->base mount tree above '{base_link}': gravity/Coriolis evaluated in \
             the base frame (correct only if base_link's frame is gravity-aligned)"
        );
    } else {
        info!("mount tree resolved: gravity/Coriolis evaluated in the world frame");
    }

    let kp = [
        params.kp1, params.kp2, params.kp3, params.kp4, params.kp5, params.kp6, params.kp7,
    ];
    let kd = [
        params.kd1, params.kd2, params.kd3, params.kd4, params.kd5, params.kd6, params.kd7,
    ];
    info!(
        "config: arm_id={arm_id} ({side:?}) rate={}Hz recv_timeout={}us",
        params.control_rate_hz, params.recv_timeout_us
    );
    info!("config: kp={kp:?} kd={kd:?}");

    // Instance lock: refuse to start if another instance with the same arm_id is
    // running. Held in the core-node datastore (released from the on_shutdown
    // hook below), so a lock leaked by a hard crash clears with the stack
    // instead of lingering like a /tmp file. get-then-store is not atomic; two
    // simultaneous starts can race (single-writer in practice).
    let lock_key = format!("openarm_arm_{arm_id}_instance_lock");
    if let Some(held) = datastore::get(&node_runner, lock_key.as_str(), DATASTORE_TIMEOUT).await? {
        return Err(node_err(format!(
            "instance lock {lock_key} held by {}",
            held.last_modified_by
        )));
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
            if let Err(e) = datastore::remove(&runner, lock_key.as_str(), LOCK_REMOVE_TIMEOUT).await
            {
                warn!("failed to remove lock {lock_key}: {e}");
            }
        });
    }

    // Hardware bringup: sequence mirrors ROS2 v10_simple_hardware on_init/on_activate.
    // Arm motor lineup + CAN addressing are identical across generations; open()
    // registers them.
    info!("opening CAN interface {can_interface} (FD={ENABLE_FD})");
    let arm = Arc::new(Mutex::new(
        ArmCan::open(&can_interface, ENABLE_FD).map_err(node_err)?,
    ));

    // Motor-disable hook, registered before the motors are ever enabled
    // so every stop path disables them, including a cancellation that
    // drops this setup future part way through bring-up. The control
    // loop's own disable on a clean stop runs first and this is then a
    // no-op on already-disabled motors.
    {
        let arm = arm.clone();
        node_runner.on_shutdown(async move {
            let mut a = arm.lock().unwrap_or_else(|e| e.into_inner());
            if let Err(e) = a.disable_all() {
                error!("disable motors: {e}");
            }
        });
    }

    // Registers first: the reads are the slowest part of bring-up and ask
    // the motors nothing but their own configuration, so doing them while
    // the arm is still limp keeps it unenergised until the moment the
    // control loop is ready to hold it. The CAN calls block, so the whole
    // mutex-guarded sequence runs on a blocking thread rather than
    // starving this worker.
    let ratings = {
        let arm = arm.clone();
        let token = node_runner.cancellation_token().clone();
        tokio::task::spawn_blocking(move || -> std::result::Result<_, String> {
            let mut a = arm.lock().unwrap_or_else(|e| e.into_inner());
            // A cancellation that drops the setup future leaves this
            // queued task to run detached, and spawn_blocking cannot be
            // aborted. Cancellation precedes the shutdown hooks and the
            // disable hook takes this same lock, so checking under the
            // lock closes both orders: a post-shutdown start bails here
            // before enabling, and a shutdown during bring-up disables
            // after the enable completes.
            if token.is_cancelled() {
                return Err("cancelled before bring-up".to_string());
            }
            let ratings =
                health::resolve_ratings(&mut a, BRINGUP_RECV_US).map_err(|e| e.to_string())?;
            bring_up(&mut a).map_err(|e| e.to_string())?;
            Ok(ratings)
        })
        .await
        .map_err(|e| peppygen::Error::Io(std::io::Error::other(e)))?
        .map_err(|e| peppygen::Error::Io(std::io::Error::other(e)))?
    };
    info!("arm ready (every motor confirmed enabled)");

    let cfg = ControlConfig {
        kp,
        kd,
        cycle_period,
        recv_timeout_us: params.recv_timeout_us,
        limits: model.limits(),
        ratings,
    };

    // is_ready service: false until bringup and control wiring complete, then
    // true. The real robot_initializer polls this (component_ready) to
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
                        ready.load(Ordering::SeqCst) && !control::HARD_FAULT.load(Ordering::SeqCst),
                    ))
                })
                .await
                {
                    error!("is_ready: {e}");
                }
            }
        });
    }

    // Stream plumbing: the listener keeps the latest governed setpoint for
    // the control loop, and the publishers emit the measured joint state
    // and the per-motor health at their own rates (the backbone consumes
    // the state; any motor_health consumer the launcher wires gets the
    // health).
    let (governed_tx, governed_rx) = watch::channel(None);
    let (measured_tx, measured_rx) = watch::channel(None);
    let (health_tx, health_rx) = watch::channel(None);
    let listener = tokio::spawn(stream::run_governed_setpoint_listener(
        node_runner.clone(),
        governed_tx,
    ));
    let publisher = tokio::spawn(stream::run_state_publisher(
        node_runner.clone(),
        state_period,
        measured_rx,
    ));
    // Names this arm's motors on the alert wire: "left arm" extends to
    // "left arm j2" per joint.
    let alert_source = match side {
        Side::Left => "left arm",
        Side::Right => "right arm",
    }
    .to_string();
    // The final flush round after cancellation is only awaited if a
    // shutdown hook waits for it: the runtime awaits hooks, not plain
    // tasks racing the token. Dropping the sender (return or panic)
    // completes the receiver, so the hook cannot hang on a dead task.
    let (health_done_tx, health_done_rx) = oneshot::channel::<()>();
    let health_publisher = tokio::spawn({
        let runner = node_runner.clone();
        let token = node_runner.cancellation_token().clone();
        async move {
            let _done = health_done_tx;
            health::run_publisher(runner, alert_source, health_rx, token).await;
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
    // Cancel the node the moment any stream task stops: a dead listener
    // or publisher would otherwise hold the arm silently while is_ready
    // stays true (same supervision as the sim followers). A stop before
    // the token was cancelled is the fault, not a reaction to shutdown,
    // so record it and exit as failed.
    {
        let token = node_runner.cancellation_token().clone();
        tokio::spawn(async move {
            tokio::select! {
                _ = listener => {}
                _ = publisher => {}
                _ = health_publisher => {}
            }
            if !token.is_cancelled() {
                error!("stream task exited unexpectedly");
                control::HARD_FAULT.store(true, Ordering::SeqCst);
            }
            token.cancel();
        });
    }
    let wiring = stream::StreamWiring {
        governed: governed_rx,
        measured: measured_tx,
        health: health_tx,
    };

    // Single control task (the only motor writer): follows the governed
    // setpoint with in-process feedforward and a final limit clamp, and on
    // shutdown disables the motors.
    let (control_started_tx, control_started_rx) = oneshot::channel::<()>();
    control::spawn(
        &node_runner,
        arm.clone(),
        cfg,
        model,
        wiring,
        shutdown_tx,
        control_started_tx,
    );
    // The ack arrives once the control loop is actually running; a task
    // that dies first drops the sender and the error return runs the
    // disable hooks. Reporting ready any earlier would let the robot
    // gate open on a spawned-but-dead controller.
    control_started_rx.await.map_err(|_| {
        peppygen::Error::Io(std::io::Error::other("the control loop never started"))
    })?;
    ready.store(true, Ordering::SeqCst);

    Ok(())
}

/// Enable the motors, verifying each acknowledges torque authority and
/// retrying stragglers; readiness is refused naming the motors that never
/// confirm. Blocking, so a stop cannot land mid-sequence with the motors
/// half enabled.
fn bring_up(arm: &mut ArmCan) -> std::result::Result<(), openarm_can::EnableFailure> {
    arm.enable_and_confirm(
        openarm_can::ENABLE_ATTEMPTS,
        POST_ENABLE_SLEEP,
        BRINGUP_RECV_US,
    )?;
    // One whole state pass so `get_state()` is real before the control loop's
    // first tick; the confirmation's own decode may be several hundred
    // milliseconds old by now. A pass that stopped at the first gap between
    // replies would leave the later joints reading as never-heard-from.
    arm.refresh_state(BRINGUP_RECV_US)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_rate_outside_the_expressible_range_is_named_not_panicked() {
        for rate_hz in [0, MAX_RATE_HZ + 1, u32::MAX] {
            let refusal = period_from_hz(rate_hz, "control_rate_hz")
                .expect_err("a rate outside the expressible range must be refused");
            assert!(
                refusal.contains("control_rate_hz"),
                "the refusal must name the parameter, got {refusal}"
            );
            assert!(
                refusal.ends_with(&format!("got {rate_hz}")),
                "the refusal must name the offending value, got {refusal}"
            );
        }
    }

    #[test]
    fn a_rate_in_range_yields_its_period() {
        assert_eq!(
            period_from_hz(100, "control_rate_hz"),
            Ok(Duration::from_millis(10))
        );
        assert_eq!(
            period_from_hz(1, "state_rate_hz"),
            Ok(Duration::from_secs(1))
        );
        assert_eq!(
            period_from_hz(MAX_RATE_HZ, "control_rate_hz"),
            Ok(Duration::from_micros(1))
        );
    }

    #[test]
    fn an_unknown_arm_id_is_named_not_panicked() {
        assert_eq!(side_for(ARM_ID_LEFT), Ok(Side::Left));
        assert_eq!(side_for(ARM_ID_RIGHT), Ok(Side::Right));
        let refusal = side_for(7).expect_err("an id outside the convention must be refused");
        assert!(refusal.ends_with("got 7"), "got {refusal}");
    }
}
