// Ambient following of a streamed gripper opening fraction: drive the motor
// toward the latest command; with none yet, hold (the motor's PD keeps its last
// setpoint, so we simply do not re-command). Either way the loop refreshes the
// motor state every tick, so the always-on state publisher serves a live
// reading rather than one frozen at bring-up until the first command (the arm
// control loop reads state every tick the same way). The opening is commanded
// directly; the motor's PD eases to it.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use openarm_can::{CanErrorThrottle, GripperCan, Mit};
use peppylib::runtime::CancellationToken;
use tokio::sync::watch;
use tokio::time::MissedTickBehavior;
use tracing::error;

use crate::command_stream::GripperCommand;
use crate::geometry;

// V10 gripper gains, matching the openarm teleop follower (config/follower.yaml
// gripper entry). Hardcoded, not configurable in the ROS2 reference either.
pub const KP: f64 = 16.0;
pub const KD: f64 = 0.2;

/// Set when the loop stops on a hard fault, so main can exit non-zero after
/// the shutdown hooks have run and the daemon records the instance as failed
/// rather than finished.
pub static HARD_FAULT: AtomicBool = AtomicBool::new(false);

const CONTEXT: &str = "gripper follow";

/// Consecutive failed ticks before the loop declares a hard fault and stops
/// the node: ~2 s at the configured 100 Hz, long enough to ride out bus-off
/// recovery, short enough that a dead bus cannot masquerade as healthy.
const FAULT_TICKS: u64 = 200;

#[derive(Clone)]
pub struct ControlConfig {
    pub cycle_period: Duration,
    pub recv_timeout_us: u32,
}

pub async fn run(
    gripper: Arc<Mutex<GripperCan<Mit>>>,
    cmd: watch::Receiver<Option<GripperCommand>>,
    cfg: ControlConfig,
    token: CancellationToken,
) {
    let mut ticker = tokio::time::interval(cfg.cycle_period);
    ticker.set_missed_tick_behavior(MissedTickBehavior::Delay);
    let mut throttle = CanErrorThrottle::new();

    loop {
        tokio::select! {
            _ = token.cancelled() => return,
            _ = ticker.tick() => {}
        }

        let opening = cmd.borrow().as_ref().map(|c| c.opening);

        // unwrap_or_else: drive even if the mutex was poisoned by a panic
        // elsewhere, so a transient fault doesn't strand the follow loop.
        let mut g = gripper.lock().unwrap_or_else(|e| e.into_inner());
        // Command only when there is a target; refresh state every tick either way.
        let tick = (|| {
            if let Some(opening) = opening {
                let target_motor_rad = geometry::fraction_to_motor_rad(opening.clamp(0.0, 1.0));
                g.mit_control(KP, KD, target_motor_rad, 0.0, 0.0)?;
            }
            g.refresh_all()?;
            g.recv_all(cfg.recv_timeout_us)
        })();
        match tick {
            Ok(()) => throttle.success(CONTEXT),
            Err(e) => throttle.failure(CONTEXT, &e),
        }
        // A burst this long is a hard fault, not a hiccup: stop the node so
        // the shutdown hooks disable the motor and the stack shows a dead
        // instance instead of a ready node publishing a frozen state.
        if throttle.consecutive() >= FAULT_TICKS {
            error!("persistent CAN fault ({FAULT_TICKS} consecutive failed ticks): stopping node");
            HARD_FAULT.store(true, Ordering::SeqCst);
            token.cancel();
            return;
        }
    }
}
