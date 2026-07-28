// Ambient following of a streamed gripper opening fraction: drive the motor
// toward the latest command; with none yet, hold (the motor's PD keeps its last
// setpoint, so we simply do not re-command). Either way the loop refreshes the
// motor state every tick, so the always-on state publisher serves a live
// reading rather than one frozen at bring-up until the first command (the arm
// control loop reads state every tick the same way). The opening is commanded
// directly; the motor's PD eases to it.

use std::sync::atomic::AtomicBool;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use openarm_can::{BusHealth, GripperCan, Mit, v10};
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

/// Bound on how far a commanded motor angle may sit from the measured one
/// before the loop refuses to chase it: the PD saturation gap tau_max / kp,
/// past which a larger error adds no torque and can only mean a target that
/// never walked from here (stale upstream command state). Rate-independent.
const JUMP_LIMIT_RAD: f64 = v10::GRIPPER_MOTOR_TYPE.torque_limit_nm() / KP;

/// Set when the loop stops on a hard fault, so main can exit non-zero after
/// the shutdown hooks have run and the daemon records the instance as failed
/// rather than finished.
pub static HARD_FAULT: AtomicBool = AtomicBool::new(false);

const CONTEXT: &str = "gripper follow";

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
    let mut health = BusHealth::new(CONTEXT, cfg.cycle_period);
    let mut last_jump_warn: Option<std::time::Instant> = None;

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
                // Last line of defense: hold rather than chase a target
                // implausibly far from the measured angle (see JUMP_LIMIT_RAD).
                if (target_motor_rad - g.get_state().position).abs() <= JUMP_LIMIT_RAD {
                    g.mit_control(KP, KD, target_motor_rad, 0.0, 0.0)?;
                } else {
                    warn_jump_throttled(&mut last_jump_warn, target_motor_rad);
                }
            }
            g.refresh_all()?;
            g.recv_all(cfg.recv_timeout_us)
        })();
        // A bus failure costs this tick's frames, not the loop: the jaws hold
        // their last commanded opening and the next tick carries a fresh
        // absolute command. Stopping here would disable the motor.
        match tick {
            Ok(()) => health.succeeded(),
            Err(e) => health.failed(&e),
        }
    }
}

/// One guard line per second, not one per tick.
fn warn_jump_throttled(last: &mut Option<std::time::Instant>, target_rad: f64) {
    let now = std::time::Instant::now();
    if last.is_none_or(|t| now.duration_since(t) >= Duration::from_secs(1)) {
        *last = Some(now);
        error!(
            "setpoint jump guard: commanded {target_rad:.3} rad is beyond the saturation gap \
             from the measured angle; holding"
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn jump_limit_is_the_saturation_gap() {
        // DM4310 tau_max 10 Nm at kp 16 -> 0.625 rad, well over half the
        // 1.05 rad travel: normal tracking never trips it.
        assert!((JUMP_LIMIT_RAD - 10.0 / 16.0).abs() < 1e-12);
    }
}
