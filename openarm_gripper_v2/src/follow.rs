// Ambient following of a streamed gripper opening fraction: drive the motor
// toward the latest command; with none yet, hold (the motor's position mode
// keeps its last setpoint, so we simply do not re-command). Either way the loop
// refreshes the motor state every tick, so the always-on state publisher serves
// a live reading rather than one frozen at bring-up until the first command
// (the arm control loop reads state every tick the same way). The opening is
// commanded directly; the motor's position mode eases to it.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use openarm_can::{GripperCan, PosForce};
use peppylib::runtime::CancellationToken;
use tokio::sync::watch;
use tokio::time::MissedTickBehavior;
use tracing::{error, info};

use crate::command_stream::GripperCommand;
use crate::geometry::{self, Geometry};

#[derive(Clone)]
pub struct ControlConfig {
    pub cycle_period: Duration,
    pub recv_timeout_us: u32,
    /// This instance's signed opening-fraction to motor-radian mapping.
    pub geometry: Geometry,
    /// POS_FORCE absolute speed limit (rad/s at the motor).
    pub speed_rad_s: f64,
    /// POS_FORCE torque-current ceiling (per-unit, 0..1): the largest grip-force
    /// cap; a streamed max_effort can only lower a tick's cap beneath it.
    pub force_limit_pu: f64,
}

pub async fn run(
    gripper: Arc<Mutex<GripperCan<PosForce>>>,
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

        let command = *cmd.borrow();

        // unwrap_or_else: drive even if the mutex was poisoned by a panic
        // elsewhere, so a transient fault doesn't strand the follow loop.
        let mut g = gripper.lock().unwrap_or_else(|e| e.into_inner());
        // Command only when there is a target; refresh state every tick either way.
        let tick = (|| {
            if let Some(command) = command {
                let target_motor_rad = cfg
                    .geometry
                    .fraction_to_motor_rad(command.opening.clamp(0.0, 1.0));
                let torque_pu =
                    geometry::effort_to_torque_pu(command.max_effort, cfg.force_limit_pu);
                g.set_position(target_motor_rad, cfg.speed_rad_s, torque_pu)?;
            }
            g.refresh_all()?;
            g.recv_all(cfg.recv_timeout_us)
        })();
        match tick {
            Ok(()) => throttle.success(),
            Err(e) => throttle.failure(&e),
        }
    }
}

/// One log line per failure burst instead of several per tick at the loop
/// rate: the first failure logs, a long outage re-logs every 100th tick, and
/// recovery logs once.
struct CanErrorThrottle {
    consecutive: u64,
}

impl CanErrorThrottle {
    const REPEAT_EVERY: u64 = 100;

    fn new() -> Self {
        Self { consecutive: 0 }
    }

    fn failure(&mut self, e: &openarm_can::CanError) {
        if self.consecutive.is_multiple_of(Self::REPEAT_EVERY) {
            error!(
                "gripper CAN tick failed ({} consecutive): {e}",
                self.consecutive + 1
            );
        }
        self.consecutive += 1;
    }

    fn success(&mut self) {
        if self.consecutive > 0 {
            info!(
                "gripper CAN recovered after {} failed ticks",
                self.consecutive
            );
        }
        self.consecutive = 0;
    }
}
