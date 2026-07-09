// Ambient following of a streamed gripper opening: drive the motor toward the
// latest fresh command; when the stream goes stale, hold by issuing no CAN
// traffic so the motor keeps its last setpoint. The opening is commanded
// directly; the motor's position mode eases to it.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use openarm_can::GripperCan;
use peppylib::runtime::CancellationToken;
use tokio::sync::watch;
use tokio::time::MissedTickBehavior;

use crate::command_stream::GripperCommand;
use crate::geometry::{self, GRIPPER_LIMITS_M};

#[derive(Clone)]
pub struct ControlConfig {
    pub cycle_period: Duration,
    pub recv_timeout_us: i32,
    /// How long a streamed command stays fresh before the follow loop holds.
    pub stream_timeout: Duration,
    /// POS_FORCE absolute speed limit (rad/s at the motor).
    pub speed_rad_s: f64,
    /// POS_FORCE torque-current limit (per-unit, 0..1): the grip-force cap.
    pub force_limit_pu: f64,
}

pub async fn run(
    gripper: Arc<Mutex<GripperCan>>,
    cmd: watch::Receiver<Option<GripperCommand>>,
    cfg: ControlConfig,
    token: CancellationToken,
) {
    let mut ticker = tokio::time::interval(cfg.cycle_period);
    ticker.set_missed_tick_behavior(MissedTickBehavior::Delay);

    loop {
        tokio::select! {
            _ = token.cancelled() => return,
            _ = ticker.tick() => {}
        }

        // Follow only a command still within the stream timeout; otherwise hold
        // (issue no CAN traffic, the motor's PD keeps its last setpoint).
        let position = {
            let guard = cmd.borrow();
            guard
                .as_ref()
                .filter(|c| c.recv_at.elapsed() <= cfg.stream_timeout)
                .map(|c| c.position)
        };
        let Some(position) = position else {
            continue;
        };
        let target_m = position.clamp(GRIPPER_LIMITS_M.lo, GRIPPER_LIMITS_M.hi);
        let target_motor_rad = geometry::meters_to_motor_rad(target_m);

        // unwrap_or_else: drive even if the mutex was poisoned by a panic
        // elsewhere, so a transient fault doesn't strand the follow loop.
        let mut g = gripper.lock().unwrap_or_else(|e| e.into_inner());
        g.set_position(target_motor_rad, cfg.speed_rad_s, cfg.force_limit_pu);
        g.refresh_all();
        g.recv_all(cfg.recv_timeout_us);
    }
}
