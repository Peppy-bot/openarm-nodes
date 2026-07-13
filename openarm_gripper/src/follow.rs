// Ambient following of a streamed gripper opening: drive the motor toward the
// latest command; until the first command arrives, hold by issuing no CAN
// traffic so the motor's PD keeps its last setpoint. The opening is commanded
// directly; the motor's PD eases to it.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use openarm_can::GripperCan;
use peppylib::runtime::CancellationToken;
use tokio::sync::watch;
use tokio::time::MissedTickBehavior;

use crate::command_stream::GripperCommand;
use crate::geometry::{self, GRIPPER_LIMITS_M};

// V10 gripper gains, matching the openarm teleop follower (config/follower.yaml
// gripper entry). Hardcoded, not configurable in the ROS2 reference either.
pub const KP: f64 = 16.0;
pub const KD: f64 = 0.2;

#[derive(Clone)]
pub struct ControlConfig {
    pub cycle_period: Duration,
    pub recv_timeout_us: i32,
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

        // Follow the latest command; until one arrives, hold (issue no CAN
        // traffic, the motor's PD keeps its last setpoint).
        let position = cmd.borrow().as_ref().map(|c| c.position);
        let Some(position) = position else {
            continue;
        };
        let target_m = position.clamp(GRIPPER_LIMITS_M.lo, GRIPPER_LIMITS_M.hi);
        let target_motor_rad = geometry::meters_to_motor_rad(target_m);

        // unwrap_or_else: drive even if the mutex was poisoned by a panic
        // elsewhere, so a transient fault doesn't strand the follow loop.
        let mut g = gripper.lock().unwrap_or_else(|e| e.into_inner());
        g.mit_control(KP, KD, target_motor_rad, 0.0, 0.0);
        g.refresh_all();
        g.recv_all(cfg.recv_timeout_us);
    }
}
