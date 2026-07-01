// Ambient following of a streamed gripper opening. While no move is running
// (busy gate clear), drive the motor toward the latest fresh command; when the
// stream goes stale, hold by issuing no CAN traffic so the motor's PD keeps its
// last setpoint. The move action and this loop share the busy gate, so they
// never both drive the single CAN handle. The opening is commanded directly; the
// motor's PD eases to it.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

use openarm_can::GripperCan;
use peppylib::runtime::CancellationToken;
use tokio::sync::watch;
use tokio::time::MissedTickBehavior;

use crate::command_stream::GripperCommand;
use crate::control::{ControlConfig, KD, KP};
use crate::geometry::{self, GRIPPER_LIMITS_M};

pub async fn run(
    gripper: Arc<Mutex<GripperCan>>,
    busy: Arc<AtomicBool>,
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

        // A move owns the gripper: yield so the action stays the sole CAN writer.
        if busy.load(Ordering::Acquire) {
            continue;
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
        g.mit_control(KP, KD, target_motor_rad, 0.0, 0.0);
        g.refresh_all();
        g.recv_all(cfg.recv_timeout_us);
    }
}
