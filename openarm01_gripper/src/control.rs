use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use peppygen::NodeRunner;
use peppygen::exposed_actions::openarm01_gripper::v1::move_gripper;
use peppylib::runtime::CancellationToken;
use tracing::{error, warn};

use openarm_can::GripperCan;

use crate::geometry::{self, GRIPPER_LIMITS_M};

// V10 gripper gains, matching the openarm teleop follower (config/follower.yaml
// gripper entry). Hardcoded, not configurable in the ROS2 reference either.
pub const KP: f64 = 16.0;
pub const KD: f64 = 0.2;

struct AcceptedGoal {
    target_position_m: f64,
    feedback_period: Duration,
}

#[derive(Clone)]
pub struct ControlConfig {
    pub cycle_period: Duration,
    pub recv_timeout_us: i32,
    pub position_tolerance_m: f64,
    pub motion_timeout: Duration,
}

struct MotionResult {
    success: bool,
    message: String,
    final_position_m: f64,
    action_time: f64,
}

impl MotionResult {
    fn reached(position_m: f64, action_time: f64) -> Self {
        Self { success: true, message: "reached".into(), final_position_m: position_m, action_time }
    }

    fn timed_out(position_m: f64, action_time: f64) -> Self {
        Self { success: false, message: "timeout".into(), final_position_m: position_m, action_time }
    }

    fn cancelled(position_m: f64, action_time: f64) -> Self {
        Self { success: false, message: "cancelled".into(), final_position_m: position_m, action_time }
    }
}

/// Spawn-and-loop entry for the move_gripper action. Mirrors the pattern in the arm node:
/// accept goal → run control loop → complete the goal. Re-enters afterwards. A goal arriving
/// mid-motion is rejected (single-flight); the instance lock in main.rs enforces single-instance.
pub async fn run_move_gripper(
    runner: Arc<NodeRunner>,
    gripper: Arc<Mutex<GripperCan>>,
    cfg: ControlConfig,
) {
    let mut handle = move_gripper::ActionHandle::expose(&runner)
        .await
        .expect("expose move_gripper");

    // Single-flight gate: reject a new goal while one is still executing. The
    // motion runs in a spawned task so the loop keeps listening (and rejecting)
    // rather than silently queueing a goal that would run stale once the
    // current one finishes.
    let busy = Arc::new(AtomicBool::new(false));

    let token = runner.cancellation_token().clone();

    loop {
        let accept = handle.handle_goal_next_request(|req| {
            // Reject targets outside the gripper's physical travel (also
            // rejects NaN/inf, which Limit::contains treats as out of range).
            if !GRIPPER_LIMITS_M.contains(req.data.position) {
                return Ok(move_gripper::GoalResponse::reject("target position out of range"));
            }
            // Atomically claim the slot; reject if a motion already holds it.
            if busy
                .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
                .is_err()
            {
                Ok(move_gripper::GoalResponse::reject(
                    "gripper is already executing a motion",
                ))
            } else {
                Ok(move_gripper::GoalResponse::accept())
            }
        });
        let ctx = tokio::select! {
            _ = token.cancelled() => break, // node shutting down
            res = accept => match res {
                Ok(Some(ctx)) => ctx,
                Ok(None) => break, // action closed (node shutting down)
                Err(e) => {
                    error!("move_gripper goal: {e}");
                    continue;
                }
            },
        };

        let goal = AcceptedGoal {
            target_position_m: ctx.request().data.position,
            feedback_period: feedback_period(ctx.request().data.feedback_frequency),
        };

        let gripper = Arc::clone(&gripper);
        let cfg = cfg.clone();
        let busy = Arc::clone(&busy);
        let token = token.clone();
        tokio::spawn(async move {
            let result = run_control_loop(&gripper, &ctx, &cfg, goal, &token).await;
            if let Err(e) = ctx
                .complete(
                    result.success,
                    result.message,
                    vec![result.final_position_m],
                    result.action_time,
                )
                .await
            {
                error!("move_gripper complete: {e}");
            }
            busy.store(false, Ordering::Release);
        });
    }
}

async fn run_control_loop(
    gripper: &Arc<Mutex<GripperCan>>,
    ctx: &move_gripper::GoalContext,
    cfg: &ControlConfig,
    goal: AcceptedGoal,
    token: &CancellationToken,
) -> MotionResult {
    // ROS2 reference (v10_simple_hardware write()): mit_control_all with target, then recv_all.
    // No trajectory: a single MIT setpoint held by the motor PD gains; the gripper's
    // short travel needs no profiling.
    // The motor speaks radians; we keep all user-facing units in meters and convert at the FFI.
    let target_motor_rad = geometry::meters_to_motor_rad(goal.target_position_m);
    let start = Instant::now();
    let mut last_feedback = Instant::now();
    let mut feedback_failures: u32 = 0;
    // Absolute timeline the loop paces against, so per-cycle sleep overshoot does
    // not accumulate (matches the arm loop and the openarm teleop reference).
    let mut next_tick = tokio::time::Instant::now();

    loop {
        let motor_rad = {
            let mut g = gripper.lock().unwrap_or_else(|e| e.into_inner());
            // Checked under the lock: the disable hook (main.rs) only runs after
            // the token cancels and takes this same lock to disable_all(), so a
            // tick that observes the cancel here never re-energises the motor
            // with one last MIT frame after it has been disabled. The biased
            // select at the bottom of the loop then fails the goal as cancelled.
            if !token.is_cancelled() {
                g.mit_control(KP, KD, target_motor_rad, 0.0, 0.0);
            }
            g.refresh_all();
            g.recv_all(cfg.recv_timeout_us);
            g.get_state().position
        };
        let position_m = geometry::motor_rad_to_meters(motor_rad);

        let elapsed = start.elapsed();
        let elapsed_secs = elapsed.as_secs_f64();
        let done = (position_m - goal.target_position_m).abs() < cfg.position_tolerance_m;

        if last_feedback.elapsed() >= goal.feedback_period {
            // Feedback is best-effort (QoS Standard); a drop must not stall the
            // loop. Warn once per motion if it starts failing, then stay quiet.
            if let Err(e) = ctx.publish_feedback(vec![position_m], elapsed_secs).await {
                feedback_failures += 1;
                if feedback_failures == 1 {
                    warn!("move_gripper feedback publish failing, suppressing repeats: {e}");
                }
            }
            last_feedback = Instant::now();
        }

        if done {
            return MotionResult::reached(position_m, elapsed_secs);
        }
        if elapsed > cfg.motion_timeout {
            return MotionResult::timed_out(position_m, elapsed_secs);
        }

        // Biased so a cancelled token always wins over an overrun (already-due)
        // tick: on shutdown stop commanding the motor and fail the goal; the
        // on_shutdown hook in main.rs disables the hardware.
        tokio::select! {
            biased;
            _ = token.cancelled() => return MotionResult::cancelled(position_m, elapsed_secs),
            _ = pace_to_deadline(&mut next_tick, cfg.cycle_period) => {}
        }
    }
}

/// Pace the loop to an absolute timeline: sleep until `next_tick`, which advances
/// by exactly one `period` each cycle, so the overshoot every `tokio::time::sleep`
/// incurs is corrected on the next cycle instead of accumulating. On an overrun
/// the deadline is already past: re-anchor to now and skip the sleep.
async fn pace_to_deadline(next_tick: &mut tokio::time::Instant, period: Duration) {
    *next_tick += period;
    let now = tokio::time::Instant::now();
    if *next_tick <= now {
        *next_tick = now;
    } else {
        tokio::time::sleep_until(*next_tick).await;
    }
}

/// Convert a feedback frequency in Hz to a Duration. Floors at 1 Hz to avoid divide-by-zero.
fn feedback_period(freq_hz: u32) -> Duration {
    Duration::from_micros(1_000_000 / freq_hz.max(1) as u64)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn feedback_period_floors_zero_freq() {
        // 0 Hz would otherwise divide by zero; we floor at 1 Hz.
        assert_eq!(feedback_period(0), Duration::from_secs(1));
    }
}
