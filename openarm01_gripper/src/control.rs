use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use peppygen::NodeRunner;
use peppygen::exposed_actions::openarm01_gripper::v1::move_gripper;
use tracing::{error, warn};

use openarm_can::{GripperCan, v10};

// V10 gripper gains — hardcoded, not configurable in ROS2 reference either.
pub const KP: f64 = 5.0;
pub const KD: f64 = 0.1;

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
}

/// Spawn-and-loop entry for the move_gripper action. Mirrors the pattern in the arm node:
/// accept goal → run control loop → complete the goal. Re-enters afterwards. A goal arriving
/// mid-motion is rejected (single-flight); the file lock in main.rs enforces single-instance.
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

    loop {
        let ctx = match handle
            .handle_goal_next_request(|req| {
                // Reject targets outside the gripper's physical travel.
                if !position_in_range(req.data.position) {
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
            })
            .await
        {
            Ok(Some(ctx)) => ctx,
            Ok(None) => break, // action closed (node shutting down)
            Err(e) => {
                error!("move_gripper goal: {e}");
                continue;
            }
        };

        let goal = AcceptedGoal {
            target_position_m: ctx.request().data.position,
            feedback_period: feedback_period(ctx.request().data.feedback_frequency),
        };

        let gripper = Arc::clone(&gripper);
        let cfg = cfg.clone();
        let busy = Arc::clone(&busy);
        tokio::spawn(async move {
            let result = run_control_loop(&gripper, &ctx, &cfg, goal).await;
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
) -> MotionResult {
    // ROS2 reference (v10_simple_hardware write()): mit_control_all with target, then recv_all.
    // No trajectory — the soft PD gains (KP=5.0, KD=0.1) smooth the motion naturally.
    // The motor speaks radians; we keep all user-facing units in meters and convert at the FFI.
    let target_motor_rad = meters_to_motor_rad(goal.target_position_m);
    let start = Instant::now();
    let mut last_feedback = Instant::now();
    let mut feedback_failures: u32 = 0;

    loop {
        let cycle_start = Instant::now();

        let motor_rad = {
            let mut g = gripper.lock().unwrap_or_else(|e| e.into_inner());
            g.mit_control(KP, KD, target_motor_rad, 0.0, 0.0);
            g.refresh_all();
            g.recv_all(cfg.recv_timeout_us);
            g.get_state().position
        };
        let position_m = motor_rad_to_meters(motor_rad);

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

        let cycle_elapsed = cycle_start.elapsed();
        if cycle_elapsed < cfg.cycle_period {
            tokio::time::sleep(cfg.cycle_period - cycle_elapsed).await;
        } else if cycle_elapsed > cfg.cycle_period.mul_f64(1.2) {
            warn!(
                "control loop overrun: {:.1}ms (budget {:.1}ms)",
                cycle_elapsed.as_secs_f64() * 1000.0,
                cfg.cycle_period.as_secs_f64() * 1000.0,
            );
        }
    }
}

/// Linear joint-meter → motor-radian mapping. Closed=0m↔0rad, open=GRIPPER_OPEN_M↔GRIPPER_OPEN_RAD.
fn meters_to_motor_rad(pos_m: f64) -> f64 {
    (pos_m / v10::GRIPPER_OPEN_M) * v10::GRIPPER_OPEN_RAD
}

/// Inverse of `meters_to_motor_rad`: motor angle back to user-facing joint position in meters.
fn motor_rad_to_meters(motor_rad: f64) -> f64 {
    (motor_rad / v10::GRIPPER_OPEN_RAD) * v10::GRIPPER_OPEN_M
}

/// True if `pos_m` is within the gripper's physical travel range.
fn position_in_range(pos_m: f64) -> bool {
    (0.0..=v10::GRIPPER_OPEN_M).contains(&pos_m)
}

/// Convert a feedback frequency in Hz to a Duration. Floors at 1 Hz to avoid divide-by-zero.
fn feedback_period(freq_hz: u32) -> Duration {
    Duration::from_micros(1_000_000 / freq_hz.max(1) as u64)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn meters_to_motor_rad_is_linear_between_endpoints() {
        // Closed and open ends define the line; midpoint should land exactly halfway.
        // Catches sign flips and scale errors.
        assert!((meters_to_motor_rad(0.0) - 0.0).abs() < 1e-12);
        assert!((meters_to_motor_rad(v10::GRIPPER_OPEN_M) - v10::GRIPPER_OPEN_RAD).abs() < 1e-12);
        let mid = meters_to_motor_rad(v10::GRIPPER_OPEN_M / 2.0);
        assert!((mid - v10::GRIPPER_OPEN_RAD / 2.0).abs() < 1e-12);
    }

    #[test]
    fn motor_rad_and_meters_round_trip() {
        // round-trip catches inverse mismatch (wrong constant in numerator/denominator).
        for pos_m in [0.0, 0.01, v10::GRIPPER_OPEN_M / 3.0, v10::GRIPPER_OPEN_M] {
            let back = motor_rad_to_meters(meters_to_motor_rad(pos_m));
            assert!((back - pos_m).abs() < 1e-12, "round-trip failed for {pos_m}");
        }
    }

    #[test]
    fn position_in_range_is_inclusive_at_both_ends() {
        assert!(position_in_range(0.0));
        assert!(position_in_range(v10::GRIPPER_OPEN_M));
        assert!(!position_in_range(-1e-9));
        assert!(!position_in_range(v10::GRIPPER_OPEN_M + 1e-9));
    }

    #[test]
    fn feedback_period_floors_zero_freq() {
        // 0 Hz would otherwise divide by zero; we floor at 1 Hz.
        assert_eq!(feedback_period(0), Duration::from_secs(1));
    }
}
