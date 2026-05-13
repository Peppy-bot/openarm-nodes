use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use peppygen::NodeRunner;
use peppygen::exposed_actions::move_gripper;
use tracing::{error, warn};

use openarm_can::{GripperCan, v10};

// V10 gripper gains — hardcoded, not configurable in ROS2 reference either.
pub const KP: f64 = 5.0;
pub const KD: f64 = 0.1;

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

impl Default for MotionResult {
    fn default() -> Self {
        Self { success: false, message: "no result".into(), final_position_m: 0.0, action_time: 0.0 }
    }
}

/// Spawn-and-loop entry for the move_gripper action. Mirrors the pattern in the arm node:
/// accept goal → run control loop → return result. Re-enters after each completed goal.
pub async fn run_move_gripper(
    runner: Arc<NodeRunner>,
    gripper: Arc<Mutex<GripperCan>>,
    cfg: ControlConfig,
) {
    let mut handle = move_gripper::ActionHandle::expose(&runner)
        .await
        .expect("expose move_gripper");

    let pending: Arc<Mutex<Option<AcceptedGoal>>> = Arc::new(Mutex::new(None));

    loop {
        // 1. Wait for a goal request.
        let pending_for_handler = pending.clone();
        if let Err(e) = handle
            .handle_goal_next_request(move |req| {
                let pos_m = req.data.position;
                if !position_in_range(pos_m) {
                    return Ok(move_gripper::GoalResponse::new(false));
                }
                // Reject if a motion is already in progress.
                let mut slot = pending_for_handler.lock().unwrap();
                if slot.is_some() {
                    return Ok(move_gripper::GoalResponse::new(false));
                }
                *slot = Some(AcceptedGoal {
                    target_position_m: pos_m,
                    feedback_period: feedback_period(req.data.feedback_frequency),
                });
                Ok(move_gripper::GoalResponse::new(true))
            })
            .await
        {
            error!("move_gripper goal: {e}");
            continue;
        }

        // 2. If accepted, run the control loop.
        let goal = pending.lock().unwrap().take();
        let result = match goal {
            Some(g) => run_control_loop(&gripper, &handle, &cfg, g).await,
            None => continue, // goal was rejected
        };

        // 3. Return the result.
        let stash: Arc<Mutex<Option<MotionResult>>> = Arc::new(Mutex::new(Some(result)));
        let stash_for_handler = stash.clone();
        if let Err(e) = handle
            .handle_result_next_request(move |_req| {
                let r = stash_for_handler.lock().unwrap().take().unwrap_or_default();
                Ok(move_gripper::ResultResponse::new(
                    r.action_time,
                    vec![r.final_position_m],
                    r.message,
                    r.success,
                ))
            })
            .await
        {
            error!("move_gripper result: {e}");
        }
    }
}

async fn run_control_loop(
    gripper: &Arc<Mutex<GripperCan>>,
    handle: &move_gripper::ActionHandle,
    cfg: &ControlConfig,
    goal: AcceptedGoal,
) -> MotionResult {
    // ROS2 reference (v10_simple_hardware write()): mit_control_all with target, then recv_all.
    // No trajectory — the soft PD gains (KP=5.0, KD=0.1) smooth the motion naturally.
    // The motor speaks radians; we keep all user-facing units in meters and convert at the FFI.
    let target_motor_rad = meters_to_motor_rad(goal.target_position_m);
    let start = Instant::now();
    let mut last_feedback = Instant::now();

    loop {
        let cycle_start = Instant::now();

        let motor_rad = {
            let mut g = gripper.lock().unwrap();
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
            if let Err(e) = handle.emit_feedback(elapsed_secs, vec![position_m]).await {
                warn!("feedback: {e}");
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
