use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use peppygen::NodeRunner;
use peppygen::exposed_actions::openarm01_gripper::v1::move_gripper;
use tracing::error;

use openarm_can::GripperCan;

use crate::geometry::{self, GRIPPER_LIMITS_M};

// V10 gripper gains, matching the openarm teleop follower (config/follower.yaml
// gripper entry). Hardcoded, not configurable in the ROS2 reference either.
pub const KP: f64 = 16.0;
pub const KD: f64 = 0.2;

struct AcceptedGoal {
    target_position_m: f64,
}

#[derive(Clone)]
pub struct ControlConfig {
    pub cycle_period: Duration,
    pub recv_timeout_us: i32,
    pub position_tolerance_m: f64,
    pub motion_timeout: Duration,
    /// How long a streamed command stays fresh before the follow loop holds.
    pub stream_timeout: Duration,
}

struct MotionResult {
    success: bool,
    message: String,
    final_position_m: f64,
    action_time: f64,
}

impl MotionResult {
    fn reached(position_m: f64, action_time: f64) -> Self {
        Self {
            success: true,
            message: "reached".into(),
            final_position_m: position_m,
            action_time,
        }
    }

    fn timed_out(position_m: f64, action_time: f64) -> Self {
        Self {
            success: false,
            message: "timeout".into(),
            final_position_m: position_m,
            action_time,
        }
    }
}

/// Spawn-and-loop entry for the move_gripper action: accept goal → run control loop
/// → complete the goal, then re-enter. A goal arriving mid-motion is rejected
/// (single-flight); the instance lock in main.rs enforces single-instance.
pub async fn run_move_gripper(
    runner: Arc<NodeRunner>,
    gripper: Arc<Mutex<GripperCan>>,
    cfg: ControlConfig,
    busy: Arc<AtomicBool>,
) {
    let mut handle = move_gripper::ActionHandle::expose(&runner)
        .await
        .expect("expose move_gripper");

    // Single-flight gate shared with the follow loop (created in main): a move
    // and the stream never both drive the gripper, and a new goal arriving while
    // a motion runs is rejected rather than queued (it would run stale). The
    // motion runs in a spawned task so the loop keeps listening (and rejecting).
    loop {
        let ctx = match handle
            .handle_goal_next_request(|req| {
                // Reject targets outside the gripper's physical travel (also
                // rejects NaN/inf, which Limit::contains treats as out of range).
                if !GRIPPER_LIMITS_M.contains(req.data.position) {
                    return Ok(move_gripper::GoalResponse::reject(
                        "target position out of range",
                    ));
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
        };

        let gripper = Arc::clone(&gripper);
        let cfg = cfg.clone();
        let busy = Arc::clone(&busy);
        tokio::spawn(async move {
            let result = run_control_loop(&gripper, &cfg, goal).await;
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
    cfg: &ControlConfig,
    goal: AcceptedGoal,
) -> MotionResult {
    // ROS2 reference (v10_simple_hardware write()): mit_control_all with target, then recv_all.
    // No trajectory: a single MIT setpoint held by the motor PD gains; the gripper's
    // short travel needs no profiling.
    // The motor speaks radians; we keep all user-facing units in meters and convert at the FFI.
    let target_motor_rad = geometry::meters_to_motor_rad(goal.target_position_m);
    let start = Instant::now();
    // Absolute timeline the loop paces against, so per-cycle sleep overshoot does
    // not accumulate (as in the openarm teleop reference).
    let mut next_tick = tokio::time::Instant::now();

    loop {
        let motor_rad = {
            let mut g = gripper.lock().unwrap_or_else(|e| e.into_inner());
            g.mit_control(KP, KD, target_motor_rad, 0.0, 0.0);
            g.refresh_all();
            g.recv_all(cfg.recv_timeout_us);
            g.get_state().position
        };
        let position_m = geometry::motor_rad_to_meters(motor_rad);

        let elapsed = start.elapsed();
        let elapsed_secs = elapsed.as_secs_f64();
        let done = (position_m - goal.target_position_m).abs() < cfg.position_tolerance_m;

        if done {
            return MotionResult::reached(position_m, elapsed_secs);
        }
        if elapsed > cfg.motion_timeout {
            return MotionResult::timed_out(position_m, elapsed_secs);
        }

        pace_to_deadline(&mut next_tick, cfg.cycle_period).await;
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
