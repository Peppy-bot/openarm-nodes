use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use peppygen::NodeRunner;
use peppygen::exposed_actions::move_gripper;
use tracing::{error, warn};

use crate::config::ResolvedSide;
use crate::drivers::mjdata_bus::MjDataBus;

// Per-finger tolerance (meters) for "reached target". `position` is the
// per-finger displacement (0 closed → ~0.044 fully open in the openarm MJCF);
// each finger is independently driven to that value.
const POSITION_TOLERANCE_M: f64 = 0.002;
const MOTION_TIMEOUT: Duration = Duration::from_secs(30);

// Stall detection: when the fingers can't reach the requested target (e.g.,
// pressed against each other at full close, jammed by an object, motor at its
// limit), qpos stops changing. We compare current sum against the sum from
// ~500ms ago (a wide enough window that genuine slow motion accumulates >1mm,
// so only a real hard-stop reads ~0). Threshold of 0.5mm over 500ms = 1mm/s
// — anything below that is treated as stalled. Result message distinguishes
// "stalled at physical limit" from "reached" so callers can tell.
const STALL_LOOKBACK_ITERS: u32 = 100; // 100 × 5ms loop sleep = 500ms window
const STALL_EPSILON_M: f64 = 5e-4;

struct AcceptedGoal {
    target_position_m: f64,
    feedback_period: Duration,
}

struct MotionResult {
    success: bool,
    message: String,
    final_positions: Vec<f64>,
    action_time: f64,
}

impl Default for MotionResult {
    fn default() -> Self {
        Self {
            success: false,
            message: "no result".into(),
            final_positions: vec![],
            action_time: 0.0,
        }
    }
}

fn feedback_period(freq_hz: u32) -> Duration {
    Duration::from_micros(1_000_000 / freq_hz.max(1) as u64)
}

pub async fn run(
    runner: Arc<NodeRunner>,
    bus: Arc<MjDataBus>,
    side: Arc<ResolvedSide>,
) {
    let mut handle = move_gripper::ActionHandle::expose(&runner)
        .await
        .expect("expose move_gripper");

    let pending: Arc<Mutex<Option<AcceptedGoal>>> = Arc::new(Mutex::new(None));

    loop {
        let pending_for_handler = pending.clone();
        if let Err(e) = handle
            .handle_goal_next_request(move |req| {
                let pos_m = req.data.position;
                if !(0.0..=0.044).contains(&pos_m) {
                    return Ok(move_gripper::GoalResponse::new(false));
                }
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

        let goal = pending.lock().unwrap().take();
        let result = match goal {
            Some(g) => run_control_loop(&bus, &side, &handle, g).await,
            None => continue,
        };

        let stash: Arc<Mutex<Option<MotionResult>>> = Arc::new(Mutex::new(Some(result)));
        let stash_for_handler = stash.clone();
        if let Err(e) = handle
            .handle_result_next_request(move |_req| {
                let r = stash_for_handler.lock().unwrap().take().unwrap_or_default();
                Ok(move_gripper::ResultResponse {
                    success: r.success,
                    message: r.message,
                    final_joint_positions: r.final_positions,
                    action_time: r.action_time,
                })
            })
            .await
        {
            error!("move_gripper result: {e}");
        }
    }
}

async fn run_control_loop(
    bus: &Arc<MjDataBus>,
    side: &Arc<ResolvedSide>,
    handle: &move_gripper::ActionHandle,
    goal: AcceptedGoal,
) -> MotionResult {
    // Each finger is driven independently to the same target. Per-finger qpos
    // ranges 0 (closed) → ~0.044 (fully open) in the openarm MJCF.
    let updates: Vec<(usize, f64)> = side
        .finger_ctrl_ids
        .iter()
        .map(|&id| (id, goal.target_position_m))
        .collect();
    bus.write_ctrl(&updates);

    let start = Instant::now();
    let mut last_feedback = Instant::now();
    let geom_filter = side.all_finger_geom_ids();
    // Stall window: every STALL_LOOKBACK_ITERS we snapshot the current sum into
    // `window_anchor`. The next time the iter count crosses the window
    // boundary, we compare against that anchor — so we're measuring motion
    // over a fixed ~500ms wall-clock window, not per-tick.
    let mut window_anchor: Option<f64> = None;
    let mut iter: u32 = 0;

    loop {
        let snap = match bus.snapshot(&side.finger_qpos_addrs, side.ee_body_id, &geom_filter) {
            Ok(s) => s,
            Err(e) => {
                warn!("snapshot: {e}");
                tokio::time::sleep(Duration::from_millis(2)).await;
                continue;
            }
        };

        let elapsed = start.elapsed();
        let elapsed_secs = elapsed.as_secs_f64();
        // Worst-finger error vs target; the slowest finger determines convergence.
        let worst_err = snap
            .qpos
            .iter()
            .map(|&q| (q - goal.target_position_m).abs())
            .fold(0.0_f64, f64::max);
        let within_tolerance = worst_err < POSITION_TOLERANCE_M;
        // Stall metric: total motion across all fingers over the last 500ms.
        // A genuine hard stop reads ~0; real slow motion accumulates above
        // STALL_EPSILON_M.
        let motion_metric: f64 = snap.qpos.iter().sum();

        iter += 1;
        let stalled = if iter % STALL_LOOKBACK_ITERS == 0 {
            let was_stalled = window_anchor
                .map(|prev| (motion_metric - prev).abs() < STALL_EPSILON_M)
                .unwrap_or(false);
            window_anchor = Some(motion_metric);
            was_stalled
        } else {
            false
        };

        if last_feedback.elapsed() >= goal.feedback_period {
            if let Err(e) = handle.emit_feedback(snap.qpos.clone(), elapsed_secs).await {
                warn!("feedback: {e}");
            }
            last_feedback = Instant::now();
        }

        if within_tolerance {
            return MotionResult {
                success: true,
                message: "reached".into(),
                final_positions: snap.qpos,
                action_time: elapsed_secs,
            };
        }
        if stalled {
            return MotionResult {
                success: true,
                message: "stalled at physical limit".into(),
                final_positions: snap.qpos,
                action_time: elapsed_secs,
            };
        }
        if elapsed > MOTION_TIMEOUT {
            return MotionResult {
                success: false,
                message: "timeout".into(),
                final_positions: snap.qpos,
                action_time: elapsed_secs,
            };
        }

        tokio::time::sleep(Duration::from_millis(5)).await;
    }
}
