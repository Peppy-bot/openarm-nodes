// Run a feedback loop on the shared gripper_state cache and republish the target
// opening every tick to survive best-effort QoS drops. Convergence on the
// measured-opening error; stall on per-window opening motion. The sim splits the
// opening across the fingers and servos to it.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use peppygen::NodeRunner;
use peppygen::exposed_actions::openarm01_gripper_actions::v1::move_gripper;
use peppylib::TopicPublisher;
use peppylib::runtime::CancellationToken;
use tracing::{error, info, warn};

use crate::config::GRIPPER_OPEN_M;
use crate::passthrough;
use crate::state::SharedState;

// Tolerance (m) on the measured opening for "reached".
const POSITION_TOLERANCE_M: f64 = 0.002;
const MOTION_TIMEOUT: Duration = Duration::from_secs(30);

// Telemetry older than this counts as no telemetry: the sim streams gripper_states
// continuously, so a gap this long means the stream has stopped, and convergence
// or stall must not be judged from a frozen value.
const STALE_TELEMETRY: Duration = Duration::from_millis(500);

// Opening change over a 500ms window; below STALL_EPSILON_M -> stalled.
const STALL_LOOKBACK_ITERS: u32 = 100;
const STALL_EPSILON_M: f64 = 5e-4;
const FEEDBACK_LOOP_TICK: Duration = Duration::from_millis(5);

// If publishing fails for this many consecutive ticks (one full stall window,
// ~500ms), the gripper isn't being commanded at all, so fail the motion instead
// of letting the stall detector report a false "physical limit".
const MAX_CONSECUTIVE_PUBLISH_FAILURES: u32 = STALL_LOOKBACK_ITERS;

// Shutdown grace: zero the opening for ~50ms after the action loop has exited so
// the sim sees a settled command before teardown. Survives a single best-effort
// drop on the bridge.
const SHUTDOWN_GRACE_TICK: Duration = Duration::from_millis(10);
const SHUTDOWN_GRACE_REPEATS: u32 = 5;

struct AcceptedGoal {
    target_opening_m: f64,
}

struct MotionResult {
    success: bool,
    is_cancelled: bool,
    message: String,
    final_positions: Vec<f64>,
    action_time: f64,
}

pub async fn run(
    runner: Arc<NodeRunner>,
    state: Arc<SharedState>,
    token: CancellationToken,
    passthrough_pub: TopicPublisher,
    gripper_id: u8,
    busy: Arc<AtomicBool>,
) {
    let mut action_handle = move_gripper::ActionHandle::expose(&runner)
        .await
        .expect("expose move_gripper");

    // The single-flight busy gate is shared with the follow loop (created in
    // main), so a goal and the stream never both drive the gripper; a goal
    // arriving mid-motion is actively rejected rather than queued. Notified when
    // a motion clears the gate, so the shutdown hook can hold teardown until any
    // in-flight goal has delivered its terminal result.
    let idle = Arc::new(tokio::sync::Notify::new());

    {
        let busy = busy.clone();
        let idle = idle.clone();
        let passthrough_pub = passthrough_pub.clone();
        runner.on_shutdown(async move {
            // Wait for any in-flight motion to deliver its terminal result so
            // we don't race the action loop's last publish with our zero.
            while busy.load(Ordering::Acquire) {
                let notified = idle.notified();
                tokio::pin!(notified);
                notified.as_mut().enable();
                if !busy.load(Ordering::Acquire) {
                    break;
                }
                notified.await;
            }
            // Drive the opening to 0 over a short grace window so the sim sees a
            // settled command before the runtime tears the publisher down.
            // Repeats survive a single best-effort drop on the bridge.
            for _ in 0..SHUTDOWN_GRACE_REPEATS {
                if let Err(e) = passthrough::publish(&passthrough_pub, gripper_id, 0.0).await {
                    warn!("shutdown publish: {e}");
                }
                tokio::time::sleep(SHUTDOWN_GRACE_TICK).await;
            }
        });
    }

    loop {
        let goal_request =
            action_handle.handle_goal_next_request(|req: &move_gripper::GoalRequest| {
                let pos_m = req.data.position;
                if !(0.0..=GRIPPER_OPEN_M).contains(&pos_m) {
                    return Ok(move_gripper::GoalResponse::reject(format!(
                        "position out of range [0.0, {GRIPPER_OPEN_M}]"
                    )));
                }
                // Single-flight: claim the shared gate in the decider so a goal
                // arriving mid-motion is rejected rather than spawning a second
                // worker that would fight the first (and the follow loop) over
                // the sim. The spawned motion's cleanup releases it.
                if busy
                    .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
                    .is_err()
                {
                    return Ok(move_gripper::GoalResponse::reject(
                        "gripper is already executing a motion",
                    ));
                }
                Ok(move_gripper::GoalResponse::accept())
            });

        let goal_ctx = tokio::select! {
            _ = token.cancelled() => break,
            result = goal_request => {
                match result {
                    Ok(Some(ctx)) => ctx,
                    Ok(None) => break, // action exposed but shutting down
                    Err(e) => {
                        error!("move_gripper goal: {e}");
                        continue;
                    }
                }
            }
        };

        // The decider already claimed the busy gate; releasing it lives in the
        // spawned motion's cleanup so on_shutdown can observe it.
        let goal = AcceptedGoal {
            target_opening_m: goal_ctx.request().data.position,
        };

        let passthrough_pub = passthrough_pub.clone();
        let state = state.clone();
        let token = token.clone();
        let busy = busy.clone();
        let idle = idle.clone();
        tokio::spawn(async move {
            let result = run_control_loop(
                &passthrough_pub,
                gripper_id,
                &state,
                &goal_ctx,
                &token,
                goal,
            )
            .await;

            // complete() and complete_cancelled() take identical field sets;
            // the difference is the client-visible status (Completed vs
            // Cancelled).
            let dispatch = if result.is_cancelled {
                goal_ctx
                    .complete_cancelled(
                        result.success,
                        result.message,
                        result.final_positions,
                        result.action_time,
                    )
                    .await
            } else {
                goal_ctx
                    .complete(
                        result.success,
                        result.message,
                        result.final_positions,
                        result.action_time,
                    )
                    .await
            };
            if let Err(e) = dispatch {
                error!("move_gripper complete: {e}");
            }
            busy.store(false, Ordering::Release);
            idle.notify_waiters();
        });
    }
    info!("move_gripper accept loop exited");
}

async fn run_control_loop(
    passthrough_pub: &TopicPublisher,
    gripper_id: u8,
    state: &Arc<SharedState>,
    goal_ctx: &move_gripper::GoalContext,
    token: &CancellationToken,
    goal: AcceptedGoal,
) -> MotionResult {
    let target = goal.target_opening_m;

    let start = Instant::now();
    let mut window_anchor: Option<f64> = None;
    let mut iter: u32 = 0;
    let mut consecutive_publish_failures: u32 = 0;

    loop {
        // Republish every tick: the passthrough is best-effort, so this is the
        // self-healing path. Idempotent. If it keeps failing, bail before
        // convergence/stall reports false success.
        match passthrough::publish(passthrough_pub, gripper_id, target).await {
            Ok(()) => consecutive_publish_failures = 0,
            Err(e) => {
                consecutive_publish_failures += 1;
                warn!("passthrough publish failed ({consecutive_publish_failures}): {e}");
                if consecutive_publish_failures >= MAX_CONSECUTIVE_PUBLISH_FAILURES {
                    return MotionResult {
                        success: false,
                        is_cancelled: false,
                        message: "passthrough publish failing: gripper not commandable".into(),
                        final_positions: vec![],
                        action_time: start.elapsed().as_secs_f64(),
                    };
                }
            }
        }

        let elapsed = start.elapsed();
        let elapsed_secs = elapsed.as_secs_f64();

        let latest = *state
            .gripper_state
            .lock()
            .unwrap_or_else(|p| p.into_inner());
        let opening = match latest {
            Some(s) if s.recv_at.elapsed() <= STALE_TELEMETRY => s.opening,
            _ => {
                // No fresh telemetry (none yet, or the stream stalled): wait, but
                // honour timeout. A missing or stale sample must not reach the
                // convergence math, where it would falsely report reached/stall.
                if elapsed > MOTION_TIMEOUT {
                    return MotionResult {
                        success: false,
                        is_cancelled: false,
                        message: "no usable telemetry from sim".into(),
                        final_positions: vec![],
                        action_time: elapsed_secs,
                    };
                }
                tokio::select! {
                    _ = token.cancelled() => return cancelled(elapsed_secs, vec![]),
                    _ = goal_ctx.cancel_signal() => return cancelled(elapsed_secs, vec![]),
                    _ = tokio::time::sleep(FEEDBACK_LOOP_TICK) => continue,
                }
            }
        };

        let within_tolerance = (opening - target).abs() < POSITION_TOLERANCE_M;

        iter += 1;
        let stalled = if iter.is_multiple_of(STALL_LOOKBACK_ITERS) {
            let was_stalled = window_anchor
                .map(|prev| (opening - prev).abs() < STALL_EPSILON_M)
                .unwrap_or(false);
            window_anchor = Some(opening);
            was_stalled
        } else {
            false
        };

        if within_tolerance {
            return MotionResult {
                success: true,
                is_cancelled: false,
                message: "reached".into(),
                final_positions: vec![opening],
                action_time: elapsed_secs,
            };
        }
        if stalled {
            return MotionResult {
                success: true,
                is_cancelled: false,
                message: "stalled at physical limit".into(),
                final_positions: vec![opening],
                action_time: elapsed_secs,
            };
        }
        if elapsed > MOTION_TIMEOUT {
            return MotionResult {
                success: false,
                is_cancelled: false,
                message: "timeout".into(),
                final_positions: vec![opening],
                action_time: elapsed_secs,
            };
        }

        tokio::select! {
            _ = token.cancelled() => return cancelled(elapsed_secs, vec![opening]),
            _ = goal_ctx.cancel_signal() => return cancelled(elapsed_secs, vec![opening]),
            _ = tokio::time::sleep(FEEDBACK_LOOP_TICK) => {}
        }
    }
}

fn cancelled(action_time: f64, final_positions: Vec<f64>) -> MotionResult {
    MotionResult {
        success: false,
        is_cancelled: true,
        message: "cancelled".into(),
        final_positions,
        action_time,
    }
}
