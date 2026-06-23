// Per-finger control: run a feedback loop on the shared gripper_state cache
// and republish set_ctrl_gripper_<side> every tick to survive best-effort
// QoS drops. Convergence on worst-finger error; stall on per-window motion.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use peppygen::NodeRunner;
use peppygen::exposed_actions::openarm01_gripper::v1::move_gripper;
use peppylib::TopicPublisher;
use peppylib::runtime::CancellationToken;
use tracing::{error, info, warn};

use crate::config::GRIPPER_OPEN_M;
use crate::setctrl;
use crate::state::SharedState;

// Per-finger tolerance (meters) for "reached target". `position` is the
// per-finger displacement (0 closed → ~0.044 fully open in the openarm MJCF);
// each finger is independently driven to that value.
const POSITION_TOLERANCE_M: f64 = 0.002;
const MOTION_TIMEOUT: Duration = Duration::from_secs(30);

// Sum-of-positions diff over a 500ms window; below STALL_EPSILON_M → stalled.
// Sum is fine for the gripper: both fingers move in the same direction.
const STALL_LOOKBACK_ITERS: u32 = 100;
const STALL_EPSILON_M: f64 = 5e-4;
const FEEDBACK_LOOP_TICK: Duration = Duration::from_millis(5);

// If set_ctrl publishing fails for this many consecutive ticks (one full stall
// window, ~500ms), the gripper isn't being commanded at all, so fail the motion
// instead of letting the stall detector report a false "physical limit".
const MAX_CONSECUTIVE_PUBLISH_FAILURES: u32 = STALL_LOOKBACK_ITERS;

// Shutdown grace: zero ctrl for ~50ms after the action loop has exited so the
// sim bridge sees a settled command before teardown. Survives a single
// best-effort drop on the bridge.
const SHUTDOWN_GRACE_TICK: Duration = Duration::from_millis(10);
const SHUTDOWN_GRACE_REPEATS: u32 = 5;

struct AcceptedGoal {
    target_position_m: f64,
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
    set_ctrl_pub: TopicPublisher,
    actuator_names: Arc<[String; 2]>,
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
        let set_ctrl_pub = set_ctrl_pub.clone();
        let actuator_names = actuator_names.clone();
        runner.on_shutdown(async move {
            // Wait for any in-flight motion to deliver its terminal result so
            // we don't race the action loop's last publish with our zeros.
            while busy.load(Ordering::Acquire) {
                let notified = idle.notified();
                tokio::pin!(notified);
                notified.as_mut().enable();
                if !busy.load(Ordering::Acquire) {
                    break;
                }
                notified.await;
            }
            // Drive ctrl=0 over a short grace window so the sim sees a settled
            // command before the runtime tears the publisher down. Repeats
            // survive a single best-effort drop on the bridge.
            for _ in 0..SHUTDOWN_GRACE_REPEATS {
                if let Err(e) = setctrl::publish(&set_ctrl_pub, &actuator_names, 0.0).await {
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
                // set_ctrl. The spawned motion's cleanup releases it.
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
            target_position_m: goal_ctx.request().data.position,
        };

        let set_ctrl_pub = set_ctrl_pub.clone();
        let state = state.clone();
        let token = token.clone();
        let busy = busy.clone();
        let idle = idle.clone();
        let actuator_names = actuator_names.clone();
        tokio::spawn(async move {
            let result = run_control_loop(
                &set_ctrl_pub,
                &state,
                &actuator_names,
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
    set_ctrl_pub: &TopicPublisher,
    state: &Arc<SharedState>,
    actuator_names: &[String; 2],
    goal_ctx: &move_gripper::GoalContext,
    token: &CancellationToken,
    goal: AcceptedGoal,
) -> MotionResult {
    // goal.target_position_m is total aperture; each finger holds half.
    let per_finger = goal.target_position_m / 2.0;

    let start = Instant::now();
    let mut window_anchor: Option<f64> = None;
    let mut iter: u32 = 0;
    let mut consecutive_publish_failures: u32 = 0;

    loop {
        // Republish every tick: peppylib Standard is best-effort, so this is
        // the self-healing path. Idempotent. If it keeps failing, bail before
        // convergence/stall reports false success.
        match setctrl::publish(set_ctrl_pub, actuator_names, per_finger).await {
            Ok(()) => consecutive_publish_failures = 0,
            Err(e) => {
                consecutive_publish_failures += 1;
                warn!("set_ctrl publish failed ({consecutive_publish_failures}): {e}");
                if consecutive_publish_failures >= MAX_CONSECUTIVE_PUBLISH_FAILURES {
                    return MotionResult {
                        success: false,
                        is_cancelled: false,
                        message: "set_ctrl publish failing: gripper not commandable".into(),
                        final_positions: vec![],
                        action_time: start.elapsed().as_secs_f64(),
                    };
                }
            }
        }

        let elapsed = start.elapsed();
        let elapsed_secs = elapsed.as_secs_f64();

        let latest = state
            .gripper_state
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone();
        let snap = match latest {
            Some(s) if !s.positions.is_empty() => s,
            _ => {
                // No usable telemetry yet: empty positions must not reach the
                // convergence math, where worst_err would fold to 0.0 and falsely
                // report "reached". Wait, but honour timeout.
                if elapsed > MOTION_TIMEOUT {
                    return MotionResult {
                        success: false,
                        is_cancelled: false,
                        message: "no usable telemetry from robot_initializer".into(),
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

        let worst_err = snap
            .positions
            .iter()
            .map(|&q| (q - per_finger).abs())
            .fold(0.0_f64, f64::max);
        let within_tolerance = worst_err < POSITION_TOLERANCE_M;
        let motion_metric: f64 = snap.positions.iter().sum();

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

        if within_tolerance {
            return MotionResult {
                success: true,
                is_cancelled: false,
                message: "reached".into(),
                final_positions: snap.positions,
                action_time: elapsed_secs,
            };
        }
        if stalled {
            return MotionResult {
                success: true,
                is_cancelled: false,
                message: "stalled at physical limit".into(),
                final_positions: snap.positions,
                action_time: elapsed_secs,
            };
        }
        if elapsed > MOTION_TIMEOUT {
            return MotionResult {
                success: false,
                is_cancelled: false,
                message: "timeout".into(),
                final_positions: snap.positions,
                action_time: elapsed_secs,
            };
        }

        tokio::select! {
            _ = token.cancelled() => return cancelled(elapsed_secs, snap.positions),
            _ = goal_ctx.cancel_signal() => return cancelled(elapsed_secs, snap.positions),
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
