use std::future::Future;
use std::sync::{Arc, Mutex, PoisonError};

use tokio::task::JoinSet;

pub mod move_arm_joints;
pub mod move_gripper;

/// In-flight per-goal relay tasks, shared between the accept loops (which
/// spawn into it) and the on_shutdown hook in main.rs. The hook awaits the
/// registry so each relay's downstream cancel_goal propagation and upstream
/// completion reply are awaited obligations instead of racing the runtime
/// teardown that follows the cancellation token firing.
pub type InFlightGoals = Arc<Mutex<JoinSet<()>>>;

/// Spawn a per-goal relay into the registry, first reaping relays that have
/// already finished so the set stays bounded by the number of live goals.
pub fn spawn_goal_task(goals: &InFlightGoals, task: impl Future<Output = ()> + Send + 'static) {
    let mut goals = goals.lock().unwrap_or_else(PoisonError::into_inner);
    while goals.try_join_next().is_some() {}
    goals.spawn(task);
}
