pub mod arm;
pub mod gripper;

use std::sync::atomic::{AtomicBool, Ordering};

/// Claim a side's single-flight move slot, or report it already busy. Shared by
/// the arm and gripper admission handlers; the matching release rides the move's
/// busy guard.
fn claim(busy: &AtomicBool) -> bool {
    busy.compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed)
        .is_ok()
}
