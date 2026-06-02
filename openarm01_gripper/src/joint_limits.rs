//! Gripper travel limits, owned by this node. Joint limits are a property of the
//! robot description, not of the CAN transport, so they are defined here rather
//! than imported from the CAN library.

/// Inclusive position window `[lower, upper]` (meters) for the gripper.
#[derive(Debug, Clone, Copy)]
pub struct Limit {
    pub lower: f64,
    pub upper: f64,
}

impl Limit {
    const fn new(lower: f64, upper: f64) -> Self {
        Self { lower, upper }
    }

    /// True if `x` lies within `[lower, upper]`. Non-finite values (NaN/inf) are
    /// never finite-bounded, so they are rejected.
    pub fn contains(&self, x: f64) -> bool {
        x.is_finite() && x >= self.lower && x <= self.upper
    }
}

/// Fully-open gripper travel in meters (OpenArm V1.0); the closed end is 0.
const OPEN_M: f64 = 0.044;

/// Physical travel window of the gripper: fully closed (0) to fully open.
pub const GRIPPER_LIMITS_M: Limit = Limit::new(0.0, OPEN_M);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn travel_window_is_closed_to_open() {
        assert!((GRIPPER_LIMITS_M.lower - 0.0).abs() < 1e-12);
        assert!((GRIPPER_LIMITS_M.upper - OPEN_M).abs() < 1e-12);
        assert!(GRIPPER_LIMITS_M.lower < GRIPPER_LIMITS_M.upper);
    }

    #[test]
    fn contains_rejects_out_of_range_and_non_finite() {
        assert!(GRIPPER_LIMITS_M.contains(0.0));
        assert!(GRIPPER_LIMITS_M.contains(OPEN_M)); // inclusive
        assert!(!GRIPPER_LIMITS_M.contains(-0.001));
        assert!(!GRIPPER_LIMITS_M.contains(OPEN_M + 0.001));
        assert!(!GRIPPER_LIMITS_M.contains(f64::NAN));
        assert!(!GRIPPER_LIMITS_M.contains(f64::INFINITY));
    }
}
