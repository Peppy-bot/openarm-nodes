//! v2 gripper joint↔motor geometry: the linear meters↔radians mapping and the travel
//! limit for the revolute pinch gripper. The motor speaks radians (0 = closed, opening
//! toward [`OPEN_RAD`]); user-facing positions are the equivalent jaw opening in meters,
//! so the shared gripper interface (position in meters) serves both generations.

use openarm_can::v20;

/// Fully-open jaw opening in meters. The revolute fingers reach [`OPEN_RAD`] at the motor;
/// this is the corresponding jaw width.
///
/// Placeholder pending hardware calibration of the finger linkage: measure the open jaw
/// width and set this so the meters↔motor map is faithful. The motor-frame travel
/// ([`OPEN_RAD`]) is the enactic reference and is correct; only this width is estimated.
pub const OPEN_M: f64 = 0.06;

/// Motor angle in radians at full open; the closed end is 0. Sourced from openarm_can's
/// v2 gripper constant (the motor opens toward a positive angle).
pub const OPEN_RAD: f64 = v20::GRIPPER_OPEN_RAD;

/// Inclusive position window `[lo, hi]` (meters) for the gripper.
#[derive(Debug, Clone, Copy)]
pub struct Limit {
    pub lo: f64,
    pub hi: f64,
}

impl Limit {
    const fn new(lo: f64, hi: f64) -> Self {
        Self { lo, hi }
    }

    /// True if `x` lies within `[lo, hi]`. Non-finite values (NaN/inf) are never
    /// finite-bounded, so they are rejected.
    pub fn contains(&self, x: f64) -> bool {
        x.is_finite() && x >= self.lo && x <= self.hi
    }
}

/// Physical travel window of the gripper: fully closed (0) to fully open.
pub const GRIPPER_LIMITS_M: Limit = Limit::new(0.0, OPEN_M);

/// Linear joint-meter → motor-radian mapping. Closed = 0 m ↔ 0 rad,
/// open = [`OPEN_M`] ↔ [`OPEN_RAD`].
pub fn meters_to_motor_rad(pos_m: f64) -> f64 {
    (pos_m / OPEN_M) * OPEN_RAD
}

/// Inverse of [`meters_to_motor_rad`]: motor angle back to joint position in meters.
pub fn motor_rad_to_meters(motor_rad: f64) -> f64 {
    (motor_rad / OPEN_RAD) * OPEN_M
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn travel_window_is_closed_to_open() {
        assert!((GRIPPER_LIMITS_M.lo - 0.0).abs() < 1e-12);
        assert!((GRIPPER_LIMITS_M.hi - OPEN_M).abs() < 1e-12);
        const { assert!(GRIPPER_LIMITS_M.lo < GRIPPER_LIMITS_M.hi) };
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

    #[test]
    fn mapping_round_trips_and_opens_positive() {
        // v2 opens toward a positive motor angle (unlike v1's negative convention).
        assert!(OPEN_RAD > 0.0);
        for m in [0.0, OPEN_M / 2.0, OPEN_M] {
            let back = motor_rad_to_meters(meters_to_motor_rad(m));
            assert!((back - m).abs() < 1e-9, "round trip {m} -> {back}");
        }
    }
}
