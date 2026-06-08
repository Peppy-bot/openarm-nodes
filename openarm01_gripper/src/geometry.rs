//! Gripper joint↔motor geometry: the linear meters↔radians mapping and the
//! travel limit. The motor speaks radians; user-facing positions are in meters.

/// Fully-open gripper travel in meters (OpenArm V1.0); the closed end is 0.
pub const OPEN_M: f64 = 0.044;

/// Motor angle in radians at full open. The joint position maps linearly to motor
/// angle (0 m ↔ 0 rad, [`OPEN_M`] ↔ [`OPEN_RAD`]); the open direction is negative
/// in the motor frame. Matches ROS2 openarm/v10_simple_hardware.
#[allow(clippy::approx_constant)]
pub const OPEN_RAD: f64 = -1.0472;

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
        assert!(GRIPPER_LIMITS_M.lo < GRIPPER_LIMITS_M.hi);
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
    fn mapping_signs_oppose() {
        // 0 m → 0 rad, OPEN_M → OPEN_RAD. The open direction is negative in the
        // motor frame; a sign flip would drive the gripper the wrong way.
        assert!(OPEN_M > 0.0);
        assert!(OPEN_RAD < 0.0);
    }

    #[test]
    fn meters_to_motor_rad_is_linear_between_endpoints() {
        // Closed and open ends define the line; midpoint should land exactly halfway.
        assert!((meters_to_motor_rad(0.0) - 0.0).abs() < 1e-12);
        assert!((meters_to_motor_rad(OPEN_M) - OPEN_RAD).abs() < 1e-12);
        let mid = meters_to_motor_rad(OPEN_M / 2.0);
        assert!((mid - OPEN_RAD / 2.0).abs() < 1e-12);
    }

    #[test]
    fn motor_rad_and_meters_round_trip() {
        // round-trip catches inverse mismatch (wrong constant in numerator/denominator).
        for pos_m in [0.0, 0.01, OPEN_M / 3.0, OPEN_M] {
            let back = motor_rad_to_meters(meters_to_motor_rad(pos_m));
            assert!((back - pos_m).abs() < 1e-12, "round-trip failed for {pos_m}");
        }
    }
}
