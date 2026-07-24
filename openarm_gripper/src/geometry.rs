//! Gripper joint↔motor geometry: the linear opening-fraction↔motor-radian
//! mapping. The motor speaks radians (0 = closed, full open at [`OPEN_RAD`]);
//! the wire speaks the opening fraction (0 = closed, 1 = fully open). Both v1
//! grippers share the one mapping (the sides are not mirrored).

/// Motor angle in radians at full open; the closed end is 0. The open direction
/// is negative in the motor frame. Matches ROS2 openarm/v10_simple_hardware.
#[allow(clippy::approx_constant)]
pub const OPEN_RAD: f64 = -1.0472;

/// Opening fraction (0 = closed, 1 = fully open) to motor radians.
pub fn fraction_to_motor_rad(fraction: f64) -> f64 {
    fraction * OPEN_RAD
}

/// Motor radians to the wire's opening fraction, clamped to 0..=1 so
/// encoder readings past the calibrated travel cannot leave the contract's
/// range. Inverse of [`fraction_to_motor_rad`] within that travel.
pub fn motor_rad_to_fraction(motor_rad: f64) -> f64 {
    (motor_rad / OPEN_RAD).clamp(0.0, 1.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn closed_is_zero_and_open_is_negative() {
        assert_eq!(fraction_to_motor_rad(0.0), 0.0);
        assert_eq!(fraction_to_motor_rad(1.0), OPEN_RAD);
        const { assert!(OPEN_RAD < 0.0) };
    }

    #[test]
    fn measured_fraction_clamps_to_the_wire_range() {
        assert_eq!(motor_rad_to_fraction(OPEN_RAD * 1.2), 1.0);
        assert_eq!(motor_rad_to_fraction(OPEN_RAD * -0.1), 0.0);
    }

    #[test]
    fn mapping_is_linear_and_round_trips() {
        let mid = fraction_to_motor_rad(0.5);
        assert!((mid - OPEN_RAD / 2.0).abs() < 1e-12);
        for fraction in [0.0, 0.25, 1.0 / 3.0, 1.0] {
            let back = motor_rad_to_fraction(fraction_to_motor_rad(fraction));
            assert!((back - fraction).abs() < 1e-12, "round trip {fraction}");
        }
    }
}
