//! v2 gripper joint↔motor geometry: the linear opening-fraction↔motor-radian
//! mapping for the revolute pinch gripper. The motor speaks radians (0 = closed,
//! full open at a signed angle); the wire speaks the opening fraction
//! (0 = closed, 1 = fully open). The two grippers are mechanically mirrored:
//! the left motor opens toward a positive angle, the right toward a negative
//! one, so the mapping is built per instance from its `gripper_id`.

use openarm_can::v20;

/// DM4310 wire torque full-scale (N*m at the shaft): the POS_FORCE per-unit
/// torque-current field spans 0..=this (enactic MOTOR_LIMIT_PARAMS tMax).
pub const MOTOR_TMAX_NM: f64 = 10.0;

/// The largest effort (N*m at the shaft) an instance configured with
/// `force_limit_pu` will exert: the ceiling reported on gripper_states.
pub fn effort_ceiling_nm(force_limit_pu: f64) -> f64 {
    force_limit_pu * MOTOR_TMAX_NM
}

/// A commanded max effort (N*m, magnitude) as the POS_FORCE per-unit
/// torque-current cap, bounded by the configured `force_limit_pu` ceiling;
/// no commanded preference means the ceiling itself.
pub fn effort_to_torque_pu(max_effort_nm: Option<f64>, force_limit_pu: f64) -> f64 {
    max_effort_nm.map_or(force_limit_pu, |nm| {
        (nm / MOTOR_TMAX_NM).min(force_limit_pu)
    })
}

/// One instance's signed motor mapping, resolved from `gripper_id` at startup.
#[derive(Debug, Clone, Copy)]
pub struct Geometry {
    /// Motor angle (rad) at full open; the closed end is 0. The magnitude is
    /// openarm_can's v2 gripper travel; the sign is this side's opening
    /// direction.
    open_rad: f64,
}

impl Geometry {
    /// The mapping for one gripper instance: 0 (left) opens toward
    /// `+GRIPPER_OPEN_RAD`, 1 (right) is mirrored and opens toward
    /// `-GRIPPER_OPEN_RAD`. `None` for any other id.
    pub fn from_gripper_id(gripper_id: u8) -> Option<Self> {
        let open_rad = match gripper_id {
            0 => v20::GRIPPER_OPEN_RAD,
            1 => -v20::GRIPPER_OPEN_RAD,
            _ => return None,
        };
        Some(Self { open_rad })
    }

    /// Opening fraction (0 = closed, 1 = fully open) to signed motor radians.
    pub fn fraction_to_motor_rad(self, fraction: f64) -> f64 {
        fraction * self.open_rad
    }

    /// Signed motor radians to the wire's opening fraction, clamped to
    /// 0..=1 so encoder readings past the calibrated travel cannot leave
    /// the contract's range. Inverse of [`Self::fraction_to_motor_rad`]
    /// within that travel.
    pub fn motor_rad_to_fraction(self, motor_rad: f64) -> f64 {
        (motor_rad / self.open_rad).clamp(0.0, 1.0)
    }

    /// Measured motor torque (N*m at the shaft) mapped into the opening
    /// frame: positive drives toward open on either side, so the wire's
    /// effort sign is side-consistent despite the mirrored motors.
    pub fn motor_torque_to_effort(self, torque: f64) -> f64 {
        torque * self.open_rad.signum()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sides_open_toward_mirrored_motor_angles() {
        let left = Geometry::from_gripper_id(0).unwrap();
        let right = Geometry::from_gripper_id(1).unwrap();
        assert!(left.fraction_to_motor_rad(1.0) > 0.0);
        assert!(right.fraction_to_motor_rad(1.0) < 0.0);
        assert_eq!(
            left.fraction_to_motor_rad(1.0),
            -right.fraction_to_motor_rad(1.0)
        );
    }

    #[test]
    fn closed_is_zero_and_mapping_round_trips() {
        for geometry in [0, 1].map(|id| Geometry::from_gripper_id(id).unwrap()) {
            assert_eq!(geometry.fraction_to_motor_rad(0.0), 0.0);
            for fraction in [0.25, 0.5, 1.0] {
                let back = geometry.motor_rad_to_fraction(geometry.fraction_to_motor_rad(fraction));
                assert!((back - fraction).abs() < 1e-12, "round trip {fraction}");
            }
        }
    }

    #[test]
    fn measured_fraction_clamps_to_the_wire_range() {
        for geometry in [0, 1].map(|id| Geometry::from_gripper_id(id).unwrap()) {
            let open = geometry.fraction_to_motor_rad(1.0);
            assert_eq!(geometry.motor_rad_to_fraction(open * 1.2), 1.0);
            assert_eq!(geometry.motor_rad_to_fraction(open * -0.1), 0.0);
        }
    }

    #[test]
    fn effort_is_side_consistent_toward_open() {
        let left = Geometry::from_gripper_id(0).unwrap();
        let right = Geometry::from_gripper_id(1).unwrap();
        // The same physical torque toward open is positive on the left motor
        // and negative on the mirrored right motor; both wires report it
        // positive, and toward closed negative.
        assert_eq!(left.motor_torque_to_effort(0.5), 0.5);
        assert_eq!(right.motor_torque_to_effort(-0.5), 0.5);
        assert_eq!(left.motor_torque_to_effort(-0.25), -0.25);
        assert_eq!(right.motor_torque_to_effort(0.25), -0.25);
    }

    #[test]
    fn out_of_range_ids_are_rejected() {
        assert!(Geometry::from_gripper_id(2).is_none());
    }

    #[test]
    fn effort_ceiling_scales_the_wire_full_scale() {
        assert_eq!(effort_ceiling_nm(1.0), MOTOR_TMAX_NM);
        assert_eq!(effort_ceiling_nm(0.2), 2.0);
    }

    #[test]
    fn commanded_effort_converts_and_respects_the_ceiling() {
        // No preference: the configured ceiling applies unchanged.
        assert_eq!(effort_to_torque_pu(None, 0.3), 0.3);
        // Within the ceiling: exact N*m to per-unit conversion.
        assert_eq!(effort_to_torque_pu(Some(1.5), 0.3), 0.15);
        // Above the ceiling: the ceiling wins.
        assert_eq!(effort_to_torque_pu(Some(5.0), 0.3), 0.3);
    }
}
