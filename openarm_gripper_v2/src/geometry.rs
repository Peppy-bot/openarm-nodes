//! v2 gripper joint↔motor geometry: the linear opening-fraction↔motor-radian
//! mapping for the revolute pinch gripper. The motor speaks radians (0 = closed,
//! full open at a signed angle); the wire speaks the opening fraction
//! (0 = closed, 1 = fully open). The two grippers are mechanically mirrored:
//! the left motor opens toward a positive angle, the right toward a negative
//! one, so the mapping is built per instance from its `gripper_id`.

use openarm_can::v20;

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

    /// Inverse of [`Self::fraction_to_motor_rad`].
    pub fn motor_rad_to_fraction(self, motor_rad: f64) -> f64 {
        motor_rad / self.open_rad
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
    fn out_of_range_ids_are_rejected() {
        assert!(Geometry::from_gripper_id(2).is_none());
    }
}
