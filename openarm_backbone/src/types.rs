//! Shared primitives for the bimanual backbone: arm DOF, the joint vector, the
//! arm side identifier, the world-pose wire decomposition, and the runtime
//! motion-timeout rule.

use srs_model::nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion};

/// Degrees of freedom of one arm, from the description that also supplies the
/// URDF the governor and planner run against.
pub const ARM_DOF: usize = openarm_description::ARM_DOF;

/// Norm floor for [`pose_from_wire`]: a quaternion at or below it is refused
/// as naming no rotation at all.
const QUATERNION_MIN_NORM: f64 = 1e-6;

/// Decompose a world-frame pose into the wire `(position, quaternion)` arrays:
/// scalar-last `[x, y, z, w]` out of nalgebra's scalar-first, the outbound
/// mirror of [`pose_from_wire`].
pub fn world_pose_arrays(pose: &Isometry3<f64>) -> ([f64; 3], [f64; 4]) {
    let t = pose.translation.vector;
    let r = pose.rotation;
    ([t.x, t.y, t.z], [r.i, r.j, r.k, r.w])
}

/// Parse a world-frame pose off the wire: three finite position components
/// and a finite, normalizable quaternion. Normalized rather than trusted (the
/// wire is four independent floats); a zero-length one is refused rather than
/// read as identity.
pub fn pose_from_wire(
    position: [f64; 3],
    orientation: [f64; 4],
) -> Result<Isometry3<f64>, &'static str> {
    if !position
        .iter()
        .chain(orientation.iter())
        .all(|v| v.is_finite())
    {
        return Err("non-finite values");
    }
    // The wire is scalar-last [x, y, z, w]; nalgebra's Quaternion is
    // scalar-first, and mixing the two is a silent 90-degree class of error.
    let quaternion = Quaternion::new(
        orientation[3],
        orientation[0],
        orientation[1],
        orientation[2],
    );
    let Some(rotation) = UnitQuaternion::try_new(quaternion, QUATERNION_MIN_NORM) else {
        return Err("an unnormalizable orientation");
    };
    Ok(Isometry3::from_parts(
        Translation3::new(position[0], position[1], position[2]),
        rotation,
    ))
}

/// Grace multiple over a move's nominal duration before the runtime declares it
/// stuck and fails the goal. The nominal proves the unobstructed motion's
/// length; the governor can hold motion off that path, so allow this multiple
/// before aborting. The timeout tracks each move, not a flat ceiling.
pub const MOTION_TIMEOUT_FACTOR: f64 = 2.0;

/// Whether a move that has run `elapsed_s` has overrun its nominal `budget_s`
/// by more than [`MOTION_TIMEOUT_FACTOR`], the runtime abort condition.
pub fn motion_timed_out(elapsed_s: f64, budget_s: f64) -> bool {
    elapsed_s > budget_s * MOTION_TIMEOUT_FACTOR
}

/// One joint-space vector (positions, velocities, or torques), j1..j7.
pub type JointVec = [f64; ARM_DOF];

/// Which arm a message addresses. The wire encodes it as `arm_id` (0 = left,
/// 1 = right); [`Side::from_arm_id`] parses that at the boundary so the rest of
/// the backbone carries a side it cannot get wrong, never a raw `u8`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Side {
    Left,
    Right,
}

impl Side {
    /// Parse a wire `arm_id` (0 = left, 1 = right), or `None` if out of range.
    pub fn from_arm_id(arm_id: u8) -> Option<Self> {
        match arm_id {
            0 => Some(Side::Left),
            1 => Some(Side::Right),
            _ => None,
        }
    }

    /// The wire `arm_id` (0 = left, 1 = right).
    pub fn arm_id(self) -> u8 {
        match self {
            Side::Left => 0,
            Side::Right => 1,
        }
    }

    /// Parse a wire `gripper_id` (0 = left, 1 = right), or `None` if out of range.
    /// The gripper wire uses the same 0/1 encoding as the arm.
    pub fn from_gripper_id(gripper_id: u8) -> Option<Self> {
        Self::from_arm_id(gripper_id)
    }

    /// Index into a left-then-right `[T; 2]`.
    pub fn index(self) -> usize {
        self.arm_id() as usize
    }

    /// Label for logs.
    pub fn label(self) -> &'static str {
        match self {
            Side::Left => "left",
            Side::Right => "right",
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Pins the outbound wire ordering to the inbound one: a scalar-first slip
    // on either side breaks the round trip.
    #[test]
    fn wire_arrays_round_trip_through_pose_from_wire() {
        let pose = Isometry3::from_parts(
            Translation3::new(0.2, -0.4, 0.9),
            UnitQuaternion::from_euler_angles(0.3, -0.5, 1.1),
        );
        let (position, orientation) = world_pose_arrays(&pose);
        let rebuilt = pose_from_wire(position, orientation).expect("round trip");
        assert!((rebuilt.translation.vector - pose.translation.vector).norm() < 1e-12);
        assert!(rebuilt.rotation.angle_to(&pose.rotation) < 1e-12);
    }

    // The timeout scales with the move's nominal duration, not a flat ceiling:
    // an 8 s nominal tolerates up to 16 s (2x), a 1 s nominal only 2 s.
    #[test]
    fn motion_timeout_scales_with_the_nominal_budget() {
        // Long validated motion gets proportionally longer before it is stuck.
        assert!(!motion_timed_out(15.0, 8.0));
        assert!(motion_timed_out(17.0, 8.0));
        // Short validated motion is held to a short leash.
        assert!(!motion_timed_out(1.9, 1.0));
        assert!(motion_timed_out(2.1, 1.0));
    }
}
