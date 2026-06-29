//! Shared primitives for the bimanual hub: arm DOF, the joint vector, a joint
//! setpoint, and the side identifiers.

/// Degrees of freedom of one arm.
pub const ARM_DOF: usize = 7;

/// One joint-space vector (positions, velocities, or torques), j1..j7.
pub type JointVec = [f64; ARM_DOF];

/// A joint-space setpoint: target positions and the velocity feedforward that
/// pairs with them.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Setpoint {
    pub positions: JointVec,
    pub velocities: JointVec,
}

pub const ARM_ID_LEFT: u8 = 0;
pub const ARM_ID_RIGHT: u8 = 1;

/// Map an `arm_id` (0 = left, 1 = right) to an index, or `None` if out of range.
pub fn side_index(arm_id: u8) -> Option<usize> {
    match arm_id {
        ARM_ID_LEFT => Some(0),
        ARM_ID_RIGHT => Some(1),
        _ => None,
    }
}
