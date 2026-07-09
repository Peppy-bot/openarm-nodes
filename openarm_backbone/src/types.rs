//! Shared primitives for the bimanual hub: arm DOF, the joint vector, and the
//! arm side identifier.

/// Degrees of freedom of one arm.
pub const ARM_DOF: usize = 7;

/// One joint-space vector (positions, velocities, or torques), j1..j7.
pub type JointVec = [f64; ARM_DOF];

/// Which arm a message addresses. The wire encodes it as `arm_id` (0 = left,
/// 1 = right); [`Side::from_arm_id`] parses that at the boundary so the rest of
/// the hub carries a side it cannot get wrong, never a raw `u8`.
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
