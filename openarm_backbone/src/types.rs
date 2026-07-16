//! Shared primitives for the bimanual backbone: arm DOF, the joint vector, the
//! arm side identifier, and the runtime motion-timeout rule.

/// Degrees of freedom of one arm.
pub const ARM_DOF: usize = 7;

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
