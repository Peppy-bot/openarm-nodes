//! Per-joint position limits for the OpenArm V1.0, owned by this node rather than
//! by the CAN library. Limits are a property of the robot description (the URDF),
//! not of the CAN transport, so they live with the controller that enforces them.
//!
//! Left and right are mirror images: joints 1 and 2 have mirrored windows (the
//! rest are symmetric), so the table is selected by `arm_id`. Values are the
//! authored limits from the OpenArm V1.0 URDF and match what `srs_model` derives
//! for the same chain.

/// Degrees of freedom of the arm (one position limit per joint).
pub const ARM_DOF: usize = 7;

/// `arm_id` values, mirroring the rest of the node (0 = left, 1 = right).
pub const ARM_ID_LEFT: u8 = 0;
pub const ARM_ID_RIGHT: u8 = 1;

/// Inclusive position window `[lower, upper]` (radians) for one joint.
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

/// Left-arm joint limits (OpenArm V1.0 URDF: `openarm_left_joint1..7`).
const LEFT: [Limit; ARM_DOF] = [
    Limit::new(-3.490659, 1.3962629999999998),
    Limit::new(-3.3161253267948965, 0.17453267320510335),
    Limit::new(-1.570796, 1.570796),
    Limit::new(0.0, 2.443461), // elbow flex: one-sided (lower bound at 0)
    Limit::new(-1.570796, 1.570796),
    Limit::new(-0.785398, 0.785398),
    Limit::new(-1.570796, 1.570796),
];

/// Right-arm joint limits (OpenArm V1.0 URDF: `openarm_right_joint1..7`). The
/// mirror of [`LEFT`]: joints 1 and 2 have mirrored windows, the rest match.
const RIGHT: [Limit; ARM_DOF] = [
    Limit::new(-1.396263, 3.490659),
    Limit::new(-0.17453267320510335, 3.3161253267948965),
    Limit::new(-1.570796, 1.570796),
    Limit::new(0.0, 2.443461),
    Limit::new(-1.570796, 1.570796),
    Limit::new(-0.785398, 0.785398),
    Limit::new(-1.570796, 1.570796),
];

/// The joint-limit table for one arm. `arm_id` is 0 (left) or 1 (right); any
/// other value is a configuration error and panics at startup.
pub fn for_arm_id(arm_id: u8) -> &'static [Limit; ARM_DOF] {
    match arm_id {
        ARM_ID_LEFT => &LEFT,
        ARM_ID_RIGHT => &RIGHT,
        other => panic!("arm_id must be {ARM_ID_LEFT} (left) or {ARM_ID_RIGHT} (right), got {other}"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn windows_are_ordered_and_elbow_is_one_sided() {
        for table in [&LEFT, &RIGHT] {
            for (i, l) in table.iter().enumerate() {
                assert!(l.lower < l.upper, "joint {i}: {l:?}");
            }
            // Joint 4 (elbow) is one-sided per URDF: lower bound at 0.
            assert!(table[3].lower.abs() < 1e-9, "j4 lower = {}", table[3].lower);
        }
    }

    #[test]
    fn left_and_right_mirror_on_j1_j2_and_match_elsewhere() {
        // j1/j2 windows are negated-and-swapped between sides; j3..j7 identical.
        for (a, b) in [(0usize, 0usize), (1, 1)] {
            assert!((LEFT[a].lower + RIGHT[b].upper).abs() < 1e-6, "j{a} lower/upper mirror");
            assert!((LEFT[a].upper + RIGHT[b].lower).abs() < 1e-6, "j{a} upper/lower mirror");
        }
        for i in 2..ARM_DOF {
            assert!((LEFT[i].lower - RIGHT[i].lower).abs() < 1e-12, "j{i} lower differs");
            assert!((LEFT[i].upper - RIGHT[i].upper).abs() < 1e-12, "j{i} upper differs");
        }
    }

    #[test]
    fn contains_rejects_out_of_range_and_non_finite() {
        let l = Limit::new(-1.0, 1.0);
        assert!(l.contains(0.0));
        assert!(l.contains(-1.0) && l.contains(1.0)); // inclusive
        assert!(!l.contains(1.0001));
        assert!(!l.contains(f64::NAN));
        assert!(!l.contains(f64::INFINITY));
        assert!(!l.contains(f64::NEG_INFINITY));
    }

    #[test]
    fn for_arm_id_selects_side() {
        assert_eq!(for_arm_id(ARM_ID_LEFT)[0].lower, LEFT[0].lower);
        assert_eq!(for_arm_id(ARM_ID_RIGHT)[0].lower, RIGHT[0].lower);
    }

    #[test]
    #[should_panic]
    fn for_arm_id_rejects_unknown() {
        let _ = for_arm_id(2);
    }
}
