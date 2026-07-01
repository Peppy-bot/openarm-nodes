use std::time::Duration;

use peppygen::Parameters;

/// v1 aperture (m) at full open; closed is 0. Bounds both the move target and
/// the streamed follow setpoint on the prismatic v1.0 gripper.
pub const GRIPPER_OPEN_M: f64 = 0.044;

/// v2 jaw opening (m) at full open. Matches openarm_gripper_v2::geometry::OPEN_M
/// (the real v2 gripper's jaw width); duplicated here rather than shared because
/// that crate pulls in the C++-linked openarm_can, too heavy for a sim node.
const V2_OPEN_M: f64 = 0.06;

/// v2 finger-joint travel (rad) per side at full open. The sim actuates the
/// URDF finger joints directly (0..pi/4), not the 0..pi/2 motor the real gripper
/// drives through the 2:1 linkage; 0 rad is closed and the joint opens toward
/// this magnitude (left positive, right negative in the sim scene).
const V2_FINGER_OPEN_RAD: f64 = std::f64::consts::FRAC_PI_4;

/// Maps the jaw opening (m) that the shared gripper interface speaks to the
/// passthrough value the sim bridge splits across the two finger joints
/// (`per_finger = value / 2`), and back. v1 fingers are prismatic in meters, so
/// the value IS the opening (identity). v2 fingers are revolute: the opening
/// scales to the summed finger angle, signed per side because the right scene
/// joints open toward a negative angle. Closed is 0 in both frames.
#[derive(Copy, Clone, Debug)]
pub struct ApertureMap {
    open_m: f64,
    /// passthrough value = opening_m * gain. 1.0 for v1; the signed
    /// meters -> summed-radian scale for v2.
    gain: f64,
}

impl ApertureMap {
    pub fn for_version(hardware_version: &str, gripper_id: GripperId) -> Self {
        match hardware_version {
            "v1" | "V1" => Self {
                open_m: GRIPPER_OPEN_M,
                gain: 1.0,
            },
            "v2" | "V2" => {
                // Right instance opens toward a negative finger angle; both
                // fingers share the value, so the sign lives on the gain.
                let sign = if gripper_id.as_u8() == 0 { 1.0 } else { -1.0 };
                Self {
                    open_m: V2_OPEN_M,
                    gain: sign * 2.0 * V2_FINGER_OPEN_RAD / V2_OPEN_M,
                }
            }
            other => panic!("hardware_version must be v1 or v2, got {other:?}"),
        }
    }

    /// Aperture (m) at full open; closed is 0. Bounds the move target and the
    /// streamed follow setpoint.
    pub fn open_m(&self) -> f64 {
        self.open_m
    }

    /// Jaw opening (m) -> the passthrough value the sim splits across the fingers.
    pub fn to_wire(&self, opening_m: f64) -> f64 {
        opening_m * self.gain
    }

    /// Measured passthrough value (summed finger positions) -> jaw opening (m).
    pub fn to_aperture(&self, wire: f64) -> f64 {
        wire / self.gain
    }
}

// Follow-loop timing for streamed gripper_commands, parsed from the node
// parameters. No velocity cap: the gripper streams the latest opening directly.
#[derive(Copy, Clone, Debug)]
pub struct ControlParams {
    pub control_period: Duration,
    pub stream_timeout: Duration,
}

impl ControlParams {
    pub fn from_params(p: &Parameters) -> Self {
        assert!(p.control_rate_hz > 0, "control_rate_hz must be > 0");
        assert!(
            p.stream_timeout_s.is_finite() && p.stream_timeout_s > 0.0,
            "stream_timeout_s must be a positive finite number"
        );
        Self {
            control_period: Duration::from_micros(1_000_000 / p.control_rate_hz as u64),
            stream_timeout: Duration::from_secs_f64(p.stream_timeout_s),
        }
    }
}

// GripperId is constructed only via `new(0|1)`. The private field stops callers
// from minting an arbitrary value and bypassing validation, so instance_id can
// rely on the invariant instead of carrying "unknown" arms.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct GripperId(u8);

#[derive(Debug)]
pub struct InvalidGripperId(pub u8);

impl std::fmt::Display for InvalidGripperId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "gripper_id must be 0 (left) or 1 (right), got {}",
            self.0
        )
    }
}

impl std::error::Error for InvalidGripperId {}

impl GripperId {
    pub fn new(id: u8) -> Result<Self, InvalidGripperId> {
        match id {
            0 | 1 => Ok(Self(id)),
            other => Err(InvalidGripperId(other)),
        }
    }

    pub fn as_u8(self) -> u8 {
        self.0
    }

    pub fn instance_id(self) -> &'static str {
        match self.0 {
            0 => "left_gripper",
            1 => "right_gripper",
            _ => unreachable!("GripperId validated at construction"),
        }
    }
}
