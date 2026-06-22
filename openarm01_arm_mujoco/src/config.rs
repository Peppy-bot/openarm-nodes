use std::time::Duration;

use peppygen::Parameters;

use crate::trajectory::JointVec;

// Control params shared by the move action and the follow loop, parsed from the
// node parameters. Names + required-vs-default policy mirror the real arm so the
// versions stay configured the same way: control_rate_hz and stream_timeout_s
// are required, the per-joint velocity caps default to the V1.0 URDF limits.
#[derive(Copy, Clone, Debug)]
pub struct ControlParams {
    pub control_period: Duration,
    pub max_joint_velocity: JointVec,
    pub stream_timeout: Duration,
}

impl ControlParams {
    pub fn from_params(p: &Parameters) -> Self {
        // control_rate_hz feeds Duration::from_micros(1_000_000 / rate); guard
        // against zero, same as the real arm.
        assert!(p.control_rate_hz > 0, "control_rate_hz must be > 0");
        assert!(
            p.stream_timeout_s.is_finite() && p.stream_timeout_s > 0.0,
            "stream_timeout_s must be a positive finite number"
        );
        let max_joint_velocity: JointVec = [
            p.max_joint_velocity_rad_s_1,
            p.max_joint_velocity_rad_s_2,
            p.max_joint_velocity_rad_s_3,
            p.max_joint_velocity_rad_s_4,
            p.max_joint_velocity_rad_s_5,
            p.max_joint_velocity_rad_s_6,
            p.max_joint_velocity_rad_s_7,
        ];
        assert!(
            max_joint_velocity.iter().all(|v| *v > 0.0),
            "all max_joint_velocity_rad_s_N must be > 0"
        );
        Self {
            control_period: Duration::from_micros(1_000_000 / p.control_rate_hz as u64),
            max_joint_velocity,
            stream_timeout: Duration::from_secs_f64(p.stream_timeout_s),
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct ArmId(u8);

#[derive(Debug)]
pub struct InvalidArmId(pub u8);

impl std::fmt::Display for InvalidArmId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "arm_id must be 0 (left) or 1 (right), got {}", self.0)
    }
}

impl std::error::Error for InvalidArmId {}

impl ArmId {
    pub fn new(id: u8) -> Result<Self, InvalidArmId> {
        match id {
            0 | 1 => Ok(Self(id)),
            other => Err(InvalidArmId(other)),
        }
    }

    pub fn side_word(self) -> &'static str {
        match self.0 {
            0 => "left",
            1 => "right",
            _ => unreachable!("ArmId({}) bypassed new()", self.0),
        }
    }

    pub fn instance_id(self) -> &'static str {
        match self.0 {
            0 => "left_arm",
            1 => "right_arm",
            _ => unreachable!("ArmId({}) bypassed new()", self.0),
        }
    }

    pub fn raw(self) -> u8 {
        self.0
    }
}
