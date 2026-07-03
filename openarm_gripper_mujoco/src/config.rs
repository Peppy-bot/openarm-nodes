use std::time::Duration;

use peppygen::Parameters;

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

