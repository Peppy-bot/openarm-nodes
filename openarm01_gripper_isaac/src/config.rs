// GripperId is constructed only via `new(0|1)`. The private field stops callers
// from minting an arbitrary value and bypassing validation, so side_word /
// instance_id can rely on the invariant instead of carrying "unknown" arms.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct GripperId(u8);

#[derive(Debug)]
pub struct InvalidGripperId(pub u8);

impl std::fmt::Display for InvalidGripperId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "gripper_id must be 0 (left) or 1 (right), got {}", self.0)
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

    pub fn side_word(self) -> &'static str {
        match self.0 {
            0 => "left",
            1 => "right",
            _ => unreachable!("GripperId validated at construction"),
        }
    }

    pub fn instance_id(self) -> &'static str {
        match self.0 {
            0 => "left_gripper",
            1 => "right_gripper",
            _ => unreachable!("GripperId validated at construction"),
        }
    }
}
