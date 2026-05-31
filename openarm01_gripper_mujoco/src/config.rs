#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct GripperId(pub u8);

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

    pub fn side_word(self) -> &'static str {
        match self.0 {
            0 => "left",
            1 => "right",
            _ => "unknown",
        }
    }

    pub fn instance_id(self) -> &'static str {
        match self.0 {
            0 => "left_gripper",
            1 => "right_gripper",
            _ => "unknown",
        }
    }
}
