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
