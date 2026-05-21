#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct GripperId(pub u8);

impl GripperId {
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
