use std::sync::{Arc, Mutex};

pub const ARM_DOF: usize = 7;
// Range bounds live in config/joint_limits.json5 (loaded by ui.rs); this is
// only the startup default for the gripper target.
pub const GRIPPER_CLOSED_M: f64 = 0.0;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Side {
    Left,
    Right,
}

impl Side {
    pub fn arm_id(self) -> u8 {
        match self {
            Self::Left => 0,
            Self::Right => 1,
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Self::Left => "left",
            Self::Right => "right",
        }
    }
}

#[derive(Clone, Debug)]
pub struct ArmTarget {
    pub joints: [f64; ARM_DOF],
    pub last_feedback: Option<[f64; ARM_DOF]>,
    pub in_flight: bool,
    // Cancels the in-flight goal so a new Send preempts instead of being
    // rejected by the arm's single-flight gate.
    pub preempt: Option<tokio_util::sync::CancellationToken>,
}

impl ArmTarget {
    pub fn home() -> Self {
        Self {
            joints: [0.0; ARM_DOF],
            last_feedback: None,
            in_flight: false,
            preempt: None,
        }
    }
}

#[derive(Clone, Debug)]
pub struct GripperTarget {
    pub position: f64,
    // Measured gripper opening (m) from the gripper_states stream.
    pub last_feedback: Option<f64>,
}

impl GripperTarget {
    pub fn closed() -> Self {
        Self {
            position: GRIPPER_CLOSED_M,
            last_feedback: None,
        }
    }
}

#[derive(Clone, Debug)]
pub struct UiState {
    pub left_arm: ArmTarget,
    pub right_arm: ArmTarget,
    pub left_gripper: GripperTarget,
    pub right_gripper: GripperTarget,
    // Streaming deadman, one per side: while false the commander emits no
    // commands for that side's arm or gripper and both targets track the measured
    // pose, so enabling never steps the robot. The arm and gripper share the
    // deadman because the operator enables a whole side at once.
    pub left_enabled: bool,
    pub right_enabled: bool,
    pub status: String,
}

impl UiState {
    pub fn new() -> Self {
        Self {
            left_arm: ArmTarget::home(),
            right_arm: ArmTarget::home(),
            left_gripper: GripperTarget::closed(),
            right_gripper: GripperTarget::closed(),
            left_enabled: false,
            right_enabled: false,
            status: "ready".to_string(),
        }
    }

    pub fn enabled(&self, side: Side) -> bool {
        match side {
            Side::Left => self.left_enabled,
            Side::Right => self.right_enabled,
        }
    }

    pub fn set_enabled(&mut self, side: Side, on: bool) {
        match side {
            Side::Left => self.left_enabled = on,
            Side::Right => self.right_enabled = on,
        }
    }

    pub fn arm(&self, side: Side) -> &ArmTarget {
        match side {
            Side::Left => &self.left_arm,
            Side::Right => &self.right_arm,
        }
    }

    pub fn arm_mut(&mut self, side: Side) -> &mut ArmTarget {
        match side {
            Side::Left => &mut self.left_arm,
            Side::Right => &mut self.right_arm,
        }
    }

    pub fn gripper(&self, side: Side) -> &GripperTarget {
        match side {
            Side::Left => &self.left_gripper,
            Side::Right => &self.right_gripper,
        }
    }

    pub fn gripper_mut(&mut self, side: Side) -> &mut GripperTarget {
        match side {
            Side::Left => &mut self.left_gripper,
            Side::Right => &mut self.right_gripper,
        }
    }

    pub fn set_status(&mut self, message: impl Into<String>) {
        self.status = message.into();
    }
}

pub type SharedState = Arc<Mutex<UiState>>;

pub fn new_shared() -> SharedState {
    Arc::new(Mutex::new(UiState::new()))
}
