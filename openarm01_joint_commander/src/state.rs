use std::sync::Arc;
use std::time::Instant;

use tokio::sync::Mutex;

pub const ARM_DOF: usize = 7;
pub const GRIPPER_OPEN_M: f64 = 0.044;
pub const GRIPPER_CLOSED_M: f64 = 0.0;
pub const GRIPPER_STEP_M: f64 = 0.005;
pub const JOINT_LIMIT_RAD: f64 = std::f64::consts::PI;
pub const DEFAULT_STEP_RAD: f64 = 0.05;
pub const MIN_STEP_RAD: f64 = 0.01;
pub const MAX_STEP_RAD: f64 = 0.5;
pub const STATUS_FRESHNESS_SECS: u64 = 6;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Focus {
    LeftArm,
    RightArm,
    LeftGripper,
    RightGripper,
}

impl Focus {
    pub fn side(self) -> Side {
        match self {
            Self::LeftArm | Self::LeftGripper => Side::Left,
            Self::RightArm | Self::RightGripper => Side::Right,
        }
    }

    pub fn is_arm(self) -> bool {
        matches!(self, Self::LeftArm | Self::RightArm)
    }
}

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

    pub fn gripper_id(self) -> u8 {
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
    pub selected_joint: usize,
    pub last_feedback: Option<[f64; ARM_DOF]>,
    pub in_flight: bool,
}

impl ArmTarget {
    pub fn home() -> Self {
        Self {
            joints: [0.0; ARM_DOF],
            selected_joint: 0,
            last_feedback: None,
            in_flight: false,
        }
    }

    pub fn step_selected(&mut self, delta: f64) {
        let j = self.selected_joint;
        let next = (self.joints[j] + delta).clamp(-JOINT_LIMIT_RAD, JOINT_LIMIT_RAD);
        self.joints[j] = next;
    }
}

#[derive(Clone, Debug)]
pub struct GripperTarget {
    pub position: f64,
    pub last_feedback: Option<Vec<f64>>,
    pub in_flight: bool,
}

impl GripperTarget {
    pub fn closed() -> Self {
        Self {
            position: GRIPPER_CLOSED_M,
            last_feedback: None,
            in_flight: false,
        }
    }

    pub fn step(&mut self, delta: f64) {
        self.position = (self.position + delta).clamp(GRIPPER_CLOSED_M, GRIPPER_OPEN_M);
    }
}

#[derive(Clone, Debug)]
pub struct StatusLine {
    pub message: String,
    pub set_at: Instant,
}

impl StatusLine {
    pub fn new(message: impl Into<String>) -> Self {
        Self {
            message: message.into(),
            set_at: Instant::now(),
        }
    }

    pub fn is_fresh(&self) -> bool {
        self.set_at.elapsed().as_secs() < STATUS_FRESHNESS_SECS
    }
}

#[derive(Clone, Debug)]
pub struct UiState {
    pub left_arm: ArmTarget,
    pub right_arm: ArmTarget,
    pub left_gripper: GripperTarget,
    pub right_gripper: GripperTarget,
    pub focus: Focus,
    pub step_rad: f64,
    pub status: Option<StatusLine>,
}

impl UiState {
    pub fn new() -> Self {
        Self {
            left_arm: ArmTarget::home(),
            right_arm: ArmTarget::home(),
            left_gripper: GripperTarget::closed(),
            right_gripper: GripperTarget::closed(),
            focus: Focus::LeftArm,
            step_rad: DEFAULT_STEP_RAD,
            status: Some(StatusLine::new(
                "ready — [/] arm, {/} gripper, 1-7 joint, ↑↓ step, Enter fire, o/c open/close, h home, q quit",
            )),
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
        self.status = Some(StatusLine::new(message));
    }

    pub fn step_size_inc(&mut self) {
        self.step_rad = (self.step_rad * 2.0).clamp(MIN_STEP_RAD, MAX_STEP_RAD);
    }

    pub fn step_size_dec(&mut self) {
        self.step_rad = (self.step_rad * 0.5).clamp(MIN_STEP_RAD, MAX_STEP_RAD);
    }
}

pub type SharedState = Arc<Mutex<UiState>>;

pub fn new_shared() -> SharedState {
    Arc::new(Mutex::new(UiState::new()))
}
