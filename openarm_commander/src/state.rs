use std::sync::{Arc, Mutex};
use std::time::Instant;

use crate::pose::{JogMode, Pose};

pub const ARM_DOF: usize = openarm_description::ARM_DOF;
// Range bounds come from the description's URDF (resolved by ui::init_limits);
// this is only the startup default for the gripper target.
pub const GRIPPER_CLOSED_M: f64 = 0.0;

/// An armed Cartesian jog: which component the jog drives (`mode`), the desired
/// world-frame pose (`Position`/`Orientation` modes), and the desired arm angle
/// (`ArmAngle` mode); the unused target is ignored for the active mode.
#[derive(Clone, Copy, Debug)]
pub struct PoseJog {
    pub mode: JogMode,
    pub desired: Pose,
    pub arm_angle: f64,
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

    pub fn from_arm_id(arm_id: u8) -> Option<Self> {
        match arm_id {
            0 => Some(Self::Left),
            1 => Some(Self::Right),
            _ => None,
        }
    }

    /// The wire `gripper_id` (0 = left, 1 = right); the same 0/1 encoding as the arm.
    pub fn gripper_id(self) -> u8 {
        self.arm_id()
    }

    /// Parse a wire `gripper_id` (0 = left, 1 = right), or `None` if out of range.
    pub fn from_gripper_id(gripper_id: u8) -> Option<Self> {
        Self::from_arm_id(gripper_id)
    }

    pub fn label(self) -> &'static str {
        match self {
            Self::Left => "left",
            Self::Right => "right",
        }
    }
}

/// A value stored per side, indexed by [`Side`]: `things[side]` reads or writes the
/// right one, with no left/right accessor split.
#[derive(Clone, Debug)]
pub struct BySide<T>([T; 2]);

impl<T: Clone> BySide<T> {
    pub fn splat(value: T) -> Self {
        Self([value.clone(), value])
    }
}

impl<T> std::ops::Index<Side> for BySide<T> {
    type Output = T;
    fn index(&self, side: Side) -> &T {
        &self.0[side as usize]
    }
}

impl<T> std::ops::IndexMut<Side> for BySide<T> {
    fn index_mut(&mut self, side: Side) -> &mut T {
        &mut self.0[side as usize]
    }
}

#[derive(Clone, Debug)]
pub struct ArmTarget {
    pub joints: [f64; ARM_DOF],
    pub last_feedback: Option<[f64; ARM_DOF]>,
    // Whether `joints` has been initialized from a real measured pose yet. Set once,
    // from the first arm_states feedback, so the target starts where the arm is
    // instead of at the home default; thereafter only streaming and discrete moves
    // move it. Prevents re-seeding the gravity-sagged measured every disable, which
    // ratcheted the arm down across enable/disable cycles.
    pub established: bool,
    pub in_flight: bool,
    // Cancels the in-flight goal so a new Send preempts instead of being
    // rejected by the arm's single-flight gate.
    pub preempt: Option<tokio_util::sync::CancellationToken>,
    // The active Cartesian jog: desired world-frame pose plus which components it
    // tracks (the Lock-RPY toggle). The command stream steps the joint target toward
    // it a capped increment per tick and holds at the reach boundary; None when the
    // operator is not jogging in Cartesian space. Cleared on enable/disable and by
    // joint-space input (the two spaces must not fight).
    pub pose_jog: Option<PoseJog>,
    // Whether the jog is currently held at the reach boundary. Drives one-shot
    // status transitions (blocked <-> moving), so neither message latches or spams.
    pub pose_blocked: bool,
}

impl ArmTarget {
    pub fn home() -> Self {
        Self {
            joints: [0.0; ARM_DOF],
            last_feedback: None,
            established: false,
            in_flight: false,
            preempt: None,
            pose_jog: None,
            pose_blocked: false,
        }
    }
}

#[derive(Clone, Debug)]
pub struct GripperTarget {
    pub position: f64,
    // Measured gripper opening (m) from the gripper_states stream.
    pub last_feedback: Option<f64>,
    // A discrete move_gripper (Actions mode) is in flight: refuses a second Execute
    // and drives the gripper card's in-flight badge. Streaming mode never sets it.
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
}

#[derive(Clone, Debug)]
pub struct UiState {
    pub arms: BySide<ArmTarget>,
    pub grippers: BySide<GripperTarget>,
    // Streaming deadman, one per side: while false the commander emits no
    // commands for that side's arm or gripper and both targets track the measured
    // pose, so enabling never steps the robot. The arm and gripper share the
    // deadman because the operator enables a whole side at once.
    pub enabled: BySide<bool>,
    // Operator controls for the hub's self-collision governor, streamed to the
    // backbone on governor_control; the hub holds its own defaults until the first
    // message. All four launch defaults are node parameters, kept in step with the
    // hub's, so a deployment tunes startup from one place; the operator then drives
    // them live from the UI.
    pub collision_enabled: bool,
    pub d_stop: f64,
    pub d_safe: f64,
    pub max_ee_velocity_m_s: f64,
    // The launch governor-enable state, restored on operator disconnect so an
    // operator who turned avoidance off cannot leave the hub latched ungoverned,
    // while a deployment that launched ungoverned is not force-armed either.
    pub collision_enabled_default: bool,
    // Latest nearest-pair self-collision proximity from the hub (it carries its own
    // receipt time). `None` until the first message; treated as stale (and rendered
    // n/a) once that receipt time ages past the readout staleness window, so a dead
    // hub does not leave the last distance latched on the panel.
    pub proximity: Option<Proximity>,
    pub status: String,
}

/// The hub's reported nearest checked pair: signed surface distance (m, positive
/// is clearance), the two link names, the governor's disposition of the commanded
/// motion, and the local time it was received (for the readout's staleness check).
#[derive(Clone, Debug)]
pub struct Proximity {
    pub distance: f64,
    pub link_a: String,
    pub link_b: String,
    pub disposition: Disposition,
    pub received_at: Instant,
}

/// The governor's disposition of the commanded motion, parsed from the wire's
/// mutually exclusive booleans. Stopped wins if a producer ever set both.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Disposition {
    Clear,
    Throttled,
    Stopped,
}

impl Disposition {
    pub fn from_wire(throttled: bool, stopped: bool) -> Self {
        match (throttled, stopped) {
            (_, true) => Self::Stopped,
            (true, false) => Self::Throttled,
            (false, false) => Self::Clear,
        }
    }
}

impl UiState {
    pub fn new(
        collision_enabled: bool,
        d_stop: f64,
        d_safe: f64,
        max_ee_velocity_m_s: f64,
    ) -> Self {
        Self {
            arms: BySide::splat(ArmTarget::home()),
            grippers: BySide::splat(GripperTarget::closed()),
            enabled: BySide::splat(false),
            collision_enabled,
            collision_enabled_default: collision_enabled,
            d_stop,
            d_safe,
            max_ee_velocity_m_s,
            proximity: None,
            status: "ready".to_string(),
        }
    }

    pub fn set_status(&mut self, message: impl Into<String>) {
        self.status = message.into();
    }
}

pub type SharedState = Arc<Mutex<UiState>>;

pub fn new_shared(
    collision_enabled: bool,
    d_stop: f64,
    d_safe: f64,
    max_ee_velocity_m_s: f64,
) -> SharedState {
    Arc::new(Mutex::new(UiState::new(
        collision_enabled,
        d_stop,
        d_safe,
        max_ee_velocity_m_s,
    )))
}
