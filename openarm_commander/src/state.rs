use std::sync::Arc;
use std::time::Instant;

use crate::gestures::BakedGesture;
use crate::pose::Jog;

pub const ARM_DOF: usize = openarm_description::ARM_DOF;
// The gripper axis is the unitless opening fraction (0 = closed, 1 = open);
// this is only the startup default for the gripper target.
pub const GRIPPER_CLOSED: f64 = 0.0;

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

    /// The wire `gripper_id` (0 = left, 1 = right); the same 0/1 encoding as the arm.
    pub fn gripper_id(self) -> u8 {
        self.arm_id()
    }

    pub fn label(self) -> &'static str {
        match self {
            Self::Left => "left",
            Self::Right => "right",
        }
    }

    /// The description crate's side selector, for its per-side lookups in the embedded
    /// URDF (the arm chain base link).
    pub fn description(self) -> openarm_description::Side {
        match self {
            Self::Left => openarm_description::Side::Left,
            Self::Right => openarm_description::Side::Right,
        }
    }
}

/// Both sides in a fixed iteration order, for per-side loops.
pub const SIDES: [Side; 2] = [Side::Left, Side::Right];

/// A value stored per side, indexed by [`Side`]: `things[side]` reads or writes the
/// right one, with no left/right accessor split. `Copy` when `T` is, so the small
/// per-tick frames pass by value.
#[derive(Clone, Copy, Debug)]
pub struct BySide<T>([T; 2]);

impl<T> BySide<T> {
    pub const fn new(left: T, right: T) -> Self {
        Self([left, right])
    }
}

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
    // What the operator is actively driving this side toward: a joint target (the setpoint
    // ramps toward it under a velocity/acceleration cap) or a Cartesian jog (stepped toward
    // a world pose one capped increment per tick, held at the reach boundary). None when the
    // side is idle. Arming either space clears the other, and it clears on enable/disable,
    // since the two spaces must not fight.
    pub jog: Option<Jog>,
    // Whether a Cartesian jog is currently held at the reach boundary. Drives one-shot
    // status transitions (blocked <-> moving), so neither message latches or spams.
    pub jog_blocked: bool,
}

impl ArmTarget {
    pub fn home() -> Self {
        Self {
            joints: [0.0; ARM_DOF],
            last_feedback: None,
            established: false,
            in_flight: false,
            preempt: None,
            jog: None,
            jog_blocked: false,
        }
    }
}

#[derive(Clone, Debug)]
pub struct GripperTarget {
    pub position: f64,
    // Measured gripper opening fraction from the gripper_states stream.
    pub last_feedback: Option<f64>,
    // The operator's effort cap (the gripper's effort unit), sent with both the
    // streamed opening and discrete moves. `None` = no preference (the wire's 0):
    // the gripper's configured ceiling stays in charge.
    pub max_effort: Option<f64>,
    // The gripper's reported effort ceiling from gripper_states: `None` until
    // the first report, 0 = the gripper has no effort control (hides the
    // panel's effort slider).
    pub effort_ceiling: Option<f64>,
    // A discrete move_gripper (Actions mode) is in flight: refuses a second Execute
    // and drives the gripper card's in-flight badge. Streaming mode never sets it.
    pub in_flight: bool,
}

impl GripperTarget {
    pub fn closed() -> Self {
        Self {
            position: GRIPPER_CLOSED,
            last_feedback: None,
            max_effort: None,
            effort_ceiling: None,
            in_flight: false,
        }
    }
}

/// A gesture in flight: the baked trajectory plus where playback is. `Arc`
/// keeps the per-tick playback advance cheap (each step re-clones the handle,
/// never the trajectory).
#[derive(Clone, Debug)]
pub struct GesturePlayback {
    pub gesture: Arc<BakedGesture>,
    pub phase: GesturePhase,
}

/// Playback phase: a quintic blend from the pose held at start (the retained
/// target, which the backbone is already holding) to each involved track's
/// first sample, then the baked trajectory on a shared clock. `gripper_from`
/// carries the opening measured at start, so a gesture that drives the jaw
/// eases it in over the same blend.
#[derive(Clone, Copy, Debug)]
pub enum GesturePhase {
    LeadIn {
        from: BySide<Option<[f64; ARM_DOF]>>,
        gripper_from: BySide<Option<f64>>,
        t: f64,
        duration_s: f64,
    },
    Playing {
        t: f64,
    },
}

#[derive(Clone, Debug)]
pub struct UiState {
    pub arms: BySide<ArmTarget>,
    pub grippers: BySide<GripperTarget>,
    // The gesture being played, if any. Playback owns its involved sides: their
    // deadmen stay off, discrete fires are refused, and the player writes their
    // arm/gripper targets each tick.
    pub gesture: Option<GesturePlayback>,
    // Streaming deadman, one per side: while false the commander emits no
    // commands for that side's arm or gripper and both targets track the measured
    // pose, so enabling never steps the robot. The arm and gripper share the
    // deadman because the operator enables a whole side at once.
    pub enabled: BySide<bool>,
    // Operator controls for the backbone's self-collision governor, streamed to the
    // backbone on governor_control; the backbone holds its own defaults until the first
    // message. All four launch defaults are node parameters, kept in step with the
    // backbone's, so a deployment tunes startup from one place; the operator then drives
    // them live from the UI.
    pub collision_enabled: bool,
    pub d_stop: f64,
    pub d_safe: f64,
    pub max_ee_velocity_m_s: f64,
    // Joint-slider jog feel, a node parameter so a deployment tunes the ramp without a
    // rebuild: the acceleration the streamed target ramps toward the slider under (the
    // whole jog is acceleration-limited). The backbone still governs the final ramp.
    pub joint_jog_acceleration_rad_s2: f64,
    // The launch governor-enable state, restored on operator disconnect so an
    // operator who turned avoidance off cannot leave the backbone latched ungoverned,
    // while a deployment that launched ungoverned is not force-armed either.
    pub collision_enabled_default: bool,
    // Latest nearest-pair self-collision proximity from the backbone (it carries its own
    // receipt time). `None` until the first message; treated as stale (and rendered
    // n/a) once that receipt time ages past the readout staleness window, so a dead
    // backbone does not leave the last distance latched on the panel.
    pub proximity: Option<Proximity>,
    pub status: String,
}

/// The backbone's reported nearest checked pair: signed surface distance (m, positive
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
        joint_jog_acceleration_rad_s2: f64,
    ) -> Self {
        Self {
            arms: BySide::splat(ArmTarget::home()),
            grippers: BySide::splat(GripperTarget::closed()),
            gesture: None,
            enabled: BySide::splat(false),
            collision_enabled,
            collision_enabled_default: collision_enabled,
            d_stop,
            d_safe,
            max_ee_velocity_m_s,
            joint_jog_acceleration_rad_s2,
            proximity: None,
            status: "ready".to_string(),
        }
    }

    pub fn set_status(&mut self, message: impl Into<String>) {
        self.status = message.into();
    }

    /// Whether a playing gesture owns this side's targets.
    pub fn gesture_holds(&self, side: Side) -> bool {
        self.gesture
            .as_ref()
            .is_some_and(|p| p.gesture.involves(side))
    }

    /// Whether this side's setpoints should stream: the operator's deadman is on,
    /// or a gesture is driving it.
    pub fn side_active(&self, side: Side) -> bool {
        self.enabled[side] || self.gesture_holds(side)
    }
}
