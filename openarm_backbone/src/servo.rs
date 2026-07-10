//! Guarded servo for move_arm goals whose straight line no joint path can track
//! continuously (reaching them requires a branch change). A discrete IK walk
//! cannot cross the singular surface between branches, but the damped
//! resolved-rate law the operator's streaming jog runs passes through it: the
//! damping bounds the joint rates while the task error carries the arm across,
//! deviating from the line only where the geometry forces it and re-converging
//! beyond. This module runs that law against a reference that walks the line at
//! the end-effector speed cap, held back by a leash whenever the arm falls
//! behind, so a move_arm goal degrades to exactly the motion streaming produces
//! instead of a blind joint-space swing.
//!
//! The plan rolls the identical law out offline (closed-form steps, well under a
//! millisecond each) before accepting the goal, so feasibility and duration are
//! known up front and a goal the servo cannot reach is rejected rather than
//! started.

use std::time::Duration;

use srs_model::nalgebra::{Isometry3, Rotation3, Vector3, Vector6};
use srs_model::{Arm, damped_pseudo_inverse};

use crate::chase::clamp_to_limits;
use crate::trajectory::{PlanLimits, interpolate_pose};
use crate::{ARM_DOF, JointVec};

/// Damping for the resolved-rate steps: the value the operator's streaming jog
/// has proven live, heavy enough to stay bounded through singular postures,
/// light enough not to visibly lag the reference.
const DLS_LAMBDA: f64 = 0.05;
/// Orientation slew rate of the reference (rad/s); position walks at the
/// end-effector speed cap, which has no orientation analogue.
const ROT_RATE_RAD_S: f64 = 1.5;
/// The reference stops walking while the arm is farther than this from it, so a
/// wall crossing is ground through instead of the reference running away.
const LEASH_M: f64 = 0.05;
/// A goal counts as reached within this position / orientation slack, matching
/// the streaming jog's convergence thresholds.
const POS_CONVERGED_M: f64 = 5e-4;
const ROT_CONVERGED_RAD: f64 = 2e-3;
/// Stall detection: over each window, the reference must advance or the goal
/// error (position or orientation) must shrink by the minimum amounts below, or
/// the servo is going nowhere (an unreachable pose, or a wall the damping
/// cannot carry it past).
const STALL_WINDOW: Duration = Duration::from_secs(2);
const MIN_REF_ADVANCE: f64 = 5e-3;
const MIN_ERR_SHRINK_M: f64 = 1e-3;
const MIN_ROT_SHRINK_RAD: f64 = 0.02;
/// Hard ceiling on a servo move; a rollout still running past this is stalled
/// in all but name.
pub const MAX_SERVO_S: f64 = 30.0;

/// One servo move's mutable state: where the reference is on the line and the
/// last stall-window checkpoint. The joint state lives with the caller (the
/// planner's commanded setpoint), which each tick's step advances.
pub struct ServoState {
    start: Isometry3<f64>,
    end: Isometry3<f64>,
    /// Reference progress along the line, 0..=1.
    reference_s: f64,
    /// Stall checkpoint: reference progress and the goal position / orientation
    /// errors at the window start, and the time budget left in the window.
    window_ref_s: f64,
    window_err_m: f64,
    window_err_rad: f64,
    window_left: Duration,
}

/// One tick's outcome.
pub enum ServoStep {
    /// Advanced: the new joint setpoint to command.
    Stepped(JointVec),
    /// Reached the goal pose within tolerance.
    Converged(JointVec),
    /// No progress over a full stall window: the goal is not reachable this way.
    Stalled,
}

impl ServoState {
    /// Distance (m) from `q`'s end-effector to the goal position, for stall and
    /// timeout reporting.
    pub fn position_err_m(&self, model: &mut Arm, q: &JointVec) -> f64 {
        let ee_base = model.at(q).ee_pose();
        let ee = model.world_pose(&ee_base);
        (self.end.translation.vector - ee.translation.vector).norm()
    }

    pub fn new(start: Isometry3<f64>, end: Isometry3<f64>) -> Self {
        Self {
            start,
            end,
            reference_s: 0.0,
            window_ref_s: 0.0,
            window_err_m: (end.translation.vector - start.translation.vector).norm(),
            window_err_rad: start.rotation.angle_to(&end.rotation),
            window_left: STALL_WINDOW,
        }
    }

    /// Advance one tick of `dt`: walk the reference (leashed to the arm), take
    /// one damped resolved-rate step of the joints toward it, and report
    /// convergence or a stall.
    pub fn step(
        &mut self,
        model: &mut Arm,
        q: &JointVec,
        max_joint_velocity_rad_s: &JointVec,
        max_ee_velocity_m_s: f64,
        dt: Duration,
    ) -> ServoStep {
        let dt_s = dt.as_secs_f64();
        let ee_base = model.at(q).ee_pose();
        let ee = model.world_pose(&ee_base);

        // Converged on the goal itself (not merely the reference)?
        let goal_pos_err = (self.end.translation.vector - ee.translation.vector).norm();
        let goal_rot_err = ee.rotation.angle_to(&self.end.rotation);
        if self.reference_s >= 1.0
            && goal_pos_err < POS_CONVERGED_M
            && goal_rot_err < ROT_CONVERGED_RAD
        {
            return ServoStep::Converged(*q);
        }

        // Walk the reference at the speed cap while the arm holds the leash; a
        // zero-length line (pure reorientation) starts fully advanced.
        let line_len = (self.end.translation.vector - self.start.translation.vector).norm();
        let reference = interpolate_pose(&self.start, &self.end, self.reference_s);
        let ref_pos_err = (reference.translation.vector - ee.translation.vector).norm();
        if line_len < POS_CONVERGED_M {
            self.reference_s = 1.0;
        } else if ref_pos_err < LEASH_M {
            self.reference_s = (self.reference_s + max_ee_velocity_m_s * dt_s / line_len).min(1.0);
        }
        let reference = interpolate_pose(&self.start, &self.end, self.reference_s);

        // One damped resolved-rate step toward the reference: position error
        // capped at the speed budget, orientation at the slew budget, both
        // rotated into the arm base frame where the Jacobian lives.
        let dp_world = reference.translation.vector - ee.translation.vector;
        let dp_world = if dp_world.norm() > POS_CONVERGED_M {
            dp_world * (max_ee_velocity_m_s * dt_s / dp_world.norm()).min(1.0)
        } else {
            Vector3::zeros()
        };
        let rot_err: Rotation3<f64> =
            (reference.rotation * ee.rotation.inverse()).to_rotation_matrix();
        let dw_world = rot_err
            .axis_angle()
            .map(|(axis, angle)| axis.into_inner() * angle.min(ROT_RATE_RAD_S * dt_s))
            .unwrap_or_else(Vector3::zeros);
        let to_base = model.base_from_world().rotation;
        let dp = to_base * dp_world;
        let dw = to_base * dw_world;
        let twist = Vector6::new(dp.x, dp.y, dp.z, dw.x, dw.y, dw.z);
        let jacobian = model.at(q).jacobian();
        let mut dq = damped_pseudo_inverse(&jacobian, DLS_LAMBDA) * twist;
        // Velocity-consistent scaling: shrink the whole step so every joint stays
        // inside its budget for this tick, preserving direction.
        let scale = (0..ARM_DOF)
            .map(|i| {
                let cap = max_joint_velocity_rad_s[i] * dt_s;
                if dq[i].abs() > cap {
                    cap / dq[i].abs()
                } else {
                    1.0
                }
            })
            .fold(1.0_f64, f64::min);
        dq *= scale;
        let stepped: JointVec = std::array::from_fn(|i| q[i] + dq[i]);
        let next = clamp_to_limits(&stepped, &model.limits());

        // Stall bookkeeping: across each window the reference must move or a
        // goal error (position or orientation, since a move can end with pure
        // rotation left) must shrink; otherwise the law is grinding in place.
        self.window_left = self.window_left.saturating_sub(dt);
        if self.window_left.is_zero() {
            let advanced = self.reference_s - self.window_ref_s >= MIN_REF_ADVANCE;
            let pos_shrunk = self.window_err_m - goal_pos_err >= MIN_ERR_SHRINK_M;
            let rot_shrunk = self.window_err_rad - goal_rot_err >= MIN_ROT_SHRINK_RAD;
            if !advanced && !pos_shrunk && !rot_shrunk {
                return ServoStep::Stalled;
            }
            self.window_ref_s = self.reference_s;
            self.window_err_m = goal_pos_err;
            self.window_err_rad = goal_rot_err;
            self.window_left = STALL_WINDOW;
        }
        ServoStep::Stepped(next)
    }
}

/// Roll the servo law out offline at the control period per step: the plan-time
/// proof that the law reaches the pose, returning how long it took, or `None`
/// when it stalls (or runs past [`MAX_SERVO_S`]). Deterministic and identical to
/// the runtime law, so an accepted goal executes the motion that was validated;
/// a few thousand closed-form steps cost milliseconds.
pub fn rollout(
    model: &mut Arm,
    start: &Isometry3<f64>,
    end: &Isometry3<f64>,
    seed: JointVec,
    limits: &PlanLimits,
) -> Option<f64> {
    let mut state = ServoState::new(*start, *end);
    let mut q = seed;
    let dt = limits.control_period;
    let steps = (MAX_SERVO_S / dt.as_secs_f64()).ceil() as usize;
    for k in 0..steps {
        match state.step(
            model,
            &q,
            limits.max_joint_velocity_rad_s,
            limits.max_ee_velocity_m_s,
            dt,
        ) {
            ServoStep::Stepped(next) => q = next,
            ServoStep::Converged(_) => return Some(k as f64 * dt.as_secs_f64()),
            ServoStep::Stalled => return None,
        }
    }
    None
}
