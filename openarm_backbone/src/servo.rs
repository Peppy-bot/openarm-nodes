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
//! millisecond each) before accepting the goal: a goal that converges within
//! [`MAX_SERVO_S`] is accepted with that duration, one that does not is rejected
//! rather than started. That offline proof is the only reachability check the
//! servo needs, so the runtime just runs the law and trusts the plan, with
//! [`MAX_SERVO_S`] as its lone backstop.

use std::time::Duration;

use srs_model::Arm;
use srs_model::nalgebra::{Isometry3, Rotation3, Vector3};

use crate::JointVec;
use crate::trajectory::{PlanLimits, interpolate_pose};

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
/// Hard ceiling on a servo move. The plan-time rollout runs at most this long; a
/// goal that has not converged by then is taken as unreachable and rejected, and
/// the runtime aborts a move still going past it (the rare case where the
/// governor holds the arm off its path indefinitely).
pub const MAX_SERVO_S: f64 = 30.0;

/// One servo move's mutable state: where the reference is on the line. The joint
/// state lives with the caller (the planner's commanded setpoint), which each
/// tick's step advances.
pub struct ServoState {
    start: Isometry3<f64>,
    end: Isometry3<f64>,
    /// Reference progress along the line, 0..=1.
    reference_s: f64,
}

/// One tick's outcome.
pub enum ServoStep {
    /// Advanced: the new joint setpoint to command.
    Stepped(JointVec),
    /// Reached the goal pose within tolerance.
    Converged(JointVec),
}

impl ServoState {
    /// Distance (m) from `q`'s end-effector to the goal position, for timeout
    /// reporting.
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
        }
    }

    /// Advance one tick of `dt`: walk the reference (leashed to the arm), take
    /// one damped resolved-rate step of the joints toward it, and report whether
    /// the goal pose is reached.
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
        // capped at the speed budget, orientation at the slew budget, realized
        // by the shared [`Arm::rate_step`] (velocity-scaled, limit-clamped).
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
        let next = model.rate_step(
            q,
            dp_world,
            dw_world,
            max_joint_velocity_rad_s,
            dt_s,
            DLS_LAMBDA,
        );
        ServoStep::Stepped(next)
    }
}

/// Roll the servo law out offline at the control period per step: the plan-time
/// proof that the law reaches the pose, returning how long it took, or `None`
/// when it has not converged within [`MAX_SERVO_S`] (unreachable this way).
/// Deterministic and identical to the runtime law, so an accepted goal executes
/// the motion that was validated; a few thousand closed-form steps cost
/// milliseconds.
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
        }
    }
    None
}
