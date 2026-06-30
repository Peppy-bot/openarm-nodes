//! Setpoint shaping for the stream follow mode: the velocity-limited chase that
//! absorbs target jumps, and the clamp that keeps streamed joint targets inside
//! the arm's limits.

use srs_model::Limit;

use crate::{ARM_DOF, JointVec};

/// Advance the setpoint one tick toward the target, each joint stepping at
/// most `vmax * dt`: the velocity-limited chase that absorbs target jumps.
pub(super) fn chase_step(
    setpoint: &JointVec,
    target: &JointVec,
    vmax: &JointVec,
    dt: f64,
) -> JointVec {
    std::array::from_fn(|i| {
        let max_step = vmax[i] * dt;
        setpoint[i] + (target[i] - setpoint[i]).clamp(-max_step, max_step)
    })
}

/// Clamp a streamed target into the arm's joint position limits, so an
/// out-of-range command tracks to the nearest reachable value instead of
/// driving past a limit.
pub(super) fn clamp_to_limits(q: &JointVec, limits: &[Limit; ARM_DOF]) -> JointVec {
    std::array::from_fn(|i| q[i].clamp(limits[i].lo, limits[i].hi))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn chase_step_caps_each_joint_at_its_velocity_limit() {
        let setpoint = [0.0; ARM_DOF];
        let mut target = [0.0; ARM_DOF];
        target[0] = 1.0; // far above j1's per-tick step
        target[1] = -1.0; // far below j2's
        let mut vmax = [1.0; ARM_DOF];
        vmax[1] = 2.0;
        let next = chase_step(&setpoint, &target, &vmax, 0.01);
        assert_eq!(next[0], 0.01);
        assert_eq!(next[1], -0.02);
        assert_eq!(next[2..], [0.0; 5]);
    }

    #[test]
    fn chase_step_lands_exactly_on_a_near_target() {
        let mut setpoint = [0.0; ARM_DOF];
        setpoint[0] = 0.995;
        let mut target = [0.0; ARM_DOF];
        target[0] = 1.0; // within one 0.01 step
        let next = chase_step(&setpoint, &target, &[1.0; ARM_DOF], 0.01);
        assert_eq!(next[0], 1.0);
    }

    #[test]
    fn chase_step_holds_when_target_equals_setpoint() {
        let q = [0.3, -0.2, 0.1, 0.5, 0.0, -0.4, 0.2];
        assert_eq!(chase_step(&q, &q, &[1.0; ARM_DOF], 0.01), q);
    }

    #[test]
    fn clamp_to_limits_pins_out_of_range_joints() {
        let limits = [Limit { lo: -1.0, hi: 1.0 }; ARM_DOF];
        let mut q = [0.0; ARM_DOF];
        q[0] = 2.0;
        q[1] = -2.0;
        q[2] = 0.5;
        let clamped = clamp_to_limits(&q, &limits);
        assert_eq!(clamped[0], 1.0);
        assert_eq!(clamped[1], -1.0);
        assert_eq!(clamped[2], 0.5);
    }
}
