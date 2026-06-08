use std::time::{Duration, Instant};

use crate::{ARM_DOF, JointVec};

/// Quintic minimum-jerk trajectory in joint space.
///
/// Position profile: s(τ) = 6τ⁵ − 15τ⁴ + 10τ³ with τ = t/T ∈ [0,1].
/// Both s'(0) = s'(1) = 0 and s''(0) = s''(1) = 0, so the joint starts and stops
/// with zero velocity and zero acceleration, the smoothest profile that hits
/// fixed boundary conditions. Peak normalised velocity is 1.875 (at τ = 0.5).
pub struct Trajectory {
    start: JointVec,
    end: JointVec,
    duration: Duration,
    pub motion_start: Instant,
}

impl Trajectory {
    pub fn new(
        start: JointVec,
        end: JointVec,
        max_velocity_rad_s: JointVec,
        min_duration_secs: f64,
    ) -> Self {
        // For each joint, peak velocity = 1.875 * |Δq_i| / T, so the smallest T that
        // respects joint i's limit is T_i = 1.875 * |Δq_i| / v_max_i. The trajectory
        // duration is max over joints (the slowest joint relative to its limit wins),
        // then floored at min_duration_secs. Mirrors ROS2 TOTG behaviour against
        // per-joint URDF velocity limits.
        let secs = start
            .iter()
            .zip(end.iter())
            .zip(max_velocity_rad_s.iter())
            .map(|((s, e), v)| 1.875 * (e - s).abs() / v)
            .fold(min_duration_secs, f64::max);
        Self {
            start,
            end,
            duration: Duration::from_secs_f64(secs),
            motion_start: Instant::now(),
        }
    }

    /// Returns (q_des, dq_des) at time `now`. After completion, q_des holds at
    /// `end` and dq_des is zero, so the controller naturally transitions into
    /// "hold the final setpoint" once the trajectory plays out.
    pub fn sample(&self, now: Instant) -> (JointVec, JointVec) {
        let t_total = self.duration.as_secs_f64();
        // Degenerate trajectory (start == end and min_duration_secs == 0): hold at end.
        if t_total == 0.0 {
            return (self.end, [0.0_f64; ARM_DOF]);
        }
        let elapsed = now.duration_since(self.motion_start).as_secs_f64();
        let tau = (elapsed / t_total).clamp(0.0, 1.0);
        // s(τ) = 6τ⁵ − 15τ⁴ + 10τ³
        let s = ((6.0 * tau - 15.0) * tau + 10.0) * tau * tau * tau;
        // ds/dt = (30τ⁴ − 60τ³ + 30τ²) / T; the polynomial is naturally 0 at τ=1.
        let ds_dt = (((30.0 * tau - 60.0) * tau + 30.0) * tau * tau) / t_total;
        let mut q = [0.0_f64; ARM_DOF];
        let mut dq = [0.0_f64; ARM_DOF];
        for i in 0..ARM_DOF {
            let delta = self.end[i] - self.start[i];
            q[i] = self.start[i] + delta * s;
            dq[i] = delta * ds_dt;
        }
        (q, dq)
    }

    pub fn is_complete(&self, now: Instant) -> bool {
        now.duration_since(self.motion_start) >= self.duration
    }

    /// The goal configuration this trajectory ends at, latched as the hold setpoint
    /// when the move completes.
    pub fn target(&self) -> JointVec {
        self.end
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const V_MAX: JointVec = [1.0; ARM_DOF];
    const EPS: f64 = 1e-9;

    fn approx_eq(a: f64, b: f64) -> bool {
        (a - b).abs() < EPS
    }

    fn vec_approx_eq(a: &[f64], b: &[f64]) -> bool {
        a.iter().zip(b.iter()).all(|(x, y)| approx_eq(*x, *y))
    }

    #[test]
    fn duration_floors_at_min() {
        let start = [0.0; ARM_DOF];
        let mut end = [0.0; ARM_DOF];
        end[0] = 0.01; // T_i = 1.875 * 0.01 = 0.01875 s, below the 100 ms floor
        let traj = Trajectory::new(start, end, V_MAX, 0.1);
        assert_eq!(traj.duration, Duration::from_millis(100));
    }

    #[test]
    fn duration_respects_larger_min() {
        let start = [0.0; ARM_DOF];
        let end = [0.1; ARM_DOF]; // would be ~0.1875 s at v_max=1
        let traj = Trajectory::new(start, end, V_MAX, 5.0);
        assert!(approx_eq(traj.duration.as_secs_f64(), 5.0));
    }

    #[test]
    fn duration_scales_with_largest_relative_motion() {
        let start = [0.0; ARM_DOF];
        let mut end = [0.0; ARM_DOF];
        end[0] = 1.0; // T_0 = 1.875
        end[3] = 0.5; // T_3 = 0.9375, slowest-relative joint wins
        let traj = Trajectory::new(start, end, V_MAX, 0.1);
        assert!(approx_eq(traj.duration.as_secs_f64(), 1.875));
    }

    #[test]
    fn boundary_at_tau_zero() {
        let start = [0.1; ARM_DOF];
        let end = [0.5; ARM_DOF];
        let traj = Trajectory::new(start, end, V_MAX, 0.1);
        let (q, dq) = traj.sample(traj.motion_start);
        assert!(vec_approx_eq(&q, &start));
        assert!(dq.iter().all(|v| v.abs() < EPS));
    }

    #[test]
    fn boundary_at_tau_one() {
        let start = [0.0; ARM_DOF];
        let end = [0.5; ARM_DOF];
        let traj = Trajectory::new(start, end, V_MAX, 0.1);
        let (q, dq) = traj.sample(traj.motion_start + traj.duration);
        assert!(vec_approx_eq(&q, &end));
        assert!(dq.iter().all(|v| v.abs() < EPS));
    }

    #[test]
    fn holds_at_end_past_duration() {
        let start = [0.0; ARM_DOF];
        let end = [1.0; ARM_DOF];
        let traj = Trajectory::new(start, end, V_MAX, 0.1);
        let (q, dq) = traj.sample(traj.motion_start + traj.duration + Duration::from_secs(5));
        assert!(vec_approx_eq(&q, &end));
        assert!(dq.iter().all(|v| v.abs() < EPS));
    }

    #[test]
    fn midpoint_position_is_halfway() {
        // s(0.5) = 6·(1/32) − 15·(1/16) + 10·(1/8) = 0.5
        let start = [0.0; ARM_DOF];
        let end = [1.0; ARM_DOF];
        let traj = Trajectory::new(start, end, V_MAX, 0.1);
        let half = Duration::from_secs_f64(traj.duration.as_secs_f64() / 2.0);
        let (q, _) = traj.sample(traj.motion_start + half);
        assert!(q.iter().all(|v| approx_eq(*v, 0.5)));
    }

    #[test]
    fn midpoint_velocity_is_peak() {
        // ds/dτ at τ=0.5 is 30·(1/16) − 60·(1/8) + 30·(1/4) = 1.875
        // dq/dt = 1.875 · Δq / T
        let start = [0.0; ARM_DOF];
        let end = [1.0; ARM_DOF];
        let traj = Trajectory::new(start, end, V_MAX, 0.1);
        let half = Duration::from_secs_f64(traj.duration.as_secs_f64() / 2.0);
        let (_, dq) = traj.sample(traj.motion_start + half);
        let expected = 1.875 / traj.duration.as_secs_f64();
        assert!(dq.iter().all(|v| approx_eq(*v, expected)));
    }

    #[test]
    fn is_complete_only_after_duration() {
        let q = [0.0; ARM_DOF];
        let traj = Trajectory::new(q, q, V_MAX, 0.1);
        assert!(!traj.is_complete(traj.motion_start));
        assert!(traj.is_complete(traj.motion_start + traj.duration));
        assert!(traj.is_complete(traj.motion_start + traj.duration + Duration::from_millis(1)));
    }
}
