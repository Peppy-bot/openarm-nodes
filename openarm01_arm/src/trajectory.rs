//! Quintic minimum-jerk joint trajectory. The arm is a follower of the hub's
//! governed setpoints and generates no trajectories during normal operation; this
//! is kept solely for the shutdown return-to-ready park, which eases the arm to a
//! known pose over a velocity-limited blend before the motors disable.

use std::time::{Duration, Instant};

use crate::{ARM_DOF, JointVec};

/// Quintic minimum-jerk trajectory in joint space.
pub struct JointTrajectory {
    start: JointVec,
    end: JointVec,
    duration: Duration,
    motion_start: Instant,
}

impl JointTrajectory {
    pub fn new(
        start: JointVec,
        end: JointVec,
        max_velocity_rad_s: JointVec,
        requested_duration_secs: f64,
    ) -> Self {
        // Each joint moves Δq_i over the whole blend (Δs = 1), so its velocity
        // ratio is |Δq_i| / v_max_i; the slowest joint relative to its limit binds.
        // Mirrors ROS2 TOTG behaviour against per-joint URDF velocity limits.
        let peak_ratio = start
            .iter()
            .zip(end.iter())
            .zip(max_velocity_rad_s.iter())
            .map(|((s, e), v)| (e - s).abs() / v)
            .fold(0.0_f64, f64::max);
        let secs = velocity_limited_duration(peak_ratio, requested_duration_secs);
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
        // Degenerate trajectory (start == end and requested_duration_secs == 0): hold at end.
        if t_total == 0.0 {
            return (self.end, [0.0_f64; ARM_DOF]);
        }
        let elapsed = now.duration_since(self.motion_start).as_secs_f64();
        let tau = (elapsed / t_total).clamp(0.0, 1.0);
        let (s, ds_dtau) = quintic(tau);
        let ds_dt = ds_dtau / t_total;
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
}

/// Quintic minimum-jerk blend `s(tau)` and its derivative `ds/dtau` for `tau` in
/// `[0, 1]`: zero velocity and acceleration at both ends.
fn quintic(tau: f64) -> (f64, f64) {
    let s = ((6.0 * tau - 15.0) * tau + 10.0) * tau * tau * tau;
    let ds_dtau = ((30.0 * tau - 60.0) * tau + 30.0) * tau * tau;
    (s, ds_dtau)
}

/// Peak of `ds/dtau` over a unit quintic blend: the trajectory's top speed on a
/// joint moving `Δ` over duration `T` is `QUINTIC_PEAK_VELOCITY · Δ / T`, which is
/// how the velocity-limited duration floor is derived.
const QUINTIC_PEAK_VELOCITY: f64 = 1.875;

/// Floor the requested duration so no joint's quintic peak speed exceeds its
/// velocity limit (`peak_velocity_ratio` is the binding `|Δq_i| / v_max_i`).
fn velocity_limited_duration(peak_velocity_ratio: f64, requested_secs: f64) -> f64 {
    requested_secs.max(QUINTIC_PEAK_VELOCITY * peak_velocity_ratio)
}
