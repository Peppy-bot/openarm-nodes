use std::time::{Duration, Instant};

use srs_model::nalgebra::{Isometry3, Translation3};
use srs_model::{Arm, ArmAnglePolicy};

use crate::{ARM_DOF, JointVec};

/// Quintic minimum-jerk trajectory in joint space.
pub struct JointTrajectory {
    start: JointVec,
    end: JointVec,
    duration: Duration,
    pub motion_start: Instant,
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

    /// Joint positions at time `now`, on the quintic blend from `start` to `end`.
    /// Holds at `end` once complete (and before `motion_start`, at `start`).
    pub fn sample(&self, now: Instant) -> JointVec {
        let t_total = self.duration.as_secs_f64();
        // Degenerate trajectory (start == end and requested_duration_secs == 0): hold at end.
        if t_total == 0.0 {
            return self.end;
        }
        let elapsed = now.duration_since(self.motion_start).as_secs_f64();
        let tau = (elapsed / t_total).clamp(0.0, 1.0);
        let (s, _) = quintic(tau);
        std::array::from_fn(|i| self.start[i] + (self.end[i] - self.start[i]) * s)
    }

    pub fn is_complete(&self, now: Instant) -> bool {
        now.duration_since(self.motion_start) >= self.duration
    }
}

/// Quintic minimum-jerk trajectory between two end-effector poses (world frame).
/// Position rides the quintic blend; orientation is slerped at the same blend
/// parameter, so position and orientation reach the goal together with zero
/// end-point velocity. Duration is sized up-front by [`plan_cartesian_duration`]
/// so the IK'd path respects the per-joint velocity limits.
pub struct CartesianTrajectory {
    start: Isometry3<f64>,
    end: Isometry3<f64>,
    duration: Duration,
    pub motion_start: Instant,
}

impl CartesianTrajectory {
    pub fn new(start: Isometry3<f64>, end: Isometry3<f64>, duration_secs: f64) -> Self {
        Self {
            start,
            end,
            duration: Duration::from_secs_f64(duration_secs.max(0.0)),
            motion_start: Instant::now(),
        }
    }

    /// EE pose at time `now`: position on the quintic blend between start and end,
    /// orientation slerped at the same blend parameter. Holds at `end` once complete.
    pub fn sample(&self, now: Instant) -> Isometry3<f64> {
        let t_total = self.duration.as_secs_f64();
        if t_total == 0.0 {
            return self.end;
        }
        let elapsed = now.duration_since(self.motion_start).as_secs_f64();
        let tau = (elapsed / t_total).clamp(0.0, 1.0);
        let (s, _) = quintic(tau);
        interpolate_pose(&self.start, &self.end, s)
    }

    pub fn is_complete(&self, now: Instant) -> bool {
        now.duration_since(self.motion_start) >= self.duration
    }
}

/// Path resolution for the up-front Cartesian velocity-limit sizing: the move's
/// geometric path is sampled this many segments and IK-solved at each to bound the
/// per-joint speed. Closed-form IK makes this sub-millisecond.
const CARTESIAN_PLAN_SAMPLES: usize = 100;

/// Plan a Cartesian move: walk the geometric path start->end, solve IK at each
/// sample (seeded for continuity), and return the trajectory duration that keeps
/// every joint within its velocity limit, floored at the caller's request. `None`
/// if any point on the path has no in-limit IK solution (the path is unreachable).
///
/// Bounding `dq/ds` (joint sensitivity to the blend parameter) numerically along
/// the path turns "respect every joint velocity limit" into a minimum duration via
/// [`velocity_limited_duration`], the same sizing the joint trajectory does
/// analytically. Near a singularity `dq/ds` is large, so the move is automatically
/// slowed rather than driven fast through it. Poses are in the world frame; IK runs
/// in the arm base frame, so each sample is converted with [`Arm::base_pose`].
pub fn plan_cartesian_duration(
    model: &Arm,
    start: &Isometry3<f64>,
    end: &Isometry3<f64>,
    seed: JointVec,
    max_joint_velocity_rad_s: &JointVec,
    requested_duration_secs: f64,
) -> Option<f64> {
    let ds = 1.0 / CARTESIAN_PLAN_SAMPLES as f64;
    let mut seed = seed;
    let mut prev_q: Option<JointVec> = None;
    // Peak of |dq_i/ds| / v_max_i over the path: the binding joint/segment.
    let mut peak_ratio = 0.0_f64;
    for k in 0..=CARTESIAN_PLAN_SAMPLES {
        let pose = interpolate_pose(start, end, k as f64 * ds);
        let base_target = model.base_pose(&pose);
        let sol = model.solve_ik(&base_target, ArmAnglePolicy::FromSeed, &seed)?;
        if let Some(prev) = prev_q {
            for i in 0..ARM_DOF {
                let dq_ds = (sol.q[i] - prev[i]).abs() / ds;
                peak_ratio = peak_ratio.max(dq_ds / max_joint_velocity_rad_s[i]);
            }
        }
        prev_q = Some(sol.q);
        seed = sol.q;
    }
    Some(velocity_limited_duration(peak_ratio, requested_duration_secs))
}

// --- Shared blend / sizing helpers -----------------------------------------

/// Peak normalised velocity of the quintic blend `s(τ)`: `ds/dτ` at τ = 0.5. On a
/// quintic of duration `T`, the peak speed of a quantity changing by Δ over the
/// blend is `QUINTIC_PEAK_VELOCITY · Δ / T`, which is how a move's duration is
/// sized to velocity limits (see [`velocity_limited_duration`]).
const QUINTIC_PEAK_VELOCITY: f64 = 1.875;

/// Quintic minimum-jerk blend `s(τ)` and its derivative `ds/dτ`, for τ = t/T ∈
/// [0,1]. `s` runs 0→1 with `s'(0) = s'(1) = 0` and `s''(0) = s''(1) = 0`, so a
/// path blended by it starts and stops with zero velocity and zero acceleration,
/// the smoothest profile that hits fixed boundary conditions. Shared by the
/// joint-space and Cartesian trajectories so both blend identically.
fn quintic(tau: f64) -> (f64, f64) {
    let s = ((6.0 * tau - 15.0) * tau + 10.0) * tau * tau * tau;
    let ds_dtau = ((30.0 * tau - 60.0) * tau + 30.0) * tau * tau;
    (s, ds_dtau)
}

/// Smallest duration (s) that keeps a quintic-blended motion within its velocity
/// limits, floored at `requested_secs` so a caller can ask for a slower move.
/// `peak_velocity_ratio` is the largest `|Δ/Δs| / v_max` over the motion (change
/// per unit blend parameter against that component's limit); the quintic's peak
/// factor scales it to the minimum feasible `T`. Shared by the joint trajectory
/// (ratio from joint deltas) and the Cartesian planner (ratio from the IK'd path).
fn velocity_limited_duration(peak_velocity_ratio: f64, requested_secs: f64) -> f64 {
    requested_secs.max(QUINTIC_PEAK_VELOCITY * peak_velocity_ratio)
}

/// Interpolate between two poses at blend parameter `s` ∈ [0,1]: position by a
/// straight-line lerp, orientation by slerp along the shorter arc (a 180°
/// reorientation interpolates smoothly along one of its two equal-length
/// geodesics). Shared by [`CartesianTrajectory`] (time-sampled, `s` from the
/// quintic) and [`plan_cartesian_duration`] (geometric, `s` uniform). `try_slerp`
/// returns `None` only when the endpoint orientations are numerically identical
/// (quaternion |dot| ≈ 1 after its shortest-arc sign flip, a rotation gap of
/// microradians), so falling back to the goal orientation is exact there, not a
/// jump.
fn interpolate_pose(start: &Isometry3<f64>, end: &Isometry3<f64>, s: f64) -> Isometry3<f64> {
    let position = start.translation.vector.lerp(&end.translation.vector, s);
    let rotation = start.rotation.try_slerp(&end.rotation, s, 1e-6).unwrap_or(end.rotation);
    Isometry3::from_parts(Translation3::from(position), rotation)
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
        let traj = JointTrajectory::new(start, end, V_MAX, 0.1);
        assert_eq!(traj.duration, Duration::from_millis(100));
    }

    #[test]
    fn duration_respects_larger_min() {
        let start = [0.0; ARM_DOF];
        let end = [0.1; ARM_DOF]; // would be ~0.1875 s at v_max=1
        let traj = JointTrajectory::new(start, end, V_MAX, 5.0);
        assert!(approx_eq(traj.duration.as_secs_f64(), 5.0));
    }

    #[test]
    fn duration_scales_with_largest_relative_motion() {
        let start = [0.0; ARM_DOF];
        let mut end = [0.0; ARM_DOF];
        end[0] = 1.0; // T_0 = 1.875
        end[3] = 0.5; // T_3 = 0.9375, slowest-relative joint wins
        let traj = JointTrajectory::new(start, end, V_MAX, 0.1);
        assert!(approx_eq(traj.duration.as_secs_f64(), 1.875));
    }

    #[test]
    fn boundary_at_tau_zero() {
        let start = [0.1; ARM_DOF];
        let end = [0.5; ARM_DOF];
        let traj = JointTrajectory::new(start, end, V_MAX, 0.1);
        assert!(vec_approx_eq(&traj.sample(traj.motion_start), &start));
    }

    #[test]
    fn boundary_at_tau_one() {
        let start = [0.0; ARM_DOF];
        let end = [0.5; ARM_DOF];
        let traj = JointTrajectory::new(start, end, V_MAX, 0.1);
        assert!(vec_approx_eq(&traj.sample(traj.motion_start + traj.duration), &end));
    }

    #[test]
    fn holds_at_end_past_duration() {
        let start = [0.0; ARM_DOF];
        let end = [1.0; ARM_DOF];
        let traj = JointTrajectory::new(start, end, V_MAX, 0.1);
        let q = traj.sample(traj.motion_start + traj.duration + Duration::from_secs(5));
        assert!(vec_approx_eq(&q, &end));
    }

    #[test]
    fn joint_zero_duration_holds_at_end() {
        // start == end with a zero request: the degenerate t_total == 0 branch.
        let q = [0.1, -0.2, 0.3, 0.4, -0.5, 0.6, -0.7];
        let traj = JointTrajectory::new(q, q, V_MAX, 0.0);
        assert!(vec_approx_eq(&traj.sample(traj.motion_start), &q));
    }

    #[test]
    fn midpoint_position_is_halfway() {
        let start = [0.0; ARM_DOF];
        let end = [1.0; ARM_DOF];
        let traj = JointTrajectory::new(start, end, V_MAX, 0.1);
        let half = Duration::from_secs_f64(traj.duration.as_secs_f64() / 2.0);
        let q = traj.sample(traj.motion_start + half);
        assert!(q.iter().all(|v| approx_eq(*v, 0.5)));
    }

    #[test]
    fn quintic_blend_profile() {
        // s runs 0 -> 1 with zero slope at both ends; peak slope QUINTIC_PEAK_VELOCITY
        // at the midpoint. This is the velocity feedforward shape the sampler rides.
        let (s0, d0) = quintic(0.0);
        let (sh, dh) = quintic(0.5);
        let (s1, d1) = quintic(1.0);
        assert!(approx_eq(s0, 0.0) && approx_eq(d0, 0.0));
        assert!(approx_eq(sh, 0.5) && approx_eq(dh, QUINTIC_PEAK_VELOCITY));
        assert!(approx_eq(s1, 1.0) && approx_eq(d1, 0.0));
    }

    #[test]
    fn is_complete_only_after_duration() {
        let q = [0.0; ARM_DOF];
        let traj = JointTrajectory::new(q, q, V_MAX, 0.1);
        assert!(!traj.is_complete(traj.motion_start));
        assert!(traj.is_complete(traj.motion_start + traj.duration));
        assert!(traj.is_complete(traj.motion_start + traj.duration + Duration::from_millis(1)));
    }

    // --- plan_cartesian_duration (real arm model) ------------------------

    fn left_arm() -> Arm {
        Arm::from_urdf_file(&format!("{}/openarm_v10.urdf", env!("CARGO_MANIFEST_DIR")), "openarm_left_link0")
            .expect("build left arm from vendored fixture URDF")
    }

    #[test]
    fn plan_cartesian_duration_sizes_in_workspace_and_floors_at_request() {
        let mut arm = left_arm();
        let seed = [0.0, 0.3, 0.0, 0.8, 0.0, 0.5, 0.0];
        let ee = arm.at(&seed).ee_pose();
        let start = arm.world_pose(&ee);
        let mut goal = start;
        goal.translation.vector.z += 0.05; // a small reachable move

        let dur = plan_cartesian_duration(&arm, &start, &goal, seed, &V_MAX, 0.0);
        assert!(dur.is_some_and(|d| d > 0.0), "an in-workspace move should plan a positive duration");
        // The request floors the velocity-limited duration.
        let floored = plan_cartesian_duration(&arm, &start, &goal, seed, &V_MAX, 5.0).expect("reachable");
        assert!(floored >= 5.0 - EPS, "duration must floor at the requested duration");
    }

    #[test]
    fn plan_cartesian_duration_rejects_an_unreachable_target() {
        let mut arm = left_arm();
        let seed = [0.0, 0.3, 0.0, 0.8, 0.0, 0.5, 0.0];
        let ee = arm.at(&seed).ee_pose();
        let start = arm.world_pose(&ee);
        let mut unreachable = start;
        unreachable.translation.vector.x += 10.0; // 10 m away: no IK solution
        assert!(plan_cartesian_duration(&arm, &start, &unreachable, seed, &V_MAX, 0.0).is_none());
    }

    // --- CartesianTrajectory ---------------------------------------------

    use srs_model::nalgebra::{UnitQuaternion, Vector3};

    fn pose(x: f64, y: f64, z: f64, yaw: f64) -> Isometry3<f64> {
        let r = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), yaw);
        Isometry3::from_parts(Translation3::new(x, y, z), r)
    }

    #[test]
    fn cartesian_180_degree_reorientation_interpolates_smoothly() {
        // Orientations 180° apart have quaternion dot 0, squarely inside slerp's
        // working range (its `None` case is |dot| ≈ 1, i.e. identical endpoint
        // orientations): the blend walks one geodesic continuously rather than
        // jumping to the goal orientation.
        let start = pose(0.0, 0.0, 0.0, 0.0);
        let end = pose(0.0, 0.0, 0.0, std::f64::consts::PI);
        let mut prev = start.rotation;
        for k in 1..=100 {
            let got = interpolate_pose(&start, &end, k as f64 / 100.0);
            let step = got.rotation.angle_to(&prev);
            assert!(step < 0.05, "rotation jumped {step} rad at sample {k}");
            prev = got.rotation;
        }
        assert!(prev.angle_to(&end.rotation) < EPS);
    }

    #[test]
    fn cartesian_boundary_at_tau_zero() {
        let start = pose(0.1, 0.2, 0.3, 0.2);
        let end = pose(0.5, -0.1, 0.4, 1.0);
        let traj = CartesianTrajectory::new(start, end, 2.0);
        let got = traj.sample(traj.motion_start);
        assert!((got.translation.vector - start.translation.vector).norm() < EPS);
        assert!(got.rotation.angle_to(&start.rotation) < EPS);
    }

    #[test]
    fn cartesian_boundary_at_tau_one() {
        let start = pose(0.1, 0.2, 0.3, 0.2);
        let end = pose(0.5, -0.1, 0.4, 1.0);
        let traj = CartesianTrajectory::new(start, end, 2.0);
        let got = traj.sample(traj.motion_start + traj.duration);
        assert!((got.translation.vector - end.translation.vector).norm() < EPS);
        assert!(got.rotation.angle_to(&end.rotation) < EPS);
    }

    #[test]
    fn cartesian_midpoint_position_is_halfway() {
        // s(0.5) = 0.5, so position and orientation are both at the halfway blend.
        let start = pose(0.0, 0.0, 0.0, 0.0);
        let end = pose(1.0, 2.0, -1.0, 1.0);
        let traj = CartesianTrajectory::new(start, end, 2.0);
        let half = Duration::from_secs_f64(traj.duration.as_secs_f64() / 2.0);
        let got = traj.sample(traj.motion_start + half);
        let mid = start.translation.vector.lerp(&end.translation.vector, 0.5);
        assert!((got.translation.vector - mid).norm() < EPS);
        // Halfway in orientation: equal angle to both endpoints.
        let to_start = got.rotation.angle_to(&start.rotation);
        let to_end = got.rotation.angle_to(&end.rotation);
        assert!(approx_eq(to_start, to_end));
    }

    #[test]
    fn cartesian_holds_at_end_past_duration() {
        let start = pose(0.0, 0.0, 0.0, 0.0);
        let end = pose(0.3, 0.3, 0.3, 0.5);
        let traj = CartesianTrajectory::new(start, end, 1.0);
        let got = traj.sample(traj.motion_start + traj.duration + Duration::from_secs(3));
        assert!((got.translation.vector - end.translation.vector).norm() < EPS);
        assert!(got.rotation.angle_to(&end.rotation) < EPS);
    }

    #[test]
    fn cartesian_zero_duration_holds_at_end() {
        let start = pose(0.0, 0.0, 0.0, 0.0);
        let end = pose(0.3, 0.3, 0.3, 0.5);
        let traj = CartesianTrajectory::new(start, end, 0.0);
        let got = traj.sample(traj.motion_start);
        assert!((got.translation.vector - end.translation.vector).norm() < EPS);
        assert!(got.rotation.angle_to(&end.rotation) < EPS);
    }

    #[test]
    fn cartesian_is_complete_only_after_duration() {
        let p = pose(0.0, 0.0, 0.0, 0.0);
        let traj = CartesianTrajectory::new(p, p, 1.0);
        assert!(!traj.is_complete(traj.motion_start));
        assert!(traj.is_complete(traj.motion_start + traj.duration));
    }
}
