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

    /// The velocity-sized total duration (s), for admission logging.
    pub fn duration_secs(&self) -> f64 {
        self.duration.as_secs_f64()
    }
}

/// Quintic minimum-jerk trajectory between two end-effector poses (world frame).
/// Position rides the quintic blend; orientation is slerped at the same blend
/// parameter, so position and orientation reach the goal together with zero
/// end-point velocity. Duration is sized up-front by [`plan_cartesian`]
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

    /// EE pose at blend parameter `s` on this trajectory's geometric path:
    /// position on the blend between start and end, orientation slerped at the
    /// same parameter. The runtime IK walk samples blends from
    /// [`subdivided_blends`] with this, so its resolution never falls below the
    /// plan's.
    pub fn sample_at_blend(&self, s: f64) -> Isometry3<f64> {
        interpolate_pose(&self.start, &self.end, s.clamp(0.0, 1.0))
    }

    /// Blend parameter `s in [0, 1]` at time `now` (the quintic of elapsed / total,
    /// 1 for a zero-duration trajectory): how far along its geometric path the
    /// trajectory is, fed to [`sample_at_blend`](Self::sample_at_blend). A steered
    /// line's per-tick elbow budget scales by the blend progressed between solves,
    /// mirroring the planner's per-sample cap.
    pub fn blend(&self, now: Instant) -> f64 {
        let t_total = self.duration.as_secs_f64();
        if t_total == 0.0 {
            return 1.0;
        }
        let elapsed = now.duration_since(self.motion_start).as_secs_f64();
        let (s, _) = quintic((elapsed / t_total).clamp(0.0, 1.0));
        s
    }

    pub fn is_complete(&self, now: Instant) -> bool {
        now.duration_since(self.motion_start) >= self.duration
    }
}

/// Path resolution for the up-front Cartesian velocity-limit sizing: the move's
/// geometric path is sampled this many segments and IK-solved at each to bound the
/// per-joint speed. Closed-form IK makes this sub-millisecond.
const CARTESIAN_PLAN_SAMPLES: usize = 100;

/// The plan's validation grid spacing in blend parameter: what one plan sample
/// spans. The runtime must never advance its IK walk coarser than this (see
/// [`subdivided_blends`]), or a short move's quintic could step past geometry the
/// plan validated cell by cell.
pub const CARTESIAN_PLAN_DS: f64 = 1.0 / CARTESIAN_PLAN_SAMPLES as f64;

/// The blend samples one runtime tick must IK-solve, walking from `prev`
/// (exclusive) to `next` (inclusive) in equal steps no wider than
/// [`CARTESIAN_PLAN_DS`]. A tick that progressed no more than one plan cell gets
/// the single sample `next` (including a zero-progress hold tick); a
/// short-duration move whose quintic outpaces the plan grid gets intermediate
/// samples, so the executed IK walk (and its per-step elbow budget) always runs
/// at least as fine as the walk that validated the line.
pub fn subdivided_blends(prev: f64, next: f64) -> impl Iterator<Item = f64> {
    let span = (next - prev).max(0.0);
    let steps = ((span / CARTESIAN_PLAN_DS).ceil() as usize).max(1);
    (1..=steps).map(move |k| prev + span * (k as f64 / steps as f64))
}

/// Arm-angle travel budget for a Cartesian move, in radians per unit blend
/// parameter: at each IK solve the elbow may step at most this far (scaled by the
/// blend progressed since the previous solve) toward higher manipulability, so a
/// move can swivel its elbow up to this much end to end to stay clear of singular
/// postures. The planner and the runtime derive their per-solve caps from the same
/// budget, so the planned dq/ds already includes the elbow motion the execution
/// performs.
pub const ARM_ANGLE_STEP_PER_BLEND_RAD: f64 = 2.0;

/// Largest joint step (rad) one plan sample of the line may demand and still be
/// tracked as a straight line. Above it the exact IK solution has jumped to
/// another branch (a genuine discontinuity: sampling finer does not shrink it),
/// which no continuous tracking can execute.
const MAX_LINE_STEP_RAD: f64 = 0.35;

/// Longest a line plan may run beyond the caller's requested duration before it
/// reads as stuck rather than deliberate. A near-singular graze that slows a line
/// to a few seconds is honest motion and beats a reconfiguration swing; one that
/// balloons past this is effectively unexecutable as a line, so the planner falls
/// through to its next tier.
const MAX_UNREQUESTED_LINE_S: f64 = 10.0;

/// How an accepted move_arm goal executes, decided by [`plan_cartesian`].
pub enum CartesianPlan {
    /// The straight line is continuously trackable: track it over this duration,
    /// resolving the elbow the same way the plan did (`steer_elbow` on means the
    /// per-blend manipulability budget was needed to keep the line alive; off
    /// means the elbow holds its seed angle, the quiet default).
    Line { duration_s: f64, steer_elbow: bool },
    /// No line exists: every continuous tracking demands a branch jump, is
    /// untrackably slow, or leaves reach mid-path. Run a joint-space quintic to
    /// this goal solution instead, reaching the same pose off the line.
    Reconfigure { goal_q: JointVec },
}

/// One policy's walk along the line: the velocity-sizing peak if the line is
/// continuously trackable under that policy.
struct LineWalk {
    /// Peak of `|dq_i/ds| / v_max_i` over the path: the binding joint/segment.
    peak_ratio: f64,
}

/// Walk the geometric path start->end at plan resolution, IK-solving each sample
/// seeded from the previous. `None` when the line is not continuously trackable
/// under `policy`: a per-sample joint step past [`MAX_LINE_STEP_RAD`] (a branch
/// jump) or a mid-path pose with no in-limit solution.
fn walk_line(
    model: &Arm,
    start: &Isometry3<f64>,
    end: &Isometry3<f64>,
    mut seed: JointVec,
    max_joint_velocity_rad_s: &JointVec,
    policy: ArmAnglePolicy,
) -> Option<LineWalk> {
    let mut prev_q: Option<JointVec> = None;
    let mut peak_ratio = 0.0_f64;
    for k in 0..=CARTESIAN_PLAN_SAMPLES {
        let pose = interpolate_pose(start, end, k as f64 * CARTESIAN_PLAN_DS);
        let sol = model.solve_ik(&model.base_pose(&pose), policy, &seed)?;
        if let Some(prev) = prev_q {
            for i in 0..ARM_DOF {
                let step = (sol.q[i] - prev[i]).abs();
                if step > MAX_LINE_STEP_RAD {
                    return None;
                }
                peak_ratio = peak_ratio.max(step / CARTESIAN_PLAN_DS / max_joint_velocity_rad_s[i]);
            }
        }
        prev_q = Some(sol.q);
        seed = sol.q;
    }
    Some(LineWalk { peak_ratio })
}

/// Plan a Cartesian move, preferring the quietest execution that works:
///
/// 1. **Held elbow** ([`ArmAnglePolicy::FromSeed`]): the elbow stays at its seed
///    angle, so an ordinary move never swings joints it does not need. Taken
///    whenever the line tracks at a sane duration.
/// 2. **Steered elbow** ([`ArmAnglePolicy::MaxManipulability`] under the shared
///    [`ARM_ANGLE_STEP_PER_BLEND_RAD`] budget): spends elbow motion only when
///    holding it would break the line (a singular graze inflating the duration,
///    or a limit wall the swivel can dodge).
/// 3. **Reconfiguration**: no line exists at all (every continuous tracking
///    demands a branch jump or leaves reach); swing through joint space to the
///    goal pose's own solution. The only tier that visibly reorganizes the arm,
///    reserved for goals that cannot be reached any other way.
///
/// A tier is accepted when its line tracks continuously and its
/// velocity-limited duration stays within the request (or
/// [`MAX_UNREQUESTED_LINE_S`] past it). `None` only when the goal pose itself
/// is unreachable. Bounding `dq/ds` numerically along the walk turns "respect
/// every joint velocity limit" into a minimum duration via
/// [`velocity_limited_duration`], the same sizing the joint trajectory does
/// analytically. Poses are in the world frame; IK runs in the arm base frame,
/// so each sample is converted with [`Arm::base_pose`].
pub fn plan_cartesian(
    model: &Arm,
    start: &Isometry3<f64>,
    end: &Isometry3<f64>,
    seed: JointVec,
    max_joint_velocity_rad_s: &JointVec,
    requested_duration_secs: f64,
) -> Option<CartesianPlan> {
    let duration_cap = requested_duration_secs.max(MAX_UNREQUESTED_LINE_S);
    let tiers = [
        (ArmAnglePolicy::FromSeed, false),
        (
            ArmAnglePolicy::MaxManipulability {
                max_step_rad: ARM_ANGLE_STEP_PER_BLEND_RAD * CARTESIAN_PLAN_DS,
            },
            true,
        ),
    ];
    for (policy, steer_elbow) in tiers {
        let Some(walk) = walk_line(model, start, end, seed, max_joint_velocity_rad_s, policy)
        else {
            continue;
        };
        let duration_s = velocity_limited_duration(walk.peak_ratio, requested_duration_secs);
        if duration_s <= duration_cap {
            return Some(CartesianPlan::Line {
                duration_s,
                steer_elbow,
            });
        }
    }
    // No line tracks: reconfigure to the goal pose's nearest-branch solution,
    // or reject a goal that is unreachable outright.
    let goal_q = model
        .solve_ik(&model.base_pose(end), ArmAnglePolicy::FromSeed, &seed)?
        .q;
    Some(CartesianPlan::Reconfigure { goal_q })
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
/// quintic) and [`plan_cartesian`] (geometric, `s` uniform). `try_slerp`
/// returns `None` only when the endpoint orientations are numerically identical
/// (quaternion |dot| ≈ 1 after its shortest-arc sign flip, a rotation gap of
/// microradians), so falling back to the goal orientation is exact there, not a
/// jump.
fn interpolate_pose(start: &Isometry3<f64>, end: &Isometry3<f64>, s: f64) -> Isometry3<f64> {
    let position = start.translation.vector.lerp(&end.translation.vector, s);
    let rotation = start
        .rotation
        .try_slerp(&end.rotation, s, 1e-6)
        .unwrap_or(end.rotation);
    Isometry3::from_parts(Translation3::from(position), rotation)
}

#[cfg(test)]
mod tests {
    use super::*;
    use openarm_description::HardwareVersion;
    use srs_model::nalgebra::{Quaternion, Translation3};

    const V_MAX: JointVec = [1.0; ARM_DOF];
    const EPS: f64 = 1e-9;

    // The launcher-default per-joint velocity limits (peppy.json5), j1..j7.
    const V_MAX_V2: JointVec = [
        16.754666, 16.754666, 5.445426, 5.445426, 20.943946, 20.943946, 20.943946,
    ];

    fn v2_right_arm() -> Arm {
        crate::arm_model(HardwareVersion::V2, "openarm_right_base_link")
            .expect("bundled v2 URDF builds")
    }

    fn world_pose(position: [f64; 3], quat_xyzw: [f64; 4]) -> Isometry3<f64> {
        Isometry3::from_parts(
            Translation3::new(position[0], position[1], position[2]),
            UnitQuaternion::from_quaternion(Quaternion::new(
                quat_xyzw[3],
                quat_xyzw[0],
                quat_xyzw[1],
                quat_xyzw[2],
            )),
        )
    }

    // The field repro: pulling the right arm straight back across the base
    // (world x +0.07 -> -0.18 at constant orientation). No continuous joint path
    // tracks that line at any arm angle (the exact IK solution jumps branches
    // mid-path), so line-tracking either inflated the move to tens of seconds or
    // tripped the runtime velocity guard. The plan must classify it as a
    // joint-space reconfiguration whose goal solution lands on the target pose.
    #[test]
    fn cross_body_pull_reconfigures_through_joint_space() {
        let mut model = v2_right_arm();
        let quat = [
            -0.06651768984258864,
            -0.5085494684876904,
            0.32535618836337443,
            0.7944156253074012,
        ];
        let start = world_pose(
            [0.0715597403410507, -0.179708420505458, 0.448631054180598],
            quat,
        );
        let end = world_pose(
            [-0.178440259658949, -0.179708420505458, 0.448631054180598],
            quat,
        );
        // Seed at the start pose, solved from the panel's Ready parking posture.
        let ready = [0.1537, 0.39547, -0.4808, 0.95, -0.0008, 0.0046, -0.0008];
        let seed = model
            .solve_ik(
                &model.base_pose(&start),
                srs_model::ArmAnglePolicy::FromSeed,
                &ready,
            )
            .expect("start pose reachable from ready")
            .q;
        let plan = plan_cartesian(&model, &start, &end, seed, &V_MAX_V2, 2.0)
            .expect("goal pose is reachable");
        let CartesianPlan::Reconfigure { goal_q } = plan else {
            panic!("an untrackable line must degrade to a reconfiguration");
        };
        let goal_ee = model.at(&goal_q).ee_pose();
        let got = model.world_pose(&goal_ee);
        assert!(
            (got.translation.vector - end.translation.vector).norm() < 1e-6,
            "reconfiguration goal must land on the target position"
        );
        assert!(
            got.rotation.angle_to(&end.rotation) < 1e-6,
            "reconfiguration goal must land on the target orientation"
        );
        // The joint move it becomes is sized analytically and stays sane.
        let traj = JointTrajectory::new(seed, goal_q, V_MAX_V2, 2.0);
        assert!(
            traj.duration.as_secs_f64() < 5.0,
            "reconfiguration should take seconds, got {:.1}s",
            traj.duration.as_secs_f64()
        );
    }

    // An easy short move must still plan as a line at exactly the requested
    // duration: the elbow steering must not inflate well-conditioned paths, and
    // trackable lines must never degrade to reconfigurations.
    #[test]
    fn easy_move_plans_the_line_at_the_requested_duration() {
        let model = v2_right_arm();
        let ready = [0.1537, 0.39547, -0.4808, 0.95, -0.0008, 0.0046, -0.0008];
        let start = {
            let mut m = v2_right_arm();
            let p = m.at(&ready).ee_pose();
            model.world_pose(&p)
        };
        let mut end = start;
        end.translation.vector.z += 0.05;
        let plan = plan_cartesian(&model, &start, &end, ready, &V_MAX_V2, 2.0)
            .expect("small lift from ready is reachable");
        let CartesianPlan::Line {
            duration_s,
            steer_elbow,
        } = plan
        else {
            panic!("a trackable line must plan as a line");
        };
        assert!(
            (duration_s - 2.0).abs() < EPS,
            "easy move must stay at the request, got {duration_s:.3}s"
        );
        assert!(!steer_elbow, "an easy move must not spend the elbow budget");
    }

    // The incident regression: after a reconfiguration parks the arm at an
    // unusual arm angle, ordinary nudges from that posture must plan as quiet
    // held-elbow lines, not cascade into further reconfigurations (the steered
    // walk's greedy optimizer can manufacture branch jumps on lines the held
    // walk tracks cleanly, so the quiet tier must be tried first).
    #[test]
    fn small_nudge_from_a_reconfigured_posture_stays_a_quiet_line() {
        let mut model = v2_right_arm();
        let quat = [
            -0.06651768984258864,
            -0.5085494684876904,
            0.32535618836337443,
            0.7944156253074012,
        ];
        let start = world_pose(
            [0.0715597403410507, -0.179708420505458, 0.448631054180598],
            quat,
        );
        let end = world_pose(
            [-0.178440259658949, -0.179708420505458, 0.448631054180598],
            quat,
        );
        let ready = [0.1537, 0.39547, -0.4808, 0.95, -0.0008, 0.0046, -0.0008];
        let seed = model
            .solve_ik(
                &model.base_pose(&start),
                srs_model::ArmAnglePolicy::FromSeed,
                &ready,
            )
            .expect("start pose reachable from ready")
            .q;
        let Some(CartesianPlan::Reconfigure { goal_q }) =
            plan_cartesian(&model, &start, &end, seed, &V_MAX_V2, 2.0)
        else {
            panic!("cross-body pull reconfigures");
        };
        // From the reconfigured posture, nudge 3 cm in +x: an ordinary move.
        let parked_ee = model.at(&goal_q).ee_pose();
        let nudge_start = model.world_pose(&parked_ee);
        let mut nudge_end = nudge_start;
        nudge_end.translation.vector.x += 0.03;
        let plan = plan_cartesian(&model, &nudge_start, &nudge_end, goal_q, &V_MAX_V2, 2.0)
            .expect("nudge from the parked posture is reachable");
        let CartesianPlan::Line {
            duration_s,
            steer_elbow,
        } = plan
        else {
            panic!("an ordinary nudge must stay a line, not reconfigure");
        };
        assert!(!steer_elbow, "an ordinary nudge must hold the elbow");
        assert!(
            (duration_s - 2.0).abs() < EPS,
            "nudge stays at the request, got {duration_s:.3}s"
        );
    }

    #[test]
    fn subdivided_blends_match_plan_resolution() {
        // Within one plan cell (or zero progress): the single sample `next`.
        let one: Vec<f64> = subdivided_blends(0.42, 0.425).collect();
        assert_eq!(one, vec![0.425]);
        let hold: Vec<f64> = subdivided_blends(0.42, 0.42).collect();
        assert_eq!(hold, vec![0.42]);
        // A tick outpacing the grid subdivides evenly: last lands exactly on
        // `next`, and no step exceeds the plan spacing.
        let many: Vec<f64> = subdivided_blends(0.1, 0.1 + 3.7 * CARTESIAN_PLAN_DS).collect();
        assert_eq!(many.len(), 4);
        assert!(approx_eq(
            *many.last().unwrap(),
            0.1 + 3.7 * CARTESIAN_PLAN_DS
        ));
        let mut prev = 0.1;
        for s in many {
            assert!(s - prev <= CARTESIAN_PLAN_DS + EPS);
            prev = s;
        }
    }

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
        assert!(vec_approx_eq(
            &traj.sample(traj.motion_start + traj.duration),
            &end
        ));
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

    // --- plan_cartesian (real arm model) ----------------------------------

    fn left_arm() -> Arm {
        crate::arm_model(
            openarm_description::HardwareVersion::V1,
            "openarm_left_link0",
        )
        .expect("build left arm from bundled URDF")
    }

    #[test]
    fn plan_cartesian_sizes_in_workspace_and_floors_at_request() {
        let mut arm = left_arm();
        let seed = [0.0, 0.3, 0.0, 0.8, 0.0, 0.5, 0.0];
        let ee = arm.at(&seed).ee_pose();
        let start = arm.world_pose(&ee);
        let mut goal = start;
        goal.translation.vector.z += 0.05; // a small reachable move

        let Some(CartesianPlan::Line { duration_s, .. }) =
            plan_cartesian(&arm, &start, &goal, seed, &V_MAX, 0.0)
        else {
            panic!("an in-workspace move should plan a line");
        };
        assert!(
            duration_s > 0.0,
            "an in-workspace move should plan a positive duration"
        );
        // The request floors the velocity-limited duration.
        let Some(CartesianPlan::Line {
            duration_s: floored,
            ..
        }) = plan_cartesian(&arm, &start, &goal, seed, &V_MAX, 5.0)
        else {
            panic!("reachable");
        };
        assert!(
            floored >= 5.0 - EPS,
            "duration must floor at the requested duration"
        );
    }

    #[test]
    fn plan_cartesian_rejects_an_unreachable_goal() {
        let mut arm = left_arm();
        let seed = [0.0, 0.3, 0.0, 0.8, 0.0, 0.5, 0.0];
        let ee = arm.at(&seed).ee_pose();
        let start = arm.world_pose(&ee);
        let mut unreachable = start;
        unreachable.translation.vector.x += 10.0; // 10 m away: no IK solution
        assert!(plan_cartesian(&arm, &start, &unreachable, seed, &V_MAX, 0.0).is_none());
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
        let got = traj.sample_at_blend(traj.blend(traj.motion_start));
        assert!((got.translation.vector - start.translation.vector).norm() < EPS);
        assert!(got.rotation.angle_to(&start.rotation) < EPS);
    }

    #[test]
    fn cartesian_boundary_at_tau_one() {
        let start = pose(0.1, 0.2, 0.3, 0.2);
        let end = pose(0.5, -0.1, 0.4, 1.0);
        let traj = CartesianTrajectory::new(start, end, 2.0);
        let got = traj.sample_at_blend(traj.blend(traj.motion_start + traj.duration));
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
        let got = traj.sample_at_blend(traj.blend(traj.motion_start + half));
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
        let got = traj.sample_at_blend(
            traj.blend(traj.motion_start + traj.duration + Duration::from_secs(3)),
        );
        assert!((got.translation.vector - end.translation.vector).norm() < EPS);
        assert!(got.rotation.angle_to(&end.rotation) < EPS);
    }

    #[test]
    fn cartesian_zero_duration_holds_at_end() {
        let start = pose(0.0, 0.0, 0.0, 0.0);
        let end = pose(0.3, 0.3, 0.3, 0.5);
        let traj = CartesianTrajectory::new(start, end, 0.0);
        let got = traj.sample_at_blend(traj.blend(traj.motion_start));
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
