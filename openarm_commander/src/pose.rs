//! Cartesian jog for the panel: differential (resolved-rate) servoing only. The
//! joint vector stays the single source of truth: the UI reads a pose off forward
//! kinematics and writes joints back through damped-least-squares steps, so a pose
//! is only ever an input/display lens, never stored state. Exact point-to-point
//! pose moves are the hub's move_arm action, not this module.
//!
//! Poses are in the world frame (matching the hub's move_arm action) and
//! orientation is exposed as intrinsic-XYZ roll/pitch/yaw via nalgebra's
//! `euler_angles` round-trip for display; every orientation computation runs on
//! quaternions, so no control path ever interpolates euler components.

use std::sync::{Arc, Mutex};

use openarm_description::HardwareVersion;
use srs_model::nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion, Vector3, Vector6};
use srs_model::{Arm, ArmAnglePolicy, damped_pseudo_inverse};

use crate::state::{ARM_DOF, Side};

/// A world-frame end-effector pose as `[x, y, z, roll, pitch, yaw]` (metres,
/// radians): the form the panel edits and displays.
pub type Pose = [f64; 6];

/// A discrete move counts as "arrived" within this position / angle slack. The hub
/// reports success once its setpoints finish streaming, which a governor stop satisfies
/// without the arm following, so the move handlers re-check the measured final pose
/// against these. One angle slack serves both joint error and orientation error.
pub const REACHED_POS_TOL_M: f64 = 0.02;
pub const REACHED_ANGLE_TOL_RAD: f64 = 5.0 * std::f64::consts::PI / 180.0; // 5 degrees

/// The two per-side arm models (FK/Jacobian/limits) behind mutexes, plus each
/// side's URDF joint velocity limits for step clamping.
///
/// Each lock is held only for a single synchronous FK or Jacobian call and never
/// across an `.await`. The only nesting is snapshot building, which holds the
/// [`UiState`] lock and then briefly takes a model lock; no path takes the locks
/// in the reverse order, so they cannot deadlock.
///
/// [`UiState`]: crate::state::UiState
#[derive(Clone)]
pub struct ArmModels {
    left: Arc<Mutex<Arm>>,
    right: Arc<Mutex<Arm>>,
    left_velocity_limits: [f64; ARM_DOF],
    right_velocity_limits: [f64; ARM_DOF],
    // World-frame x/y/z reachable bounds per side, from the FK envelope (computed once
    // at construction); the panel sizes its position sliders to these.
    left_bounds: [[f64; 2]; 3],
    right_bounds: [[f64; 2]; 3],
}

impl ArmModels {
    /// Build both arm models from the generation's embedded description, mirroring
    /// the hub's `arm_model` (same URDF, same elbow singularity floor) so the panel
    /// solves against the identical chain the hub governs. Joint velocity limits
    /// come from the same URDF, so a jog step never demands more per tick than the
    /// hub's chase can follow.
    pub fn from_version(version: HardwareVersion) -> Self {
        let mut left = build_arm(version, Side::Left);
        let mut right = build_arm(version, Side::Right);
        let left_bounds = workspace_aabb(&mut left);
        let right_bounds = workspace_aabb(&mut right);
        Self {
            left_velocity_limits: velocity_limits(version.urdf(), Side::Left),
            right_velocity_limits: velocity_limits(version.urdf(), Side::Right),
            left: Arc::new(Mutex::new(left)),
            right: Arc::new(Mutex::new(right)),
            left_bounds,
            right_bounds,
        }
    }

    /// World-frame end-effector pose of `joints` for `side`.
    pub fn ee_pose_world(&self, side: Side, joints: &[f64; ARM_DOF]) -> Pose {
        let mut model = self.get(side).lock().unwrap_or_else(|p| p.into_inner());
        let base = model.at(joints).ee_pose();
        decompose(&model.world_pose(&base))
    }

    /// World-frame end-effector orientation of `joints` as a quaternion `[x, y, z, w]`,
    /// for the panel's arcball (which composes orientation on quaternions, never euler).
    pub fn ee_quat_world(&self, side: Side, joints: &[f64; ARM_DOF]) -> [f64; 4] {
        let mut model = self.get(side).lock().unwrap_or_else(|p| p.into_inner());
        let base = model.at(joints).ee_pose();
        let q = model.world_pose(&base).rotation;
        [q.i, q.j, q.k, q.w]
    }

    /// Solve inverse kinematics for a world-frame end-effector pose (position in
    /// metres, orientation as a unit quaternion), seeded from `seed` so the branch
    /// nearest the current configuration is chosen. `None` when the pose is
    /// unreachable or admits no in-limit solution. Used to preview an Actions-mode
    /// pose move as joints; the hub re-solves and plans the move itself.
    pub fn solve_ik(
        &self,
        side: Side,
        position: [f64; 3],
        rotation: UnitQuaternion<f64>,
        seed: &[f64; ARM_DOF],
    ) -> Option<[f64; ARM_DOF]> {
        self.solve_ik_with(side, position, rotation, ArmAnglePolicy::FromSeed, seed)
    }

    /// IK holding the arm angle at `psi` (elbow swivel pinned): the null-space jog
    /// re-solves the current end-effector pose at a stepped psi. `None` when no
    /// in-limit solution exists at that psi.
    fn solve_ik_fixed(
        &self,
        side: Side,
        position: [f64; 3],
        rotation: UnitQuaternion<f64>,
        psi: f64,
        seed: &[f64; ARM_DOF],
    ) -> Option<[f64; ARM_DOF]> {
        self.solve_ik_with(side, position, rotation, ArmAnglePolicy::Fixed(psi), seed)
    }

    fn solve_ik_with(
        &self,
        side: Side,
        position: [f64; 3],
        rotation: UnitQuaternion<f64>,
        arm_angle: ArmAnglePolicy,
        seed: &[f64; ARM_DOF],
    ) -> Option<[f64; ARM_DOF]> {
        let model = self.get(side).lock().unwrap_or_else(|p| p.into_inner());
        let world = Isometry3::from_parts(
            Translation3::new(position[0], position[1], position[2]),
            rotation,
        );
        let base_target = model.base_pose(&world);
        model.solve_ik(&base_target, arm_angle, seed).map(|s| s.q)
    }

    /// Current arm angle psi (elbow swivel, rad) of `joints` for `side`; `None` only at
    /// the straight-arm singularity, which the elbow floor keeps the arm off.
    pub fn arm_angle(&self, side: Side, joints: &[f64; ARM_DOF]) -> Option<f64> {
        let model = self.get(side).lock().unwrap_or_else(|p| p.into_inner());
        model.arm_angle(joints)
    }

    /// Scale the joint delta `to - from` so no joint exceeds its velocity budget over
    /// `dt_s`, preserving direction (the same clamp the resolved-rate step applies).
    fn velocity_clamp(
        &self,
        side: Side,
        from: &[f64; ARM_DOF],
        to: &[f64; ARM_DOF],
        dt_s: f64,
    ) -> [f64; ARM_DOF] {
        let budget = self.velocity_limits(side);
        let scale = (0..ARM_DOF)
            .map(|i| {
                let dq = (to[i] - from[i]).abs();
                let cap = budget[i] * dt_s;
                if dq > cap { cap / dq } else { 1.0 }
            })
            .fold(1.0_f64, f64::min);
        std::array::from_fn(|i| from[i] + (to[i] - from[i]) * scale)
    }

    /// One damped-least-squares joint step realizing a world-frame task increment
    /// at the configuration `joints`. The untasked half of the twist is commanded
    /// to zero, so a position step softly holds orientation (and an orientation
    /// step softly holds position); the damping trades that hold away only where
    /// the geometry forces it. The step is scaled so no joint exceeds its URDF
    /// velocity limit over `dt_s`, and clamped into position limits. `None` when
    /// limits pin the step (the free-space envelope boundary).
    fn resolved_rate_step(
        &self,
        side: Side,
        joints: &[f64; ARM_DOF],
        task: RateTask,
        dt_s: f64,
    ) -> Option<[f64; ARM_DOF]> {
        let mut model = self.get(side).lock().unwrap_or_else(|p| p.into_inner());
        let jacobian = model.at(joints).jacobian();
        // The Jacobian lives in the arm base frame; rotate the world-frame task in.
        let to_base = model.base_from_world().rotation;
        let dx_world = match task {
            RateTask::Linear(dx) | RateTask::Angular(dx) => dx,
        };
        let dx = to_base * dx_world;
        let twist = match task {
            RateTask::Linear(_) => Vector6::new(dx.x, dx.y, dx.z, 0.0, 0.0, 0.0),
            RateTask::Angular(_) => Vector6::new(0.0, 0.0, 0.0, dx.x, dx.y, dx.z),
        };
        let mut dq = damped_pseudo_inverse(&jacobian, DLS_LAMBDA) * twist;
        // Velocity-consistent clamping (not accept/reject): scale the whole step
        // down so every joint stays inside its velocity budget for this tick,
        // preserving the step direction. The hub chases under the same limits, so
        // a clamped step is always followable within one tick.
        let budget = self.velocity_limits(side);
        let scale = (0..ARM_DOF)
            .map(|i| {
                let cap = budget[i] * dt_s;
                if dq[i].abs() > cap {
                    cap / dq[i].abs()
                } else {
                    1.0
                }
            })
            .fold(1.0_f64, f64::min);
        dq *= scale;
        let limits = model.limits();
        let q: [f64; ARM_DOF] =
            std::array::from_fn(|i| (joints[i] + dq[i]).clamp(limits[i].lo, limits[i].hi));
        drop(model);
        // Limits may have eaten the step: measure what actually moved along the
        // demanded direction and treat a mostly-pinned step as the boundary.
        let achieved = self.ee_pose_world(side, &q);
        let before = self.ee_pose_world(side, joints);
        let moved = match task {
            RateTask::Linear(_) => Vector3::new(
                achieved[0] - before[0],
                achieved[1] - before[1],
                achieved[2] - before[2],
            ),
            RateTask::Angular(_) => {
                let q_a = UnitQuaternion::from_euler_angles(achieved[3], achieved[4], achieved[5]);
                let q_b = UnitQuaternion::from_euler_angles(before[3], before[4], before[5]);
                let err = q_a * q_b.inverse();
                err.axis()
                    .map(|a| a.into_inner() * err.angle())
                    .unwrap_or_default()
            }
        };
        let progress = moved.dot(&dx_world) / dx_world.norm_squared().max(1e-12);
        (progress > MIN_STEP_PROGRESS).then_some(q)
    }

    fn get(&self, side: Side) -> &Arc<Mutex<Arm>> {
        match side {
            Side::Left => &self.left,
            Side::Right => &self.right,
        }
    }

    fn velocity_limits(&self, side: Side) -> &[f64; ARM_DOF] {
        match side {
            Side::Left => &self.left_velocity_limits,
            Side::Right => &self.right_velocity_limits,
        }
    }

    /// World-frame x/y/z reachable bounds `[[min, max]; 3]` for `side`, so the panel
    /// sizes its position sliders to the arm's actual reach (correct per generation).
    pub fn pos_bounds(&self, side: Side) -> [[f64; 2]; 3] {
        match side {
            Side::Left => self.left_bounds,
            Side::Right => self.right_bounds,
        }
    }
}

/// A world-frame task increment for one resolved-rate step: a linear metre-step or
/// an angular axis-angle step (radians).
enum RateTask {
    Linear(Vector3<f64>),
    Angular(Vector3<f64>),
}

/// Angular jog rate (rad/s). The linear rate is the operator's live EE speed cap
/// (one knob governs both the jog and the hub's enforcement); rotation has no
/// operator knob, so it jogs at this fixed, comfortable rate.
const JOG_ROT_RATE_RAD_S: f64 = 1.5;
/// Arm-angle (elbow swivel) jog rate (rad/s); fixed like the rotation rate, since the
/// null-space motion has no operator speed knob.
const ARM_ANGLE_RATE_RAD_S: f64 = 1.2;
/// Damping for the resolved-rate steps: heavy enough to stay bounded through
/// singular postures, light enough not to visibly lag a jog step.
const DLS_LAMBDA: f64 = 0.05;

/// Per-tick Cartesian step caps, derived from the actual tick period and the
/// operator's live EE speed cap, so a different `command_rate_hz` or a retuned
/// speed knob changes the step size, never the speed.
#[derive(Clone, Copy)]
pub struct JogCaps {
    pub pos_step_m: f64,
    pub rot_step_rad: f64,
    pub arm_angle_step_rad: f64,
    pub dt_s: f64,
}

impl JogCaps {
    /// Caps for one tick of length `dt` seconds at `max_ee_velocity_m_s`. The
    /// speed is the operator's governor knob, validated positive-finite at entry;
    /// the clamp here only bounds a degenerate combination, it is not a tuning.
    pub fn per_tick(dt_s: f64, max_ee_velocity_m_s: f64) -> Self {
        Self {
            pos_step_m: (max_ee_velocity_m_s * dt_s).clamp(1e-5, 0.05),
            rot_step_rad: (JOG_ROT_RATE_RAD_S * dt_s).clamp(1e-4, 0.2),
            arm_angle_step_rad: (ARM_ANGLE_RATE_RAD_S * dt_s).clamp(1e-4, 0.2),
            dt_s,
        }
    }
}

/// Which component a Cartesian jog drives: `Position` tracks x/y/z (holding
/// orientation), `Orientation` tracks the hand frame (holding position), `ArmAngle`
/// swivels the elbow through the null space (holding the whole end-effector pose).
/// Whatever control the operator touches leads.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JogMode {
    Position,
    Orientation,
    ArmAngle,
}

/// The operator's active drive for one arm, reconciled toward once per stream tick.
/// Joint sliders and world-frame controls both arm one of these, so the two spaces
/// share a single "what is this side being driven toward" slot and one advance path;
/// arming either clears the other, since the spaces must not fight.
#[derive(Clone, Copy, Debug)]
pub enum Jog {
    /// Joint sliders: stream these joints straight to the hub, which governs the
    /// ramp under its joint-velocity cap. Reconciles in one step, so the panel's
    /// slider never lags the operator's drag (no node-side interpolation).
    Joints([f64; ARM_DOF]),
    /// World-frame controls: step the joints one resolved-rate increment per tick
    /// toward the target, since the command wire carries only joints and a pose jump
    /// would teleport or branch-flip the arm.
    Cartesian(CartesianJog),
}

/// An armed Cartesian jog: which component leads (`mode`), the desired world-frame
/// pose (used by `Position`/`Orientation`), and the desired arm angle (used by
/// `ArmAngle`); the field the active mode does not use is ignored.
#[derive(Clone, Copy, Debug)]
pub struct CartesianJog {
    pub mode: JogMode,
    pub desired: Pose,
    pub arm_angle: f64,
}

/// One jog tick's outcome.
pub enum JogStep {
    /// The commanded pose reached the desired pose; the jog is complete.
    Converged,
    /// Advanced one capped Cartesian step: the new joint target to stream.
    Stepped([f64; ARM_DOF]),
    /// The next step is pinned by limits or the envelope boundary: hold this tick.
    /// The jog stays armed, so pulling the desired pose back into reach resumes it.
    Blocked,
}

/// Position / angle slack within which a pose component counts as arrived. Half a
/// millimetre and ~0.1 degrees: invisible on the panel, far above FK round-trip noise.
const POS_CONVERGED_M: f64 = 5e-4;
const ROT_CONVERGED_RAD: f64 = 2e-3;
const ARM_ANGLE_CONVERGED_RAD: f64 = 2e-3;
/// A resolved-rate step that achieves less than this fraction of its demanded
/// motion is pinned by joint limits: the envelope boundary in free space.
const MIN_STEP_PROGRESS: f64 = 0.2;

/// Advance one Cartesian jog tick for `side`: step the pose of `joints` one capped
/// increment toward `jog.desired` under `jog.mode`. Steps are velocity-clamped in
/// joint space, so the target walks toward the desired values and stops exactly where
/// reach or limits end.
pub fn jog_tick(
    models: &ArmModels,
    side: Side,
    joints: &[f64; ARM_DOF],
    jog: &CartesianJog,
    caps: JogCaps,
) -> JogStep {
    let current = models.ee_pose_world(side, joints);
    let step = match jog.mode {
        JogMode::Position => {
            let Some(dx) = position_step(&current, &jog.desired, caps.pos_step_m) else {
                return JogStep::Converged;
            };
            models.resolved_rate_step(side, joints, RateTask::Linear(dx), caps.dt_s)
        }
        JogMode::Orientation => {
            let Some(dw) = orientation_step(&current, &jog.desired, caps.rot_step_rad) else {
                return JogStep::Converged;
            };
            models.resolved_rate_step(side, joints, RateTask::Angular(dw), caps.dt_s)
        }
        JogMode::ArmAngle => {
            let Some(psi_cur) = models.arm_angle(side, joints) else {
                return JogStep::Blocked;
            };
            let err = jog.arm_angle - psi_cur;
            if err.abs() < ARM_ANGLE_CONVERGED_RAD {
                return JogStep::Converged;
            }
            let psi_step = psi_cur + err.clamp(-caps.arm_angle_step_rad, caps.arm_angle_step_rad);
            // Hold the current end-effector pose, re-solve at the stepped arm angle: a
            // pure null-space move (the elbow swivels, the hand stays put).
            let position = [current[0], current[1], current[2]];
            let q4 = models.ee_quat_world(side, joints);
            let rotation =
                UnitQuaternion::from_quaternion(Quaternion::new(q4[3], q4[0], q4[1], q4[2]));
            models
                .solve_ik_fixed(side, position, rotation, psi_step, joints)
                .map(|q_new| models.velocity_clamp(side, joints, &q_new, caps.dt_s))
        }
    };
    match step {
        Some(q) => JogStep::Stepped(q),
        None => JogStep::Blocked,
    }
}

/// One tick's world-frame linear step toward the desired position, capped in
/// norm; `None` once within the convergence slack.
fn position_step(current: &Pose, desired: &Pose, cap_m: f64) -> Option<Vector3<f64>> {
    let delta = Vector3::new(
        desired[0] - current[0],
        desired[1] - current[1],
        desired[2] - current[2],
    );
    let dist = delta.norm();
    if dist < POS_CONVERGED_M {
        return None;
    }
    Some(delta * (cap_m.min(dist) / dist))
}

/// One tick's world-frame angular step (axis-angle vector) toward the desired
/// orientation, capped in angle; `None` once within the convergence slack. The
/// error is a quaternion geodesic, so it is chart-independent: no euler seam or
/// gimbal alias can inflate it.
fn orientation_step(current: &Pose, desired: &Pose, cap_rad: f64) -> Option<Vector3<f64>> {
    let q_cur = UnitQuaternion::from_euler_angles(current[3], current[4], current[5]);
    let q_des = UnitQuaternion::from_euler_angles(desired[3], desired[4], desired[5]);
    let err = q_des * q_cur.inverse();
    let angle = err.angle();
    if angle < ROT_CONVERGED_RAD {
        return None;
    }
    let axis = err.axis()?;
    Some(axis.into_inner() * cap_rad.min(angle))
}

/// Build one arm model from the generation's embedded description, with the elbow
/// singularity floor applied (mirrors `openarm_backbone`'s `arm_model`). A bad base
/// link aborts bringup, matching how the hub fails.
fn build_arm(version: HardwareVersion, side: Side) -> Arm {
    let urdf = version.urdf();
    let base = base_link(urdf, side);
    Arm::from_urdf(urdf, &base)
        .unwrap_or_else(|e| panic!("build {} arm model from base '{base}': {e}", side.label()))
        .with_lower_floor(
            version.elbow_joint_index(),
            version.elbow_singularity_floor_rad(),
        )
}

/// The base link where `side`'s SRS chain starts: the parent of its first joint. The
/// joint names (`openarm_{side}_joint1`) are stable across generations, but the base
/// link name is not (v1 `..._link0`, v2 `..._base_link`), so resolve it from the URDF
/// rather than hardcode a per-version name.
fn base_link(urdf: &str, side: Side) -> String {
    let robot = urdf_rs::read_from_string(urdf).expect("bundled URDF must parse");
    let joint1 = format!("openarm_{}_joint1", side.label());
    robot
        .joints
        .iter()
        .find(|j| j.name == joint1)
        .map(|j| j.parent.link.clone())
        .unwrap_or_else(|| panic!("URDF missing joint {joint1}"))
}

/// Per-joint velocity limits (rad/s) for `side`, j1..j7, from the bundled URDF:
/// the same numbers the hub's chase enforces, so commander steps and hub follow
/// capability agree by construction.
fn velocity_limits(urdf: &str, side: Side) -> [f64; ARM_DOF] {
    let robot = urdf_rs::read_from_string(urdf).expect("bundled URDF must parse");
    std::array::from_fn(|i| {
        let name = format!("openarm_{}_joint{}", side.label(), i + 1);
        let joint = robot
            .joints
            .iter()
            .find(|j| j.name == name)
            .unwrap_or_else(|| panic!("URDF missing joint {name}"));
        let v = joint.limit.velocity;
        assert!(
            v.is_finite() && v > 0.0,
            "URDF velocity limit for {name} must be positive"
        );
        v
    })
}

fn decompose(pose: &Isometry3<f64>) -> Pose {
    let t = pose.translation.vector;
    let (roll, pitch, yaw) = pose.rotation.euler_angles();
    [t.x, t.y, t.z, roll, pitch, yaw]
}

/// Euclidean distance (m) between two world-frame points.
pub fn dist3(a: [f64; 3], b: [f64; 3]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

/// Rotation angle (rad) between two orientations given as `[x, y, z, w]` quaternions;
/// robust to the double cover (q and -q are the same rotation, angle 0).
pub fn quat_angle(a: [f64; 4], b: [f64; 4]) -> f64 {
    let qa = UnitQuaternion::from_quaternion(Quaternion::new(a[3], a[0], a[1], a[2]));
    let qb = UnitQuaternion::from_quaternion(Quaternion::new(b[3], b[0], b[1], b[2]));
    qa.angle_to(&qb)
}

/// Grid-sample FK over the joint limits and return the world-frame EE bounding box
/// `[[min, max]; 3]` (x, y, z), padded with a small margin. Runs once per side at
/// construction to size the panel's position sliders to the actual reachable envelope,
/// so the bounds are correct per generation rather than hardcoded.
fn workspace_aabb(arm: &mut Arm) -> [[f64; 2]; 3] {
    const N: usize = 4;
    const MARGIN_M: f64 = 0.02;
    let lims = arm.limits();
    let ranges: [(f64, f64); ARM_DOF] = std::array::from_fn(|i| (lims[i].lo, lims[i].hi));
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for idx in 0..N.pow(ARM_DOF as u32) {
        let mut rem = idx;
        let q: [f64; ARM_DOF] = std::array::from_fn(|j| {
            let step = rem % N;
            rem /= N;
            let t = step as f64 / (N - 1) as f64;
            ranges[j].0 + t * (ranges[j].1 - ranges[j].0)
        });
        let base = arm.at(&q).ee_pose();
        let p = arm.world_pose(&base).translation.vector;
        for k in 0..3 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    std::array::from_fn(|k| [lo[k] - MARGIN_M, hi[k] + MARGIN_M])
}

#[cfg(test)]
mod tests {
    use super::*;

    fn models() -> ArmModels {
        ArmModels::from_version(HardwareVersion::V2)
    }

    // The caps a 100 Hz tick at the sim launcher's 0.5 m/s knob derives.
    fn test_caps() -> JogCaps {
        JogCaps::per_tick(0.01, 0.5)
    }

    fn cart(mode: JogMode, desired: Pose, arm_angle: f64) -> CartesianJog {
        CartesianJog {
            mode,
            desired,
            arm_angle,
        }
    }

    #[test]
    fn workspace_bounds_are_sane_and_contain_home() {
        let m = models();
        for side in [Side::Left, Side::Right] {
            let b = m.pos_bounds(side);
            for [lo, hi] in b {
                assert!(lo < hi, "bound [{lo}, {hi}] must be non-empty");
            }
            // A known-reachable pose (home: zeros, elbow off its floor) sits inside.
            let home = m.ee_pose_world(side, &[0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0]);
            for k in 0..3 {
                assert!(
                    b[k][0] <= home[k] && home[k] <= b[k][1],
                    "home axis {k} = {} outside bounds {:?}",
                    home[k],
                    b[k]
                );
            }
        }
    }

    fn geodesic(a: &Pose, b: &Pose) -> f64 {
        let q_a = UnitQuaternion::from_euler_angles(a[3], a[4], a[5]);
        let q_b = UnitQuaternion::from_euler_angles(b[3], b[4], b[5]);
        (q_a * q_b.inverse()).angle()
    }

    #[test]
    fn caps_scale_with_rate_and_speed() {
        // Same speed at 100 Hz vs 500 Hz: a 5x smaller step, the identical velocity.
        let hz100 = JogCaps::per_tick(0.01, 0.5);
        let hz500 = JogCaps::per_tick(0.002, 0.5);
        assert!((hz100.pos_step_m - 0.005).abs() < 1e-12);
        assert!((hz500.pos_step_m - 0.001).abs() < 1e-12);
        assert!((hz100.pos_step_m / 0.01 - hz500.pos_step_m / 0.002).abs() < 1e-12);
        // Retuning the operator knob rescales the step 1:1.
        assert!((JogCaps::per_tick(0.01, 0.25).pos_step_m - 0.0025).abs() < 1e-12);
        // The angular rate is the fixed jog constant, likewise per-tick.
        assert!((hz100.rot_step_rad - JOG_ROT_RATE_RAD_S * 0.01).abs() < 1e-12);
    }

    #[test]
    fn velocity_limits_load_from_the_urdf() {
        let m = models();
        for side in [Side::Left, Side::Right] {
            let v = m.velocity_limits(side);
            assert!(v.iter().all(|x| x.is_finite() && *x > 0.0));
            // j3/j4 are the slow pair in both generations' datasheets; sanity-pin
            // that we read per-joint values, not one shared number.
            assert!(
                v[2] < v[0],
                "j3 ({}) should be slower than j1 ({})",
                v[2],
                v[0]
            );
        }
    }

    #[test]
    fn steps_never_exceed_joint_velocity_budgets() {
        // Demand an aggressive 5 cm/tick step; the returned joint delta must stay
        // inside every joint's URDF velocity budget for the tick.
        let m = models();
        let q0 = [0.3, 0.1, 0.2, 0.8, 0.3, 0.2, 0.15];
        let p0 = m.ee_pose_world(Side::Left, &q0);
        let caps = JogCaps {
            pos_step_m: 0.05,
            rot_step_rad: 0.2,
            arm_angle_step_rad: 0.2,
            dt_s: 0.01,
        };
        let desired = [p0[0] + 1.0, p0[1], p0[2], p0[3], p0[4], p0[5]];
        if let JogStep::Stepped(q) = jog_tick(
            &m,
            Side::Left,
            &q0,
            &cart(JogMode::Position, desired, 0.0),
            caps,
        ) {
            let budget = m.velocity_limits(Side::Left);
            for i in 0..ARM_DOF {
                let v = (q[i] - q0[i]).abs() / caps.dt_s;
                assert!(
                    v <= budget[i] * 1.0001,
                    "joint {i} at {v:.2} rad/s exceeds its {:.2} rad/s budget",
                    budget[i]
                );
            }
        } else {
            panic!("aggressive but reachable step must advance");
        }
    }

    #[test]
    fn position_jog_from_home_reaches_far_and_holds_orientation() {
        // The original from-home x drag scenario: the jog must walk deep into the
        // workspace with orientation softly held, then hold at the envelope.
        let m = models();
        let start = [0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0];
        let p0 = m.ee_pose_world(Side::Left, &start);
        let desired = [p0[0] + 0.3, p0[1], p0[2], p0[3], p0[4], p0[5]];
        let mut q = start;
        for _ in 0..3000 {
            match jog_tick(
                &m,
                Side::Left,
                &q,
                &cart(JogMode::Position, desired, 0.0),
                test_caps(),
            ) {
                JogStep::Stepped(next) => q = next,
                JogStep::Blocked | JogStep::Converged => break,
            }
        }
        let p = m.ee_pose_world(Side::Left, &q);
        assert!(
            p[0] - p0[0] > 0.15,
            "must cover real distance, got {:.4}",
            p[0] - p0[0]
        );
        assert!(
            geodesic(&p, &p0) < 0.2,
            "orientation must hold softly, drifted {:.4} rad",
            geodesic(&p, &p0)
        );
    }

    #[test]
    fn orientation_jog_turns_the_hand_while_position_holds() {
        let m = models();
        let q0 = [0.3, 0.1, 0.2, 0.8, 0.3, 0.2, 0.15];
        let p0 = m.ee_pose_world(Side::Left, &q0);
        let desired = [p0[0], p0[1], p0[2], p0[3], p0[4], p0[5] + 0.3];
        let mut q = q0;
        for _ in 0..200 {
            match jog_tick(
                &m,
                Side::Left,
                &q,
                &cart(JogMode::Orientation, desired, 0.0),
                test_caps(),
            ) {
                JogStep::Stepped(next) => q = next,
                JogStep::Converged | JogStep::Blocked => break,
            }
        }
        let p = m.ee_pose_world(Side::Left, &q);
        assert!(geodesic(&p, &p0) > 0.15, "hand must actually turn");
        let drift =
            ((p[0] - p0[0]).powi(2) + (p[1] - p0[1]).powi(2) + (p[2] - p0[2]).powi(2)).sqrt();
        assert!(
            drift < 0.05,
            "position drift under an orientation jog stays small, got {drift:.4}"
        );
    }

    #[test]
    fn jog_converges_on_the_current_pose() {
        let m = models();
        let q = [0.3, 0.1, 0.2, 0.8, 0.3, 0.2, 0.15];
        let p = m.ee_pose_world(Side::Left, &q);
        for mode in [JogMode::Position, JogMode::Orientation] {
            assert!(matches!(
                jog_tick(&m, Side::Left, &q, &cart(mode, p, 0.0), test_caps()),
                JogStep::Converged
            ));
        }
    }

    #[test]
    fn arm_angle_jog_swivels_the_elbow_holding_the_ee_pose() {
        // Driving the arm angle moves psi toward the target while the end-effector
        // pose stays put: a pure null-space move (elbow swivels, hand holds).
        let m = models();
        let q0 = [0.3, 0.1, 0.2, 0.8, 0.3, 0.2, 0.15];
        let p0 = m.ee_pose_world(Side::Left, &q0);
        let psi0 = m
            .arm_angle(Side::Left, &q0)
            .expect("arm angle is defined off the straight-arm floor");
        let target_psi = psi0 + 0.3;
        let mut q = q0;
        for _ in 0..500 {
            match jog_tick(
                &m,
                Side::Left,
                &q,
                &cart(JogMode::ArmAngle, p0, target_psi),
                test_caps(),
            ) {
                JogStep::Stepped(next) => q = next,
                JogStep::Converged | JogStep::Blocked => break,
            }
        }
        let psi = m.arm_angle(Side::Left, &q).unwrap();
        let p = m.ee_pose_world(Side::Left, &q);
        assert!(
            (psi - target_psi).abs() < 0.05,
            "psi should reach the target, got {psi:.3} vs {target_psi:.3}"
        );
        let pos_drift =
            ((p[0] - p0[0]).powi(2) + (p[1] - p0[1]).powi(2) + (p[2] - p0[2]).powi(2)).sqrt();
        assert!(
            pos_drift < 0.01,
            "EE position must hold under an arm-angle jog, drifted {pos_drift:.4}"
        );
        assert!(
            geodesic(&p, &p0) < 0.05,
            "EE orientation must hold, drifted {:.4} rad",
            geodesic(&p, &p0)
        );
    }

    #[test]
    fn jog_blocks_at_the_envelope_and_never_converges_short() {
        // Far past reach: the jog must end Blocked (never Converged) and every
        // step along the way must be velocity-bounded.
        let m = models();
        let mut q = [0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0];
        let p0 = m.ee_pose_world(Side::Left, &q);
        let desired = [p0[0] + 2.0, p0[1], p0[2], p0[3], p0[4], p0[5]];
        let budget = m.velocity_limits(Side::Left);
        for _ in 0..5000 {
            match jog_tick(
                &m,
                Side::Left,
                &q,
                &cart(JogMode::Position, desired, 0.0),
                test_caps(),
            ) {
                JogStep::Stepped(next) => {
                    for i in 0..ARM_DOF {
                        assert!((next[i] - q[i]).abs() / 0.01 <= budget[i] * 1.0001);
                    }
                    q = next;
                }
                JogStep::Blocked => return,
                JogStep::Converged => panic!("must not converge on an unreachable pose"),
            }
        }
        panic!("jog neither blocked nor converged within 5000 ticks");
    }
}
