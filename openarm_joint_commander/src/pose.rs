//! Cartesian pose <-> joints glue for the panel. The joint vector stays the single
//! source of truth: the UI reads a pose off forward kinematics and writes joints
//! back through inverse kinematics, so a pose is only ever an input/display lens,
//! never stored state.
//!
//! Poses are in the world frame (matching the hub's `move_arm` action) and
//! orientation is exposed as intrinsic-XYZ roll/pitch/yaw via nalgebra's
//! `euler_angles` / `from_euler_angles` round-trip; the canonical type carried into
//! IK is a quaternion `Isometry3`, re-derived each read so the RPY form never
//! accumulates.

use std::sync::{Arc, Mutex};

use openarm_description::HardwareVersion;
use srs_model::nalgebra::{Isometry3, Translation3, UnitQuaternion};
use srs_model::{Arm, ArmAnglePolicy};

use crate::state::{ARM_DOF, Side};

/// A world-frame end-effector pose as `[x, y, z, roll, pitch, yaw]` (metres,
/// radians): the form the panel edits and displays.
pub type Pose = [f64; 6];

/// The two per-side arm models (FK/IK/limits) behind mutexes.
///
/// Each lock is held only for a single synchronous FK or IK call and never across
/// an `.await`. The only nesting is snapshot building, which holds the [`UiState`]
/// lock and then briefly takes a model lock; no path takes the locks in the reverse
/// order, so they cannot deadlock.
///
/// [`UiState`]: crate::state::UiState
#[derive(Clone)]
pub struct ArmModels {
    left: Arc<Mutex<Arm>>,
    right: Arc<Mutex<Arm>>,
}

impl ArmModels {
    /// Build both arm models from the generation's embedded description, mirroring
    /// the hub's `arm_model` (same URDF, same elbow singularity floor) so the panel
    /// solves against the identical chain the hub governs.
    pub fn from_version(version: HardwareVersion) -> Self {
        Self {
            left: Arc::new(Mutex::new(build_arm(version, Side::Left))),
            right: Arc::new(Mutex::new(build_arm(version, Side::Right))),
        }
    }

    /// World-frame end-effector pose of `joints` for `side`.
    pub fn ee_pose_world(&self, side: Side, joints: &[f64; ARM_DOF]) -> Pose {
        let mut model = self.get(side).lock().unwrap_or_else(|p| p.into_inner());
        let base = model.at(joints).ee_pose();
        decompose(&model.world_pose(&base))
    }

    /// Joints achieving world-frame `pose` for `side`, seeded from `seed` for
    /// arm-angle continuity. `None` when the pose is unreachable, singular, or admits
    /// no in-limit solution.
    pub fn solve_pose_ik(
        &self,
        side: Side,
        pose: &Pose,
        seed: &[f64; ARM_DOF],
    ) -> Option<[f64; ARM_DOF]> {
        let model = self.get(side).lock().unwrap_or_else(|p| p.into_inner());
        let target = model.base_pose(&compose(pose));
        model
            .solve_ik(&target, ArmAnglePolicy::FromSeed, seed)
            .map(|sol| sol.q)
    }

    fn get(&self, side: Side) -> &Arc<Mutex<Arm>> {
        match side {
            Side::Left => &self.left,
            Side::Right => &self.right,
        }
    }
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

fn decompose(pose: &Isometry3<f64>) -> Pose {
    let t = pose.translation.vector;
    let (roll, pitch, yaw) = pose.rotation.euler_angles();
    [t.x, t.y, t.z, roll, pitch, yaw]
}

fn compose(pose: &Pose) -> Isometry3<f64> {
    let [x, y, z, roll, pitch, yaw] = *pose;
    Isometry3::from_parts(
        Translation3::new(x, y, z),
        UnitQuaternion::from_euler_angles(roll, pitch, yaw),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn models() -> ArmModels {
        let version: HardwareVersion = "v1".parse().expect("v1 parses");
        ArmModels::from_version(version)
    }

    #[test]
    fn fk_then_ik_recovers_the_configuration() {
        // A hand-picked in-limit, non-singular config (values inside both arms'
        // mirrored ranges): FK to a world pose, then IK back seeded from the same
        // config must land on it. Seeded IK keeps the arm angle, so the branch is
        // unique. (v2 FK/IK is exercised end-to-end by the ui pose_to_joints test.)
        let m = models();
        let q = [0.3, 0.1, 0.2, 0.8, 0.3, 0.2, 0.15];
        for side in [Side::Left, Side::Right] {
            let pose = m.ee_pose_world(side, &q);
            let solved = m
                .solve_pose_ik(side, &pose, &q)
                .expect("reachable pose solves");
            for (i, (s, e)) in solved.iter().zip(q.iter()).enumerate() {
                assert!((s - e).abs() < 1e-6, "{side:?} joint {i}: {s} vs {e}");
            }
        }
    }

    #[test]
    fn rpy_round_trips_through_a_quaternion() {
        let pose = [0.1, -0.2, 0.3, 0.4, -0.5, 0.6];
        let back = decompose(&compose(&pose));
        for (i, (b, p)) in back.iter().zip(pose.iter()).enumerate() {
            assert!((b - p).abs() < 1e-12, "component {i}: {b} vs {p}");
        }
    }

    #[test]
    fn unreachable_pose_yields_no_solution() {
        let m = models();
        let seed = [0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0];
        // Ten metres out is well beyond the arm's reach.
        let pose = [10.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        assert!(m.solve_pose_ik(Side::Left, &pose, &seed).is_none());
    }
}
