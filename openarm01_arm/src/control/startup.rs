//! The startup state: one gentle joint move from wherever the arm powered on to
//! a fixed non-singular ready pose, before any goal is admitted. Entered only
//! when no producer is streaming at boot; when one is, the control loop enters
//! `Follow` directly and tracks the live stream (velocity-capped) from the
//! power-on pose instead.

use std::sync::atomic::Ordering;

use srs_model::Limit;
use tracing::info;

use super::follow::Follow;
use super::{ControlConfig, Mode, TickIo, command, fmt_joints};
use crate::{ARM_DOF, JointVec};
use crate::trajectory::JointTrajectory;

/// Non-singular rest configuration the arm homes to when no producer is
/// streaming at boot. The arm powers off wherever it hung (often a near-straight
/// elbow, on the straight-arm singularity), so this brings it to a known,
/// well-conditioned pose. The elbow (J4) is bent a hair above its URDF lower
/// limit; every other joint rests at 0.
const READY_POSE: JointVec = [0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0];

/// Requested duration (s) of the startup move, floored at the joint velocity
/// limits like any joint move (so even a far home is a gentle, planned ease).
const READY_MOVE_DURATION_S: f64 = 3.0;

/// Assert that [`READY_POSE`] lies within `limits`. Called from `main` during
/// bringup so a misconfigured constant fails the process before any hardware is
/// touched, never from inside the spawned control task.
pub(crate) fn assert_ready_pose_in_limits(limits: &[Limit; ARM_DOF]) {
    assert!(
        READY_POSE.iter().zip(limits).all(|(&q, l)| l.contains(q)),
        "READY_POSE outside joint limits: {READY_POSE:?}",
    );
}

pub(super) struct StartupMove {
    trajectory: JointTrajectory,
}

impl StartupMove {
    /// Plan the move from the measured power-on configuration `q0` to the ready
    /// pose: eases out rather than lunging, and tolerates powering off below the
    /// elbow floor (it moves up into range).
    pub(super) fn new(q0: JointVec, cfg: &ControlConfig) -> Self {
        info!("startup: no stream present, moving to ready pose {} (from {})", fmt_joints(&READY_POSE), fmt_joints(&q0));
        Self {
            trajectory: JointTrajectory::new(q0, READY_POSE, cfg.max_joint_velocity_rad_s, READY_MOVE_DURATION_S),
        }
    }

    /// Command the trajectory sample; on completion release the `busy` flag held
    /// since spawn (goals were rejected as busy until now) and enter `Follow` at
    /// the ready pose.
    pub(super) fn tick(self, io: &TickIo<'_>) -> Mode {
        let (q_des, dq_des) = self.trajectory.sample(io.now);
        command(io, &q_des, &dq_des);
        if self.trajectory.is_complete(io.now) {
            io.busy.store(false, Ordering::Release);
            return Mode::Follow(Follow::idle(READY_POSE));
        }
        Mode::Startup(self)
    }
}
