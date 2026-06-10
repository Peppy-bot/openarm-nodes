//! The startup state: one gentle joint move from wherever the arm powered on to
//! a fixed non-singular ready pose, before any goal is admitted.

use std::sync::atomic::Ordering;

use tracing::info;

use super::{ControlConfig, Mode, TickIo, command, fmt_joints};
use crate::JointVec;
use crate::trajectory::JointTrajectory;

/// Non-singular rest configuration the arm moves to once on startup, before it
/// admits any goal. The arm powers off wherever it hung (often a near-straight
/// elbow, on the straight-arm singularity), so this brings it to a known,
/// well-conditioned pose from which Cartesian control is well behaved. The elbow
/// (J4) is bent a hair above its URDF lower limit; every other joint rests at 0.
const READY_POSE: JointVec = [0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0];

/// Requested duration (s) of the startup move to [`READY_POSE`], floored at the
/// joint velocity limits like any joint move.
const READY_MOVE_DURATION_S: f64 = 3.0;

pub(super) struct StartupMove {
    trajectory: JointTrajectory,
}

impl StartupMove {
    /// Plan the move from the measured power-on configuration `q0`: starting from
    /// the measured state, it eases out rather than lunging, and tolerates
    /// powering off below the elbow floor (it moves up into range).
    pub(super) fn new(q0: JointVec, cfg: &ControlConfig) -> Self {
        assert!(
            READY_POSE.iter().zip(&cfg.limits).all(|(&q, l)| l.contains(q)),
            "READY_POSE outside joint limits: {READY_POSE:?}",
        );
        info!("startup: moving to ready pose {} (from {})", fmt_joints(&READY_POSE), fmt_joints(&q0));
        Self {
            trajectory: JointTrajectory::new(q0, READY_POSE, cfg.max_joint_velocity_rad_s, READY_MOVE_DURATION_S),
        }
    }

    /// Command the trajectory sample; on completion release the `busy` flag held
    /// since spawn (goals were rejected as busy until now) and hold the ready pose.
    pub(super) fn tick(self, io: &TickIo<'_>) -> Mode {
        let (q_des, dq_des) = self.trajectory.sample(io.now);
        command(io, &q_des, &dq_des);
        if self.trajectory.is_complete(io.now) {
            io.busy.store(false, Ordering::Release);
            return Mode::Hold { setpoint: READY_POSE };
        }
        Mode::Startup(self)
    }
}
