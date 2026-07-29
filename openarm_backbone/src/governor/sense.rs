//! The one place the collision model is queried each tick.
//!
//! Every clearance and gradient the pipeline needs is sampled here into an
//! immutable [`Sensed`], before any limiter runs. The model rewrites its
//! finger placement on every query ([`BimanualCollisionModel::set_gripper_openings`]),
//! so a design where limiters query it directly would make their order
//! load-bearing: whoever asks last leaves the model configured for whoever asks
//! next, and the gradient becomes one taken at the wrong finger placement.
//! Sampling once removes that coupling by construction.
//!
//! Each mechanism owns its own sampling; this module only sequences them, so
//! that the model is read here and nowhere else.

use bimanual_collision_model::CollisionError;
use tracing::error;

use super::limiters::Tripwire;
use super::{ARM_DOF, DUAL_DOF, GOV_DOF, GovState, Governor, LEFT_OPENING};
use super::{NearestPair, RIGHT_OPENING};

/// Everything the limiters and the barrier read, sampled once at `prev`.
pub(super) struct Sensed {
    /// This tick's period (s). Carried here so a limiter is a pure function of
    /// the step and the snapshot alone.
    pub dt: f64,
    /// Signed surface clearance at `prev` (m; negative is penetration).
    pub d_prev: f64,
    /// `d(clearance)/d(dof)` at `prev`. `None` in deep penetration, where the
    /// witness points coincide and no separating direction exists: the barrier
    /// stands down there and the floor scan alone guards the step, so the
    /// operator can still drive out of the contact.
    pub grad: Option<[f64; GOV_DOF]>,
    /// The nearest checked pair at `prev`, for the log line and the readout.
    pub pair: NearestPair,
    /// `Some` only while the measured-state tripwire is latched.
    pub tripwire: Option<Tripwire>,
}

impl Governor {
    /// Sample the model at `prev` and, while the tripwire is armed, at the
    /// commanded configurations it needs. `None` when the clearance at `prev`
    /// cannot be obtained at all, which the caller turns into a hold.
    pub(super) fn sense(
        &mut self,
        dt: f64,
        prev: &GovState,
        cand: &GovState,
        measured: &GovState,
    ) -> Option<Sensed> {
        // The tripwire samples the measured state first; every later query in
        // this function re-places the fingers, so nothing may read the model
        // between these calls and their use.
        let tripwire = self.sense_tripwire(prev, cand, measured);

        self.model
            .set_gripper_openings(prev.openings.left, prev.openings.right);
        match self
            .model
            .distance_gradient(&prev.arms.left, &prev.arms.right)
        {
            Ok(g) => {
                let mut grad = [0.0; GOV_DOF];
                grad[..ARM_DOF].copy_from_slice(&g.grad_left);
                grad[ARM_DOF..DUAL_DOF].copy_from_slice(&g.grad_right);
                grad[LEFT_OPENING] = g.grad_openings[0];
                grad[RIGHT_OPENING] = g.grad_openings[1];
                Some(Sensed {
                    dt,
                    d_prev: g.proximity.distance,
                    grad: Some(grad),
                    pair: NearestPair {
                        distance: g.proximity.distance,
                        link_a: g.proximity.link_a.to_string(),
                        link_b: g.proximity.link_b.to_string(),
                    },
                    tripwire,
                })
            }
            Err(CollisionError::WitnessesCoincide { .. }) => {
                // Deep penetration. There is no gradient to steer on, but there
                // is still a distance to hold, so report the reading without one
                // rather than failing: freezing here would trap the operator
                // inside the collision.
                let pair = self.proximity(prev)?;
                Some(Sensed {
                    dt,
                    d_prev: pair.distance,
                    grad: None,
                    pair,
                    tripwire,
                })
            }
            Err(e) => {
                // NonFinite / NoPairs cannot arise from a finite, governed prev
                // with pairs configured; treat as a fault and hold rather than
                // steer on it.
                error!("collision: distance_gradient: {e}; holding");
                None
            }
        }
    }
}
