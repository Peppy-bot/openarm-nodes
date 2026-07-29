//! The one place the collision model is queried each tick.
//!
//! Every clearance and gradient the pipeline needs is sampled here into an
//! immutable [`Sensed`], before any limiter runs. The model rewrites its
//! finger placement on every query ([`BimanualCollisionModel::set_gripper_openings`]),
//! so a design where limiters query it directly makes their order load-bearing:
//! whoever asks last leaves the model configured for whoever asks next, and the
//! gradient silently becomes one taken at the wrong finger placement. Sampling
//! once removes that coupling by construction rather than by comment.
//!
//! This is also the only stage that carries state across ticks: the
//! measured-state tripwire's hysteresis latch.

use bimanual_collision_model::CollisionError;
use tracing::{error, info, warn};

use super::{ARM_DOF, DUAL_DOF, GOV_DOF, GovState, Governor, LEFT_OPENING, NearestPair};
use super::{ArmPair, RIGHT_OPENING, concat};

/// The measured-state tripwire trips when the real clearance drops below this
/// fraction of `d_stop`, and releases only once it recovers past the full
/// `d_stop` (hysteresis). Sitting below the commanded floor leaves headroom for
/// tracking jitter at the wall, where the barrier parks the commanded clearance
/// at `d_stop`, so only a genuine divergence trips it. A module constant (not a
/// node parameter); promote it to a parameter when tuning on hardware.
pub(super) const MONITOR_TRIP_FRACTION: f64 = 0.5;
// The trip floor must sit strictly inside (0, d_stop) or the hysteresis band
// [trip_floor, d_stop) collapses and the latch logic degrades silently.
const _: () = assert!(MONITOR_TRIP_FRACTION > 0.0 && MONITOR_TRIP_FRACTION < 1.0);

/// Everything the limiters and the barrier read, sampled once at `prev`.
pub(super) struct Sensed {
    /// Signed surface clearance at `prev` (m; negative is penetration).
    pub d_prev: f64,
    /// `d(clearance)/d(dof)` at `prev`. `None` in deep penetration, where the
    /// witness points coincide and no separating direction exists: the barrier
    /// stands down there and the floor scan alone guards the step, so the
    /// operator can still drive out instead of being frozen inside the contact.
    pub grad: Option<[f64; GOV_DOF]>,
    /// The nearest checked pair at `prev`, for the log line and the readout.
    pub pair: NearestPair,
    /// `Some` only while the measured-state tripwire is latched.
    pub tripwire: Option<Tripwire>,
}

/// Commanded-space clearances the measured-state tripwire needs to decide which
/// side is making the real breach worse. Sampled only while the latch is armed,
/// so an untripped tick costs one measured query and nothing else.
///
/// Judging in the commanded space (against `Sensed::d_prev`, the held
/// setpoint's own clearance) rather than against the measured clearance is what
/// keeps a systematic tracking offset from either waving every closing command
/// through or deadlocking every escape.
pub(super) struct Tripwire {
    /// Clearance at the full candidate.
    pub d_cand: f64,
    /// Per side, the clearance of that side moving alone with the other held,
    /// and `None` when the side does not move or its query failed. A failed
    /// query reads as "does not open", which holds rather than frees.
    pub d_solo: ArmPair<Option<f64>>,
}

impl Governor {
    /// Sample the model at `prev` and, while the tripwire is armed, at the
    /// commanded configurations it needs. `None` when the clearance at `prev`
    /// cannot be obtained at all, which the caller turns into a hold.
    pub(super) fn sense(
        &mut self,
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

    /// Update the hysteresis latch from the measured clearance and, while it is
    /// armed, sample the commanded-space clearances the gate needs.
    ///
    /// A failed measured query leaves the latch untouched and returns `None`
    /// (defer to the commanded barrier), so a bad measurement can neither block
    /// separation nor latch a hold.
    fn sense_tripwire(
        &mut self,
        prev: &GovState,
        cand: &GovState,
        measured: &GovState,
    ) -> Option<Tripwire> {
        let d_measured = self.distance_at(&concat(measured))?;
        let trip_floor = MONITOR_TRIP_FRACTION * self.d_stop;
        let threshold = if self.monitor_tripped {
            self.d_stop
        } else {
            trip_floor
        };
        let breached = d_measured < threshold;
        if breached != self.monitor_tripped {
            if breached {
                warn!(
                    "collision MONITOR: measured clearance past {trip_floor:+.4} m, blocking approach (separation still allowed)"
                );
            } else {
                info!("collision MONITOR: measured clearance recovered past d_stop, resuming");
            }
            self.monitor_tripped = breached;
        }
        if !breached {
            return None;
        }

        let d_cand = self.distance_at(&concat(cand))?;
        let solo = |side_is_left: bool| GovState {
            arms: ArmPair::new(
                if side_is_left { cand } else { prev }.arms.left,
                if side_is_left { prev } else { cand }.arms.right,
            ),
            openings: ArmPair::new(
                if side_is_left { cand } else { prev }.openings.left,
                if side_is_left { prev } else { cand }.openings.right,
            ),
        };
        let moves = |side_is_left: bool| {
            let (c, p) = (cand, prev);
            if side_is_left {
                c.arms.left != p.arms.left || c.openings.left != p.openings.left
            } else {
                c.arms.right != p.arms.right || c.openings.right != p.openings.right
            }
        };
        let sample = |g: &mut Self, side_is_left: bool| -> Option<f64> {
            moves(side_is_left)
                .then(|| g.distance_at(&concat(&solo(side_is_left))))
                .flatten()
        };
        let d_solo = ArmPair::new(sample(self, true), sample(self, false));
        Some(Tripwire { d_cand, d_solo })
    }
}
