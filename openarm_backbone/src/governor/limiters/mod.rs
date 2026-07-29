//! The independent restrictions that limit a step.
//!
//! Each limiter is a pure function of the proposed [`Step`] and the tick's
//! immutable [`Sensed`] snapshot, returning an [`Allowance`]. None of them sees
//! another's output and none of them touches the collision model, so they may
//! be evaluated in any order and combined by taking the most restrictive
//! fraction per DOF.
//!
//! A restriction that cannot be written as a per-DOF fraction does not belong
//! here. The closing-velocity barrier is the one such case in the governor: it
//! is a directional projection, and it lives in [`super::barrier`].

pub(in crate::governor) mod measured_tripwire;

pub(in crate::governor) use measured_tripwire::{MeasuredTripwire, Tripwire};

use super::allowance::Allowance;
use super::sense::Sensed;
use super::{GOV_DOF, Step};

/// One independent restriction on a step.
pub(super) trait Limiter {
    /// Reported as the binding mechanism when this limiter is the tightest.
    fn name(&self) -> &'static str;

    fn allow(&self, step: &Step, sensed: &Sensed) -> Allowance;
}

/// The per-tick travel each governed DOF may take: the arm joint speed cap for
/// a joint, the opening rate for a finger.
///
/// The chase upstream already limits every DOF, so on a healthy tick this is
/// inert. It is here because the floor scan sizes its probe count from the same
/// bound: distance is not monotone along a segment, so a step past the bound
/// under-resolves the scan and can step over a pocket and miss a breach.
/// Clamping keeps that precondition true by construction, and keeps the robot
/// under control when something upstream is wrong.
pub(super) struct DofSpeed {
    /// Largest excursion each DOF may take this tick.
    pub max_step: [f64; GOV_DOF],
}

impl Limiter for DofSpeed {
    fn name(&self) -> &'static str {
        "dof-speed"
    }

    fn allow(&self, step: &Step, _sensed: &Sensed) -> Allowance {
        Allowance::new(std::array::from_fn(|i| {
            let excursion = (step.target[i] - step.prev[i]).abs();
            // `max_step` is positive (both rate bounds are validated above
            // zero), so a motionless DOF takes this branch and never divides.
            if excursion <= self.max_step[i] {
                1.0
            } else {
                self.max_step[i] / excursion
            }
        }))
    }
}
