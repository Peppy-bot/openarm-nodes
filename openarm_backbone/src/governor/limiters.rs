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

use super::allowance::Allowance;
use super::sense::Sensed;
use super::{Step, is_left_dof};

/// One independent restriction on a step.
pub(super) trait Limiter {
    /// Reported as the binding mechanism when this limiter is the tightest.
    fn name(&self) -> &'static str;

    fn allow(&self, step: &Step, sensed: &Sensed) -> Allowance;
}

/// Defense in depth against tracking error: the barrier shapes only the
/// commanded stream and cannot see how well the arms follow it, so this holds
/// closing motion whenever the *real* clearance has closed past the tripwire
/// floor, until it recovers.
///
/// The gate is per side (an arm and its gripper opening together): one
/// operator's closing push must not trap the other side's escape. When the
/// joint candidate closes, each side is re-judged with the other held, and any
/// sub-motion that does not worsen the commanded clearance stays free.
///
/// Whether the tripwire is armed at all, and the clearances this reads, are
/// decided in [`super::sense`]; by the time it limits, the decision is already data.
pub(super) struct MeasuredTripwire;

impl Limiter for MeasuredTripwire {
    fn name(&self) -> &'static str {
        "measured-tripwire"
    }

    fn allow(&self, _step: &Step, sensed: &Sensed) -> Allowance {
        let Some(tripwire) = &sensed.tripwire else {
            return Allowance::FULL;
        };
        // Judged against the held setpoint's own clearance, in the same
        // (commanded) space: comparing across spaces would pass every closing
        // command under a systematic tracking offset and freeze every escape
        // under the opposite one.
        let opens = |d: Option<f64>| d.filter(|d| *d >= sensed.d_prev);
        if tripwire.d_cand >= sensed.d_prev {
            return Allowance::FULL;
        }
        // The joint candidate closes, so free at most one side: the two can
        // each open alone yet still converge on the same gap together.
        match (opens(tripwire.d_solo.left), opens(tripwire.d_solo.right)) {
            (Some(left), Some(right)) => Allowance::gate(|i| is_left_dof(i) == (left >= right)),
            (Some(_), None) => Allowance::gate(is_left_dof),
            (None, Some(_)) => Allowance::gate(|i| !is_left_dof(i)),
            (None, None) => Allowance::FREEZE,
        }
    }
}
