//! The two collision stages that shape a step: the closing-velocity barrier and
//! the exact floor scan.
//!
//! These are the only stages that are not independent per-DOF limits, and they
//! run in this order for a reason.
//!
//! [`Governor::project_closing`] is a *directional* transform: it removes just
//! enough of the gap-closing component that the clearance loses no more than
//! `allowed_closing(d) * dt`, and leaves tangential and separating motion at
//! full speed. That cannot be expressed as a per-DOF fraction, so it cannot be
//! a limiter. It runs after the limiters so its guarantee holds on the step that is
//! actually published rather than on an intermediate one.
//!
//! [`Governor::clip_to_floor`] is the exact backstop, and it runs last. Surface
//! distance is not monotone along a joint-space segment, so no upstream stage
//! can prove the realized path stays clear; this one walks the segment and
//! retracts to the furthest point that does. It is what makes the limiters'
//! order-independence safe rather than merely convenient.

use super::{
    APPROACH_VELOCITY_AT_SAFE_M_S, Clip, DUAL_DOF, FLOOR_BISECT_ITERS, GOV_DOF, Governor,
    MAX_PROBE_ARC_RAD, MAX_PROBE_OPENING_FRAC, MIN_GRADIENT_NORM_SQ, RECOVERY_LOSS_M_PER_S,
    SEGMENT_SAMPLES_MIN, dot, is_left_dof, split,
};

impl Governor {
    /// The clearance this tick's governed step must not drop below.
    ///
    /// `d_stop` on an approach, or the current clearance if the arms are inside
    /// it, so closing further is refused. Once the bodies actually overlap the
    /// rule relaxes to a bounded rate of loss: escaping an interpenetration
    /// routinely sweeps deeper before it separates, and a floor that forbids any
    /// loss refuses that whole segment every tick. See [`RECOVERY_LOSS_M_PER_S`].
    fn step_floor(&self, d_now: f64, dt: f64) -> f64 {
        if d_now >= 0.0 {
            return d_now.min(self.d_stop);
        }
        d_now - RECOVERY_LOSS_M_PER_S * dt
    }

    /// The closing-velocity barrier: scale back only the gap-closing component of the
    /// step (minimum-norm, along the distance gradient) so the clearance loses no more
    /// than `allowed_closing(d_now) * dt`, then clamp each DOF's step into
    /// `[0, commanded]` so the barrier can only slow motion, never add motion a DOF
    /// was not commanded nor reverse one it was. Returns the governed configuration
    /// and whether it limited the step.
    pub(super) fn project_closing(
        &self,
        prev_q: &[f64; GOV_DOF],
        cand_q: &[f64; GOV_DOF],
        grad: &[f64; GOV_DOF],
        d_now: f64,
        dt: f64,
    ) -> ([f64; GOV_DOF], bool) {
        let step: [f64; GOV_DOF] = std::array::from_fn(|i| cand_q[i] - prev_q[i]);
        // Predicted change in clearance over this tick if the full step is taken, and
        // the most clearance the barrier permits losing.
        let predicted_delta_d = dot(grad, &step);
        let max_loss = self.allowed_closing(d_now) * dt;
        let norm_sq = dot(grad, grad);
        let (projected, limited) =
            if predicted_delta_d >= -max_loss || norm_sq <= MIN_GRADIENT_NORM_SQ {
                (*cand_q, false)
            } else {
                // Subtract just enough of the closing component (along the gradient) to
                // land on the barrier `grad . step = -max_loss`.
                let excess = (predicted_delta_d + max_loss) / norm_sq;
                (
                    std::array::from_fn(|i| prev_q[i] + step[i] - excess * grad[i]),
                    true,
                )
            };
        if !limited {
            // Unrestricted motion must carry the commanded value itself: the
            // clamp below reconstructs each DOF from `prev + delta`, which is not
            // bit-identical to the candidate.
            return (*cand_q, false);
        }
        // The minimum-norm correction spreads the closing reduction along the
        // gradient, which can jog a DOF the operator did not drive or reverse one
        // they did. Clamp each DOF's governed step into [0, commanded step]: a held
        // DOF stays put, none reverses, separating motion is untouched.
        let governed = std::array::from_fn(|i| {
            prev_q[i] + (projected[i] - prev_q[i]).clamp(step[i].min(0.0), step[i].max(0.0))
        });
        (governed, true)
    }

    /// Permitted closing speed (m/s) at signed surface distance `d`: zero at or
    /// under `d_stop`, the full approach at or over `d_safe`, linear between.
    ///
    /// Inside an actual overlap this is a recovery budget rather than an
    /// approach one. Leaving it at zero there makes the projection itself the
    /// trap: an escape whose first move goes deeper is a closing step, so it
    /// would be projected away and the arms could never leave the collision.
    fn allowed_closing(&self, d: f64) -> f64 {
        if d < 0.0 {
            RECOVERY_LOSS_M_PER_S
        } else if d <= self.d_stop {
            0.0
        } else if d >= self.d_safe {
            APPROACH_VELOCITY_AT_SAFE_M_S
        } else {
            APPROACH_VELOCITY_AT_SAFE_M_S * (d - self.d_stop) / (self.d_safe - self.d_stop)
        }
    }

    /// The per-DOF hold mask for [`clip_to_floor`]. When exactly one side's own
    /// motion (the other held at `prev`) opens the clearance, that separating
    /// side is held at `target` while the floor scan clips the other, so the
    /// approaching side's clip cannot drag the separating side's escape back
    /// with it: the shared segment parameter would otherwise retract both to the
    /// same point. Two operators can then retreat independently even while one
    /// pushes in. When both sides approach (nothing separates), or both separate
    /// alone yet may jointly close, nothing is held and the shared-segment
    /// backstop governs both.
    ///
    /// A hold pins that side at `target` for the whole scan, so the scan never
    /// probes the held side's own sweep: the hold is granted only when that solo
    /// sweep itself scans clear of the floor (the endpoint alone can step over a
    /// pocket), which also keeps the scan's clear-start precondition (the held
    /// base is the solo config, at or above `d_prev`). A side that does not move
    /// is never held: pinning it would be a no-op that only disables the scan's
    /// Lipschitz skip.
    fn separating_hold(
        &mut self,
        prev: &[f64; GOV_DOF],
        target: &[f64; GOV_DOF],
        d_prev: f64,
        dt: f64,
    ) -> [bool; GOV_DOF] {
        let side_dofs = |left: bool| (0..GOV_DOF).filter(move |&i| is_left_dof(i) == left);
        let solo = |left: bool| -> [f64; GOV_DOF] {
            let mut q = *prev;
            for i in side_dofs(left) {
                q[i] = target[i];
            }
            q
        };
        let moves = |left: bool| side_dofs(left).any(|i| target[i] != prev[i]);
        let no_hold = [false; GOV_DOF];
        let separates = |g: &mut Self, q: &[f64; GOV_DOF]| {
            g.distance_at(q).is_some_and(|d| d >= d_prev)
                && matches!(g.scan_to_floor(prev, q, &no_hold, d_prev, dt), Clip::Clear)
        };
        let (solo_left, solo_right) = (solo(true), solo(false));
        let sep_left = moves(true) && separates(self, &solo_left);
        let sep_right = moves(false) && separates(self, &solo_right);
        std::array::from_fn(|i| match (sep_left, sep_right) {
            (true, false) => is_left_dof(i),
            (false, true) => !is_left_dof(i),
            _ => false,
        })
    }

    /// Retract `target` to the furthest point along the segment from `prev`
    /// that stays at or above the step floor.
    ///
    /// Computes its own separating-side exemption, so callers never thread a
    /// hold mask: the exemption is this stage's business and nothing else's.
    pub(super) fn clip_to_floor(
        &mut self,
        prev: &[f64; GOV_DOF],
        target: &[f64; GOV_DOF],
        d_now: f64,
        dt: f64,
    ) -> Clip {
        let hold = self.separating_hold(prev, target, d_now, dt);
        self.scan_to_floor(prev, target, &hold, d_now, dt)
    }

    /// Walk from `prev` toward `target` and return [`Clip::Clipped`] at the first
    /// point where the straight segment drops below the step floor, or
    /// [`Clip::Clear`] if every probed point stays at or above it. `d_now` is the
    /// clearance at `prev`; the floor is [`step_floor`](Self::step_floor)`(d_now)`,
    /// so `prev` itself is at or above it by construction. Bimanual distance is
    /// not monotone along a joint-space segment, so this probes interior points
    /// (one per `MAX_PROBE_ARC_RAD` of joint motion, at least
    /// `SEGMENT_SAMPLES_MIN`) to bracket the first breach (an endpoint check
    /// alone can step over a pocket, and a fixed grid can step over one on a
    /// large jump) and bisects within that bracket for the boundary. A failed
    /// query counts as a breach (so a model-rejected configuration is never
    /// returned), retracting conservatively.
    ///
    /// Skips the scan outright when the step provably cannot reach the floor:
    /// the model's Lipschitz step bound caps the clearance change anywhere along
    /// the segment, so `d_now - floor > bound` means no interior point can cross.
    /// This makes the common ticks (holding still, slow motion, ample clearance)
    /// nearly free while fast in-band approaches keep the full scan.
    pub(super) fn scan_to_floor(
        &mut self,
        prev: &[f64; GOV_DOF],
        target: &[f64; GOV_DOF],
        hold: &[bool; GOV_DOF],
        d_now: f64,
        dt: f64,
    ) -> Clip {
        let floor = self.step_floor(d_now, dt);
        // Held DOF (a separating side) sit at `target` for the whole scan; the
        // rest interpolate, so the clip retracts only the approaching side.
        let point_at = |t: f64| -> [f64; GOV_DOF] {
            std::array::from_fn(|i| {
                if hold[i] {
                    target[i]
                } else {
                    prev[i] + t * (target[i] - prev[i])
                }
            })
        };
        // Probe spacing: one probe per resolution unit of the fastest-moving
        // DOF, so no DOF moves more than its probe resolution between probes.
        // The step reaching here is bounded by `DofSpeed` in the limit stage,
        // which is what keeps this count finite.
        let mut max_probe_ratio = 0.0_f64;
        for i in 0..GOV_DOF {
            let excursion = (target[i] - prev[i]).abs();
            let probe_resolution = if i < DUAL_DOF {
                MAX_PROBE_ARC_RAD
            } else {
                MAX_PROBE_OPENING_FRAC
            };
            max_probe_ratio = max_probe_ratio.max(excursion / probe_resolution);
        }
        // The Lipschitz early-out is keyed to `d_now` at `prev` (all-prev); a hold
        // starts the scan from a different base, so only skip when nothing is held.
        let step_q: [f64; GOV_DOF] = std::array::from_fn(|i| target[i] - prev[i]);
        let dq = split(&step_q);
        if hold.iter().all(|&h| !h)
            && d_now - floor
                > self.model.clearance_step_bound(
                    &dq.arms.left,
                    &dq.arms.right,
                    &[dq.openings.left, dq.openings.right],
                )
        {
            return Clip::Clear;
        }
        // One probe per resolution unit of the fastest-moving DOF (floored for
        // tiny steps); no fixed ceiling, so the spacing guarantee holds for any
        // step within the bound above.
        let samples = (max_probe_ratio.ceil() as usize).max(SEGMENT_SAMPLES_MIN);
        let mut last_clear = 0.0_f64;
        for s in 1..=samples {
            let t = s as f64 / samples as f64;
            match self.distance_at(&point_at(t)) {
                Some(d) if d >= floor => last_clear = t,
                _ => {
                    let (mut lo, mut hi) = (last_clear, t);
                    for _ in 0..FLOOR_BISECT_ITERS {
                        let mid = 0.5 * (lo + hi);
                        match self.distance_at(&point_at(mid)) {
                            Some(d) if d >= floor => lo = mid,
                            _ => hi = mid,
                        }
                    }
                    return Clip::Clipped(point_at(lo));
                }
            }
        }
        Clip::Clear
    }
}
