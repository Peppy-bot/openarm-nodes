//! Self-collision governor: a closing-velocity barrier over the bimanual
//! collision model. Each tick it limits only the component of the commanded joint
//! step that closes the nearest gap between the two arms, leaving tangential and
//! separating motion at full speed. The permitted approach speed ramps from a full
//! approach at `d_safe` down to zero at `d_stop` (a Faverjon-Tournassoud velocity
//! damper / exponential control-barrier form), so the surface clearance never
//! crosses `d_stop` in continuous time. URDF-based, so it governs sim and hardware
//! identically. Throttle/stop/clear transitions are logged.
//!
//! The barrier needs the gradient of the min surface distance with respect to the
//! 14 joints; the collision model computes it analytically from the nearest pair's
//! witness points (one distance query, no finite differencing). The residual
//! approximation is the per-tick linearization of the step, bounded by the small
//! control-rate step and absorbed by the exact line-search backstop: each tick the
//! realized clearance is held at or above the floor `min(d_now, d_stop)`, so an
//! approach never crosses `d_stop` and an in-penetration recovery never deepens.
//!
//! The barrier above shapes only the commanded stream and is blind to how well the
//! arms track it. A second, independent measured-state monitor (defense in depth)
//! holds closing motion, judged per arm, whenever the real clearance, from the
//! measured joint state, has closed past `MONITOR_TRIP_FRACTION * d_stop`, until
//! it recovers past `d_stop` (hysteresis, so jitter at the wall cannot chatter
//! the hold); an arm whose own motion opens the real gap always stays free. It
//! shares the governor enable, so the operator toggle gates the commanded
//! barrier and this tripwire together.

use bimanual_collision_model::{BimanualCollisionModel, CollisionError};
use tracing::{error, info, warn};

use crate::torso::{TORSO_BODY, torso_regions};
use crate::{ARM_DOF, ArmPair, JointVec};

/// Joints across both arms, left (0..7) then right (7..14).
const DUAL_DOF: usize = 2 * ARM_DOF;

/// Every governed degree of freedom: both arms' joints, then the left and right
/// gripper opening fractions. One vector so the barrier, the floor scan, the
/// separating hold, and the measured-state monitor treat an opening exactly
/// like a joint.
const GOV_DOF: usize = DUAL_DOF + 2;
const LEFT_OPENING: usize = DUAL_DOF;
const RIGHT_OPENING: usize = DUAL_DOF + 1;

/// One governed configuration: both arms' joints and both grippers' opening
/// fractions (0 = closed, 1 = fully open). The single state [`Governor::govern`]
/// throttles, scans, and monitors, so every guarantee the arms get covers the
/// fingers identically.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GovState {
    pub arms: ArmPair<JointVec>,
    pub openings: ArmPair<f64>,
}

impl GovState {
    pub fn new(arms: ArmPair<JointVec>, openings: ArmPair<f64>) -> Self {
        Self { arms, openings }
    }
}

/// Approach speed (m/s) the barrier permits at the outer edge of the band
/// (`d_safe`); it ramps linearly to zero at `d_stop`, so the clearance decays no
/// faster than this as the arms close. A module constant (not a node parameter) so
/// the node builds without regenerating peppygen; promote it to a parameter when
/// tuning on hardware.
const APPROACH_VELOCITY_AT_SAFE_M_S: f64 = 0.15;

/// Largest rate (opening fraction per second) the coordinator's chase drives a
/// gripper opening: the opening analog of the arm joint speed cap, bounding each
/// tick's opening step before it reaches the governor (whose floor scan asserts
/// and sizes probes against the same rate via
/// [`max_opening_rate_frac_s`](Governor::max_opening_rate_frac_s)). The gripper
/// node and hardware own the real opening speed. Stated in opening fraction, the
/// unit every opening DOF (wire and model alike) already uses: `3.0 /s` drives a
/// full open or close in ~1/3 s. A module constant like the approach speed above;
/// promote it to a parameter when tuning on hardware.
const MAX_OPENING_RATE_FRAC_S: f64 = 3.0;

/// Probe resolution of the floor scan on an opening DOF (fraction), the opening
/// analog of `MAX_PROBE_ARC_RAD`: one probe per this much opening travel, ~0.7 mm
/// of jaw motion, comparable surface resolution to the joint arc.
const MAX_PROBE_OPENING_FRAC: f64 = 0.01;

/// A squared gradient norm at or below this (m/rad)² means the clearance is locally
/// insensitive to motion (no closing direction exists), so the step passes
/// unconstrained instead of dividing by a near-zero norm.
const MIN_GRADIENT_NORM_SQ: f64 = 1e-18;

/// The measured-state monitor trips when the real clearance drops below this
/// fraction of `d_stop`, and releases only once it recovers past the full `d_stop`
/// (hysteresis). Sitting below the commanded floor leaves headroom for tracking
/// jitter at the wall, where the barrier parks the commanded clearance at `d_stop`,
/// so only a genuine divergence trips it. A module constant (not a node parameter),
/// like the approach speed above; promote it to a parameter when tuning on hardware.
const MONITOR_TRIP_FRACTION: f64 = 0.5;
// The trip floor must sit strictly inside (0, d_stop) or the hysteresis band
// [trip_floor, d_stop) collapses and the latch logic degrades silently.
const _: () = assert!(MONITOR_TRIP_FRACTION > 0.0 && MONITOR_TRIP_FRACTION < 1.0);

/// Floor-scan resolution: the backstop walks a per-tick segment and probes at least
/// once every `MAX_PROBE_ARC_RAD` of joint motion, so the spatial resolution is
/// bounded regardless of how large the step is. Bimanual surface distance is not
/// monotone along a joint-space segment, so a fixed grid would step over a thin
/// pocket on a large jump; scaling the probe count to the segment length keeps the
/// resolution fixed. Tied to the smallest hull feature the scan must resolve (a few
/// mm of surface motion).
const MAX_PROBE_ARC_RAD: f64 = 0.01;
/// Probe-count floor for near-zero steps, where the `MAX_PROBE_ARC_RAD` count
/// rounds down to almost nothing. The spacing guarantee itself comes from the
/// per-arc count (a full-speed step still gets `excursion / MAX_PROBE_ARC_RAD`
/// probes); this floor only keeps a handful of probes on the smallest steps, so
/// it is sized for per-tick cost at high control rates rather than density.
/// There is no fixed ceiling; the count scales with the step, and
/// `clip_to_floor` asserts the step never exceeds its velocity-limited bound,
/// which is what caps the count.
const SEGMENT_SAMPLES_MIN: usize = 4;

/// Bisection iterations within a bracketing interval once the scan finds the first
/// crossing: at the coarsest, `1/SEGMENT_SAMPLES_MIN / 2^8 ~= 1e-3` of the step.
const FLOOR_BISECT_ITERS: usize = 8;

/// Disposition of the last governed cycle: the commanded motion passed
/// unrestricted, was scaled down to hold the band, or was denied entirely
/// (stop floor, measured-state monitor hold, or a fault hold). Ordered by
/// severity; transitions are logged once, not at the control rate.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Guard {
    Clear,
    Throttling,
    Stopped,
}

/// Outcome of walking a per-tick segment against the floor.
enum Clip {
    /// Every sampled point along the segment stayed at or above the floor.
    Clear,
    /// The segment crossed below the floor; carries the furthest point reached
    /// before the first crossing.
    Clipped([f64; GOV_DOF]),
}

/// The nearest checked pair at a configuration: signed surface distance (m;
/// positive is clearance, negative is penetration) and the two link names. The
/// operator proximity readout.
pub struct NearestPair {
    pub distance: f64,
    pub link_a: String,
    pub link_b: String,
}

pub struct Governor {
    model: BimanualCollisionModel,
    /// Closing is fully stopped at or under this signed surface distance (m).
    d_stop: f64,
    /// Outside this signed surface distance (m) the barrier is inactive.
    d_safe: f64,
    /// Largest single-joint speed (rad/s). The per-tick floor scan bounds its probe
    /// count to the velocity-limited step and asserts no step exceeds it.
    max_joint_velocity_rad_s: f64,
    enabled: bool,
    guard: Guard,
    /// Whether the measured-state monitor is currently holding. Latched with
    /// hysteresis: set when the real clearance closes past `MONITOR_TRIP_FRACTION *
    /// d_stop`, cleared once it recovers past `d_stop`.
    monitor_tripped: bool,
}

impl Governor {
    /// Build the bimanual model (with the tight torso proxy) and validate the band.
    /// Fails loudly on a bad URDF / mesh dir / base link or an invalid band, so a
    /// misconfigured backbone aborts at bringup instead of running ungoverned.
    #[allow(clippy::too_many_arguments)] // distinct model inputs + band + speed bound + toggle
    pub fn build(
        urdf: &str,
        meshes_dir: &str,
        left_base: &str,
        right_base: &str,
        d_stop: f64,
        d_safe: f64,
        max_joint_velocity_rad_s: f64,
        enabled: bool,
    ) -> Result<Self, String> {
        if !valid_band(d_stop, d_safe) {
            return Err(format!(
                "invalid band: require 0 < d_stop ({d_stop}) < d_safe ({d_safe})"
            ));
        }
        if !(max_joint_velocity_rad_s.is_finite() && max_joint_velocity_rad_s > 0.0) {
            return Err(format!(
                "invalid max_joint_velocity_rad_s ({max_joint_velocity_rad_s}): must be finite and > 0"
            ));
        }
        let model = build_collision_model(urdf, meshes_dir, left_base, right_base)?;
        Ok(Self {
            model,
            d_stop,
            d_safe,
            max_joint_velocity_rad_s,
            enabled,
            guard: Guard::Clear,
            monitor_tripped: false,
        })
    }

    /// Flip the governor on/off at runtime (the operator toggle). Disabling resets
    /// the transition state so the next throttle re-logs from clear.
    pub fn set_enabled(&mut self, enabled: bool) {
        if enabled == self.enabled {
            return;
        }
        info!(
            "collision avoidance {}",
            if enabled {
                "ENABLED"
            } else {
                "DISABLED (passthrough)"
            }
        );
        self.enabled = enabled;
        if !enabled {
            self.guard = Guard::Clear;
            self.monitor_tripped = false;
        }
    }

    /// The nearest checked pair's signed surface distance and link names at this
    /// configuration (fingers placed at its openings), for the operator readout.
    /// Excluded pairs are never returned (the model drops them), and this is
    /// independent of the enabled state so the readout is live even in
    /// passthrough. `None` if the distance query fails.
    pub fn proximity(&mut self, state: &GovState) -> Option<NearestPair> {
        self.model
            .set_gripper_openings(state.openings.left, state.openings.right);
        self.model
            .min_distance(&state.arms.left, &state.arms.right)
            .ok()
            .map(|p| NearestPair {
                distance: p.distance,
                link_a: p.link_a.to_string(),
                link_b: p.link_b.to_string(),
            })
    }

    /// Disposition of the last governed cycle, for the status readout. Clear
    /// while disabled (passthrough restricts nothing), except the non-finite
    /// candidate hold, which reports Stopped in either mode.
    pub fn guard(&self) -> Guard {
        self.guard
    }

    /// Retune the band at runtime (the operator's stop/safe controls). Rejects an
    /// invalid band (`0 < d_stop < d_safe` required), keeping the current one, and
    /// is a no-op when unchanged so it can be called every tick.
    pub fn set_band(&mut self, d_stop: f64, d_safe: f64) {
        if d_stop == self.d_stop && d_safe == self.d_safe {
            return;
        }
        if !valid_band(d_stop, d_safe) {
            warn!("collision: ignoring invalid band (d_stop={d_stop}, d_safe={d_safe})");
            return;
        }
        info!("collision band set to d_stop={d_stop} d_safe={d_safe}");
        self.d_stop = d_stop;
        self.d_safe = d_safe;
    }

    /// Govern one bimanual step from `prev` to `cand` over `dt`, returning the
    /// governed configuration. Arms and gripper openings ride the same vector:
    /// the gap-closing component of the step is limited so the clearance loses
    /// no more than `allowed_closing(d) * dt` this tick, whether the closing
    /// motion is a joint or a finger opening; tangential and separating motion
    /// pass unchanged, and a disabled governor passes `cand` straight through.
    /// Fails safe to holding `prev` if the distance query fails (the model
    /// rejects a non-finite configuration or coincident witnesses in deep
    /// penetration).
    ///
    /// `measured` is the real joint state and opening fractions. Independently
    /// of the commanded barrier, if the measured clearance has closed past the
    /// monitor floor the last setpoint is held until it recovers (defense in
    /// depth, gated by the same enable, so a disabled governor skips it too).
    pub fn govern(
        &mut self,
        prev: &GovState,
        cand: &GovState,
        measured: &GovState,
        dt: f64,
    ) -> GovState {
        // Fail-safe up front: never stream a non-finite candidate (an upstream
        // glitch) to the followers. The in-band paths reach this via the distance
        // query, but the disabled and far-apart fast paths return `cand` directly,
        // so guard here so every path holds `prev` rather than passing it through.
        if concat(cand).iter().any(|x| !x.is_finite()) {
            self.guard = Guard::Stopped;
            return *prev;
        }
        if !self.enabled {
            return *cand;
        }
        // Measured-state tripwire: the commanded barrier below shapes only the
        // commanded stream and cannot see tracking error, so if the bodies have
        // actually closed past the monitor floor, hold the closing motion. The
        // gate is per side: a side whose own motion opens the real gap stays
        // free even while the other side's push is held, so neither operator can
        // trap the other's escape.
        let monitor_held;
        let cand = match self.monitor_gate(prev, cand, measured) {
            Some(gated) if gated == *prev => {
                self.guard = Guard::Stopped;
                return *prev;
            }
            Some(gated) => {
                monitor_held = true;
                gated
            }
            None => {
                monitor_held = false;
                *cand
            }
        };
        let cand = &cand;
        // One analytic query yields the current clearance and its gradient over
        // all governed DOF (the fingers are placed at prev's openings first, so
        // the query and its opening columns are evaluated at prev).
        self.model
            .set_gripper_openings(prev.openings.left, prev.openings.right);
        let (d_now, grad, link_a, link_b) = match self
            .model
            .distance_gradient(&prev.arms.left, &prev.arms.right)
        {
            Ok(g) => {
                let mut grad = [0.0; GOV_DOF];
                grad[..ARM_DOF].copy_from_slice(&g.grad_left);
                grad[ARM_DOF..DUAL_DOF].copy_from_slice(&g.grad_right);
                grad[LEFT_OPENING] = g.grad_openings[0];
                grad[RIGHT_OPENING] = g.grad_openings[1];
                (
                    g.proximity.distance,
                    grad,
                    g.proximity.link_a.to_string(),
                    g.proximity.link_b.to_string(),
                )
            }
            Err(CollisionError::WitnessesCoincide { .. }) => {
                // No usable gradient (deep penetration: the witnesses coincide). Do
                // not freeze (that traps the operator inside the collision); fall
                // back to a gradient-free, distance-only guard that still lets them
                // escape and never lets penetration deepen.
                if self.guard != Guard::Stopped {
                    warn!("collision: deep penetration, distance-only escape guard");
                    self.guard = Guard::Stopped;
                }
                return self.govern_without_gradient(prev, cand, dt);
            }
            Err(e) => {
                // NonFinite / NoPairs cannot arise from a finite, governed prev with
                // pairs configured; treat as a fault and hold rather than steer on it.
                error!("collision: distance_gradient: {e}; holding");
                self.guard = Guard::Stopped;
                return *prev;
            }
        };
        let prev_q = concat(prev);
        let cand_q = concat(cand);

        // Outside the influence zone the barrier imposes no closing limit, but the
        // candidate must still not cross the stop floor. Distance is not monotone
        // along the segment, so scan it rather than trusting either endpoint: a
        // single tick can pass through a pocket while both ends read clear.
        // A partial monitor hold (one side kept at prev) restricts the operator
        // even when the barrier below finds the gated candidate free, so the
        // reported disposition is never Clear while it is active.
        let monitor_floor = if monitor_held {
            Guard::Throttling
        } else {
            Guard::Clear
        };
        if d_now >= self.d_safe {
            let hold = self.separating_hold(&prev_q, &cand_q, d_now, dt);
            let (guard, governed) = match self.clip_to_floor(&prev_q, &cand_q, &hold, d_now, dt) {
                Clip::Clear => (Guard::Clear, *cand),
                Clip::Clipped(q) => (Guard::Stopped, split(&q)),
            };
            self.log_transition(guard.max(monitor_floor), d_now, &link_a, &link_b);
            return governed;
        }

        // In the band: throttle only the closing component (the velocity-damper
        // barrier), then hold the realized clearance at the floor with the exact
        // backstop, since the first-order projection can still let surface curvature
        // carry the clamped step past it. The backstop holds a separating side at its
        // target so one operator's push cannot clip the other's retreat.
        let (projected_q, throttled) = self.throttle_closing(&prev_q, &cand_q, &grad, d_now, dt);
        let hold = self.separating_hold(&prev_q, &projected_q, d_now, dt);
        let (governed_q, limited) =
            match self.clip_to_floor(&prev_q, &projected_q, &hold, d_now, dt) {
                Clip::Clear => (projected_q, throttled),
                Clip::Clipped(q) => (q, true),
            };

        let guard = if !limited {
            Guard::Clear
        } else if d_now <= self.d_stop {
            Guard::Stopped
        } else {
            Guard::Throttling
        };
        self.log_transition(guard.max(monitor_floor), d_now, &link_a, &link_b);
        split(&governed_q)
    }

    /// Measured-state monitor (defense in depth): the commanded barrier shapes only
    /// the commanded stream and cannot see tracking error, so this watches the real
    /// clearance from the measured state (joints and openings alike). When the
    /// bodies have actually closed past `MONITOR_TRIP_FRACTION * d_stop` it blocks
    /// commands that would close the gap further, until the clearance recovers past
    /// `d_stop` (hysteresis, latched in `monitor_tripped`, so a measurement
    /// hovering at the wall cannot chatter).
    ///
    /// While tripped, "closes further" is judged in the commanded space: a
    /// candidate keeps its freedom when its clearance is at or above the held
    /// setpoint's own clearance, so tracking divergence between the commanded and
    /// measured configurations can neither wave closing commands through nor
    /// deadlock a recovering escape, and a breach the candidate does not worsen
    /// (a pair not involving it) never freezes it.
    ///
    /// Separation is never blocked, and it is judged PER SIDE (an arm and its
    /// gripper opening together): one operator's closing push must not trap the
    /// other side's escape, so when the joint candidate closes, each side's motion
    /// is re-judged with the other held and any sub-motion that does not worsen
    /// the commanded clearance passes. Two individually opening motions can still
    /// jointly close (both sides converging on one gap); the joint candidate was
    /// already confirmed closing in that case, so the gate keeps only the single
    /// better escape.
    ///
    /// Returns `None` to pass the candidate unchanged, or `Some(gated)` with the
    /// closing sides held at `prev` (both held means a full hold). A failed
    /// distance query counts as closing (fail-safe). Only consulted while enabled.
    fn monitor_gate(
        &mut self,
        prev: &GovState,
        cand: &GovState,
        measured: &GovState,
    ) -> Option<GovState> {
        // No measured clearance (non-finite state or deep penetration): defer to the
        // main governing (whose deep-penetration fallback still lets the operator
        // escape), so a failed query never blocks separation or latches the hold.
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
        // The candidate is judged against the held setpoint's own clearance, in
        // the same (commanded) space: cross-space comparison against d_measured
        // would pass every closing command under a systematic tracking offset,
        // and freeze every escape under the opposite offset.
        let d_prev = self.distance_at(&concat(prev))?;
        let opens = |g: &mut Self, s: &GovState| -> Option<f64> {
            g.distance_at(&concat(s)).filter(|d| *d >= d_prev)
        };
        if opens(self, cand).is_some() {
            return None;
        }
        // The joint candidate closes: judge each side's motion with the other
        // held, so the closing side is held while an escaping side stays free.
        let solo_left = GovState {
            arms: ArmPair::new(cand.arms.left, prev.arms.right),
            openings: ArmPair::new(cand.openings.left, prev.openings.right),
        };
        let solo_right = GovState {
            arms: ArmPair::new(prev.arms.left, cand.arms.right),
            openings: ArmPair::new(prev.openings.left, cand.openings.right),
        };
        let left_moves =
            cand.arms.left != prev.arms.left || cand.openings.left != prev.openings.left;
        let right_moves =
            cand.arms.right != prev.arms.right || cand.openings.right != prev.openings.right;
        let d_left = left_moves.then(|| opens(self, &solo_left)).flatten();
        let d_right = right_moves.then(|| opens(self, &solo_right)).flatten();
        Some(match (d_left, d_right) {
            // Both open alone yet close together: keep the better single escape.
            (Some(dl), Some(dr)) => {
                if dl >= dr {
                    solo_left
                } else {
                    solo_right
                }
            }
            (Some(_), None) => solo_left,
            (None, Some(_)) => solo_right,
            (None, None) => *prev,
        })
    }

    /// Gradient-free fallback for deep penetration: with no usable gradient, allow
    /// the commanded step as long as the realized clearance does not drop below the
    /// floor (the current clearance, since we are already inside `d_stop`), else
    /// retract toward `prev`. Escape (which increases clearance) always passes;
    /// penetration never deepens; the operator is never frozen in place.
    fn govern_without_gradient(&mut self, prev: &GovState, cand: &GovState, dt: f64) -> GovState {
        let prev_q = concat(prev);
        let cand_q = concat(cand);
        let Some(d_now) = self.distance_at(&prev_q) else {
            return *prev;
        };
        let hold = self.separating_hold(&prev_q, &cand_q, d_now, dt);
        match self.clip_to_floor(&prev_q, &cand_q, &hold, d_now, dt) {
            Clip::Clear => split(&cand_q),
            Clip::Clipped(q) => split(&q),
        }
    }

    /// The clearance this tick's governed step must not drop below: `d_stop`
    /// normally, or the current clearance if the arms are already inside it (so an
    /// in-penetration recovery is never forced to close further).
    fn step_floor(&self, d_now: f64) -> f64 {
        d_now.min(self.d_stop)
    }

    /// The closing-velocity barrier: scale back only the gap-closing component of the
    /// step (minimum-norm, along the distance gradient) so the clearance loses no more
    /// than `allowed_closing(d_now) * dt`, then clamp each DOF's step into
    /// `[0, commanded]` so the barrier can only slow motion, never add motion a DOF
    /// was not commanded nor reverse one it was. Returns the governed configuration
    /// and whether it limited the step.
    fn throttle_closing(
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
        // The minimum-norm correction spreads the closing reduction along the
        // gradient, which can jog a DOF the operator did not drive or reverse one
        // they did. Clamp each DOF's governed step into [0, commanded step]: a held
        // DOF stays put, none reverses, separating motion is untouched.
        let governed = std::array::from_fn(|i| {
            prev_q[i] + (projected[i] - prev_q[i]).clamp(step[i].min(0.0), step[i].max(0.0))
        });
        (governed, limited)
    }

    /// Permitted approach speed (m/s) at signed surface distance `d`: zero at or
    /// under `d_stop`, the full approach at or over `d_safe`, linear between.
    fn allowed_closing(&self, d: f64) -> f64 {
        if d <= self.d_stop {
            0.0
        } else if d >= self.d_safe {
            APPROACH_VELOCITY_AT_SAFE_M_S
        } else {
            APPROACH_VELOCITY_AT_SAFE_M_S * (d - self.d_stop) / (self.d_safe - self.d_stop)
        }
    }

    /// Largest rate (fraction/s) the coordinator's chase may drive an opening
    /// candidate; the probe sizing and the velocity-limit assertion in
    /// [`clip_to_floor`](Self::clip_to_floor) are keyed to the same value.
    pub fn max_opening_rate_frac_s(&self) -> f64 {
        MAX_OPENING_RATE_FRAC_S
    }

    /// Largest per-tick step governed DOF `i` may take, from the chase's arm
    /// velocity limit or the opening rate limit. The floor scan's probe count
    /// and its upstream-bug assertion both key off this.
    fn dof_speed_limit(&self, i: usize) -> f64 {
        if i < DUAL_DOF {
            self.max_joint_velocity_rad_s
        } else {
            self.max_opening_rate_frac_s()
        }
    }

    /// Signed clearance at a governed configuration: fingers placed at its
    /// openings, then the min distance over all checked pairs. `None` on a
    /// query error so callers fail safe.
    fn distance_at(&mut self, q: &[f64; GOV_DOF]) -> Option<f64> {
        let s = split(q);
        self.model
            .set_gripper_openings(s.openings.left, s.openings.right);
        self.model
            .min_distance(&s.arms.left, &s.arms.right)
            .ok()
            .map(|p| p.distance)
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
                && matches!(g.clip_to_floor(prev, q, &no_hold, d_prev, dt), Clip::Clear)
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
    fn clip_to_floor(
        &mut self,
        prev: &[f64; GOV_DOF],
        target: &[f64; GOV_DOF],
        hold: &[bool; GOV_DOF],
        d_now: f64,
        dt: f64,
    ) -> Clip {
        let floor = self.step_floor(d_now);
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
        // The chase velocity-limits every DOF before it reaches the governor (arm
        // joints by the joint speed cap, openings by the opening rate). A larger
        // excursion is an upstream bug that would silently under-resolve the
        // scan, so fail loudly (the `* 1.01` absorbs float rounding at the exact
        // limit). The same per-DOF ratio sizes the probe count below: probes are
        // spaced so no DOF moves more than its probe resolution between them.
        let mut max_probe_ratio = 0.0_f64;
        for i in 0..GOV_DOF {
            let excursion = (target[i] - prev[i]).abs();
            let max_step = self.dof_speed_limit(i) * dt;
            assert!(
                excursion <= max_step * 1.01,
                "governed step {excursion:.4} on DOF {i} exceeds its velocity-limited bound {max_step:.4}"
            );
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

    fn log_transition(&mut self, next: Guard, d: f64, link_a: &str, link_b: &str) {
        if next == self.guard {
            return;
        }
        match next {
            Guard::Stopped => warn!(
                "collision: STOP - motion halted at d={d:+.4} m between {link_a} and {link_b}"
            ),
            Guard::Throttling => {
                warn!("collision: throttling approach, d={d:+.4} m, pair {link_a}/{link_b}")
            }
            Guard::Clear => info!("collision: clear, resuming full speed"),
        }
        self.guard = next;
    }
}

/// Build the bimanual collision model with the shared tight torso proxy, from
/// the embedded URDF and its materialized meshes. Used by both the arm governor
/// and the gripper gate so the two reason over identical geometry.
pub(crate) fn build_collision_model(
    urdf: &str,
    meshes_dir: &str,
    left_base: &str,
    right_base: &str,
) -> Result<BimanualCollisionModel, String> {
    BimanualCollisionModel::builder(urdf, meshes_dir, left_base, right_base)
        .regions(TORSO_BODY, torso_regions()?)
        .build()
        .map_err(|e| format!("build collision model: {e}"))
}

/// A valid band requires finite `0 < d_stop < d_safe` (the ramp denominator
/// `d_safe - d_stop` is then positive).
pub(crate) fn valid_band(d_stop: f64, d_safe: f64) -> bool {
    d_stop.is_finite() && d_safe.is_finite() && d_stop > 0.0 && d_safe > d_stop
}

/// Pack a governed configuration into one vector: left joints, right joints,
/// left opening, right opening.
fn concat(s: &GovState) -> [f64; GOV_DOF] {
    std::array::from_fn(|i| match i {
        LEFT_OPENING => s.openings.left,
        RIGHT_OPENING => s.openings.right,
        i if i < ARM_DOF => s.arms.left[i],
        i => s.arms.right[i - ARM_DOF],
    })
}

/// Split a governed vector back into the configuration.
fn split(q: &[f64; GOV_DOF]) -> GovState {
    GovState {
        arms: ArmPair::new(
            std::array::from_fn(|i| q[i]),
            std::array::from_fn(|i| q[ARM_DOF + i]),
        ),
        openings: ArmPair::new(q[LEFT_OPENING], q[RIGHT_OPENING]),
    }
}

/// True if governed DOF `i` belongs to the left half (arm joints or opening).
fn is_left_dof(i: usize) -> bool {
    i < ARM_DOF || i == LEFT_OPENING
}

fn dot(a: &[f64; GOV_DOF], b: &[f64; GOV_DOF]) -> f64 {
    a.iter().zip(b).map(|(x, y)| x * y).sum()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Materialize a generation's bundled collision meshes so the file-based collision
    /// builder can fit hulls; the URDF itself comes from the same `HardwareVersion`.
    /// Written once per generation into a unique tempdir held for the test process:
    /// `cargo test` runs these in parallel, so re-writing per call would let one test
    /// truncate a mesh mid-read of another's `build()`; a unique path also avoids
    /// clashing with a concurrent test process on the same host.
    fn fixture_meshes_dir(version: openarm_description::HardwareVersion) -> std::path::PathBuf {
        static V1_DIR: std::sync::OnceLock<tempfile::TempDir> = std::sync::OnceLock::new();
        static V2_DIR: std::sync::OnceLock<tempfile::TempDir> = std::sync::OnceLock::new();
        let cell = match version {
            openarm_description::HardwareVersion::V1 => &V1_DIR,
            openarm_description::HardwareVersion::V2 => &V2_DIR,
        };
        cell.get_or_init(|| {
            let dir = tempfile::tempdir().expect("create scratch dir for collision meshes");
            version
                .write_meshes_to(dir.path())
                .expect("materialize collision meshes");
            dir
        })
        .path()
        .to_path_buf()
    }

    const D_STOP: f64 = 0.005;
    const D_SAFE: f64 = 0.02;
    const DT: f64 = 0.01;
    /// Generous so the velocity-limited-step assertion never binds on the synthetic
    /// direct-jump configs these tests use; the assertion itself is covered separately.
    const MAX_JOINT_VELOCITY_RAD_S: f64 = 1000.0;

    /// In-limit home; the elbow's one-sided lower limit is 0.05.
    fn home() -> ArmPair<JointVec> {
        ArmPair::new(
            [0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0],
        )
    }

    /// Arms at `arms` with both jaws fully open: the widest finger envelope, which
    /// is what the arm-barrier scenarios governed against before openings were
    /// governed DOF, so their tuned poses and distances carry over unchanged.
    fn at(arms: ArmPair<JointVec>) -> GovState {
        GovState::new(arms, ArmPair::new(1.0, 1.0))
    }

    fn governor_for(version: openarm_description::HardwareVersion, enabled: bool) -> Governor {
        let meshes_dir = fixture_meshes_dir(version);
        Governor::build(
            version.urdf(),
            meshes_dir.to_str().expect("meshes dir path is valid UTF-8"),
            version.base_link(openarm_description::Side::Left),
            version.base_link(openarm_description::Side::Right),
            D_STOP,
            D_SAFE,
            MAX_JOINT_VELOCITY_RAD_S,
            enabled,
        )
        .expect("build governor from bundled description")
    }

    fn governor(enabled: bool) -> Governor {
        governor_for(openarm_description::HardwareVersion::V1, enabled)
    }

    fn v2_governor(enabled: bool) -> Governor {
        governor_for(openarm_description::HardwareVersion::V2, enabled)
    }

    #[test]
    fn v2_governor_builds_with_the_revolute_gripper() {
        // The v2 revolute finger joints must parse into live-placed finger
        // bodies; a build failure here means the revolute finger path regressed.
        v2_governor(true);
    }

    #[test]
    fn v2_finger_opening_changes_live_clearance_on_both_sides() {
        // The real v2 assets, not a fixture: sweep the wrists inward until a
        // finger (ee_link) is the nearest body with the grippers open, then pin
        // that closing the grippers strictly recovers clearance. v2 mirrors its
        // right gripper by flipping the finger joint limit range, so this fails
        // if either side's open/closed sense is read off the URDF limit order
        // instead of the meshes.
        let mut g = v2_governor(true);
        let pose_at = |t: f64| {
            let mut p = home();
            p.left[2] = t;
            p.right[2] = -t;
            p
        };
        let pose = (0..=40)
            .map(|i| pose_at(i as f64 * 0.05))
            .find(|p| {
                g.proximity(&at(*p))
                    .is_some_and(|n| n.link_a.contains("ee_link") || n.link_b.contains("ee_link"))
            })
            .expect("some wrists-inward pose has a finger as the nearest body when open");
        let d_open = g.proximity(&at(pose)).expect("query").distance;
        let d_closed = g
            .proximity(&GovState::new(pose, ArmPair::new(0.0, 0.0)))
            .expect("query")
            .distance;
        assert!(
            d_closed > d_open + 1e-4,
            "closing the v2 grippers should recover clearance: open {d_open:+.4}, closed {d_closed:+.4}"
        );
    }

    #[test]
    #[should_panic(expected = "exceeds its velocity-limited bound")]
    fn a_step_beyond_the_velocity_limit_trips_the_scan_assert() {
        // A tiny velocity makes the bound (max_joint_velocity * DT) 5e-4 rad, so any
        // real step exceeds it and the scan's velocity-limit assertion fires rather
        // than silently under-resolving the segment.
        let meshes_dir = fixture_meshes_dir(openarm_description::HardwareVersion::V1);
        let mut g = Governor::build(
            openarm_description::HardwareVersion::V1.urdf(),
            meshes_dir.to_str().expect("meshes dir path is valid UTF-8"),
            openarm_description::HardwareVersion::V1.base_link(openarm_description::Side::Left),
            openarm_description::HardwareVersion::V1.base_link(openarm_description::Side::Right),
            D_STOP,
            D_SAFE,
            0.05,
            true,
        )
        .expect("build governor from bundled description");
        let prev = at(home());
        let mut cand = prev;
        cand.arms.left[0] += 0.5; // 0.5 rad >> the 5e-4 rad velocity-limited bound
        let _ = g.govern(&prev, &cand, &prev, DT);
    }

    /// Both arms elbow-bent, j3 wrapping the wrists toward the centerline by `t`.
    fn wrists_inward(t: f64) -> ArmPair<JointVec> {
        ArmPair::new(
            [0.0, 0.0, t, 0.4, 0.0, 0.0, 0.0],
            [0.0, 0.0, -t, 0.4, 0.0, 0.0, 0.0],
        )
    }

    /// Signed clearance at a governed configuration (fingers placed at its openings).
    fn distance(g: &mut Governor, s: &GovState) -> f64 {
        g.distance_at(&concat(s)).expect("finite config")
    }

    /// Step `from` toward `to` by at most `max` rad on each joint (a stand-in for
    /// the velocity-limited chase that feeds the governor in the real loop).
    fn chase(from: &ArmPair<JointVec>, to: &ArmPair<JointVec>, max: f64) -> ArmPair<JointVec> {
        let one = |f: &JointVec, t: &JointVec| {
            std::array::from_fn(|i| f[i] + (t[i] - f[i]).clamp(-max, max))
        };
        ArmPair::new(one(&from.left, &to.left), one(&from.right, &to.right))
    }

    /// Govern a chase from home toward a deeply folded pose until the clearance
    /// first drops just inside the band, so a test starts in the positive-clearance
    /// regime the barrier is designed for (not deep penetration).
    fn drive_into_band(g: &mut Governor) -> GovState {
        let target = wrists_inward(1.2);
        let mut q = at(home());
        for _ in 0..400 {
            if distance(g, &q) < D_SAFE {
                break;
            }
            let cand = at(chase(&q.arms, &target, 0.05));
            q = g.govern(&q, &cand, &q, DT);
        }
        q
    }

    #[test]
    fn disabled_is_passthrough() {
        let mut g = governor(false);
        let deep = at(wrists_inward(1.2));
        assert_eq!(g.govern(&at(home()), &deep, &at(home()), DT), deep);
        assert_eq!(g.guard(), Guard::Clear, "passthrough restricts nothing");
    }

    #[test]
    fn far_apart_is_unthrottled() {
        let mut g = governor(true);
        // Home clearance is outside the band, so any step passes untouched.
        let cand = at(wrists_inward(0.2));
        assert!(
            distance(&mut g, &at(home())) >= D_SAFE,
            "home should sit outside the band"
        );
        assert_eq!(g.govern(&at(home()), &cand, &at(home()), DT), cand);
        assert_eq!(g.guard(), Guard::Clear, "unrestricted motion reads clear");
    }

    #[test]
    fn separating_motion_always_passes() {
        let mut g = governor(true);
        // Drive just into the band, then step back toward home: separating motion
        // (clearance increasing) is never throttled.
        let q = drive_into_band(&mut g);
        let cand = at(chase(&q.arms, &home(), 0.02));
        assert_eq!(g.govern(&q, &cand, &q, DT), cand);
    }

    #[test]
    fn gradient_free_guard_allows_escape_never_deepens() {
        let mut g = governor(true);
        // Deeply folded pose, the regime where the analytic gradient can degrade.
        let deep = at(wrists_inward(1.5));
        let d0 = distance(&mut g, &deep);
        let floor = d0.min(D_STOP);
        // Escape toward home increases clearance: allowed, never frozen.
        let escape = at(chase(&deep.arms, &home(), 0.02));
        let out = g.govern_without_gradient(&deep, &escape, DT);
        assert_ne!(out, deep, "escape was frozen in place");
        assert!(
            distance(&mut g, &out) >= floor - 1e-3,
            "escape dropped below the floor"
        );
        // A deeper command is held at the floor, never pushed past it.
        let deeper = at(chase(&deep.arms, &wrists_inward(2.0), 0.02));
        let held = g.govern_without_gradient(&deep, &deeper, DT);
        assert!(
            distance(&mut g, &held) >= floor - 1e-3,
            "guard let penetration deepen"
        );
    }

    #[test]
    fn held_arm_is_not_jogged_and_commanded_joints_never_reverse() {
        let mut g = governor(true);
        let q = drive_into_band(&mut g);
        // Command only the left arm further toward the centerline (closing); hold
        // the right exactly where it is.
        let pushed = chase(&q.arms, &wrists_inward(1.5), 0.02);
        let cand = GovState::new(ArmPair::new(pushed.left, q.arms.right), q.openings);
        let governed = g.govern(&q, &cand, &q, DT);
        // The held right arm must not be jogged by the barrier's correction.
        assert_eq!(
            governed.arms.right, q.arms.right,
            "held right arm was jogged"
        );
        // Each commanded left joint's governed step stays within [0, commanded]:
        // same sign as the command, never larger, never reversed.
        for i in 0..ARM_DOF {
            let cmd = cand.arms.left[i] - q.arms.left[i];
            let gov = governed.arms.left[i] - q.arms.left[i];
            assert!(
                gov * cmd >= -1e-12 && gov.abs() <= cmd.abs() + 1e-12,
                "left joint {i}: governed step {gov} outside [0, {cmd}]"
            );
        }
    }

    #[test]
    fn tangential_motion_passes_unthrottled() {
        let mut g = governor(true);
        let q = drive_into_band(&mut g);
        // Build a step orthogonal to the distance gradient (purely tangential): it
        // does not change clearance, so the barrier must pass it unthrottled.
        g.model
            .set_gripper_openings(q.openings.left, q.openings.right);
        let grad_pair = g
            .model
            .distance_gradient(&q.arms.left, &q.arms.right)
            .expect("gradient");
        let mut grad = [0.0; GOV_DOF];
        grad[..ARM_DOF].copy_from_slice(&grad_pair.grad_left);
        grad[ARM_DOF..DUAL_DOF].copy_from_slice(&grad_pair.grad_right);
        grad[LEFT_OPENING] = grad_pair.grad_openings[0];
        grad[RIGHT_OPENING] = grad_pair.grad_openings[1];
        let raw: [f64; GOV_DOF] = std::array::from_fn(|i| {
            if i < DUAL_DOF {
                ((i % 3) as f64 - 1.0) * 0.01
            } else {
                0.0
            }
        });
        let comp = dot(&raw, &grad) / dot(&grad, &grad);
        let tangential: [f64; GOV_DOF] = std::array::from_fn(|i| raw[i] - comp * grad[i]);
        let q16 = concat(&q);
        let cand = split(&std::array::from_fn(|i| q16[i] + tangential[i]));
        let governed = g.govern(&q, &cand, &q, DT);
        for i in 0..ARM_DOF {
            assert!(
                (governed.arms.left[i] - cand.arms.left[i]).abs() < 1e-9,
                "left tangential joint {i} was throttled"
            );
            assert!(
                (governed.arms.right[i] - cand.arms.right[i]).abs() < 1e-9,
                "right tangential joint {i} was throttled"
            );
        }
    }

    #[test]
    fn floor_holds_on_both_sides_of_the_scan_skip_boundary() {
        // The Lipschitz early-out skips the floor scan when the margin above
        // the floor exceeds the model's step bound. Engineer one closing step
        // whose bound sits just under that margin (skip may fire) and one just
        // over (the scan must run), and require the floor contract to hold on
        // both sides, so an off-by-margin or an underestimating bound cannot
        // silently reintroduce endpoint-trusting.
        let mut g = governor(true);
        let q = drive_into_band(&mut g);
        let d_now = distance(&mut g, &q);
        let margin = d_now - D_STOP;
        assert!(margin > 0.0, "setup: in band, above the stop");

        let prev16 = concat(&q);
        let toward16 = concat(&at(chase(&q.arms, &wrists_inward(1.5), 0.02)));
        let step16: [f64; GOV_DOF] = std::array::from_fn(|i| toward16[i] - prev16[i]);
        let dq = split(&step16);
        let bound = g.model.clearance_step_bound(
            &dq.arms.left,
            &dq.arms.right,
            &[dq.openings.left, dq.openings.right],
        );
        assert!(bound > 0.0, "setup: a closing step has a positive bound");

        for (scale, expect_skip) in [(0.9 * margin / bound, true), (1.1 * margin / bound, false)] {
            let target16: [f64; GOV_DOF] = std::array::from_fn(|i| prev16[i] + scale * step16[i]);
            // The two cases must actually straddle the skip predicate
            // (margin > bound of the scaled step), or a reshaped bound would
            // quietly turn this into a generic floor sweep.
            let scaled = split(&std::array::from_fn(|i| target16[i] - prev16[i]));
            let scaled_bound = g.model.clearance_step_bound(
                &scaled.arms.left,
                &scaled.arms.right,
                &[scaled.openings.left, scaled.openings.right],
            );
            assert_eq!(
                margin > scaled_bound,
                expect_skip,
                "scale {scale} does not straddle the skip predicate (margin {margin:.5}, bound {scaled_bound:.5})"
            );
            // No hold: this exercises the shared-segment skip predicate directly.
            match g.clip_to_floor(&prev16, &target16, &[false; GOV_DOF], d_now, DT) {
                Clip::Clear => {
                    // Whether cleared by the skip or by the scan, no point of
                    // the accepted segment may sit below the stop floor.
                    assert!(
                        segment_min(&mut g, &q, &split(&target16), 32) >= D_STOP - 1e-3,
                        "cleared segment dips below the stop at scale {scale}"
                    );
                }
                Clip::Clipped(p) => {
                    assert!(
                        distance(&mut g, &split(&p)) >= D_STOP - 1e-9,
                        "clipped point below the stop at scale {scale}"
                    );
                }
            }
        }
    }

    #[test]
    fn barrier_keeps_clearance_above_stop() {
        let mut g = governor(true);
        let target = wrists_inward(1.5);
        let mut q = at(home());
        let mut entered_band = false;
        for _ in 0..250 {
            let prev = q;
            let cand = at(chase(&prev.arms, &target, 0.02));
            q = g.govern(&prev, &cand, &prev, DT);
            let d = distance(&mut g, &q);
            entered_band |= d < D_SAFE;
            // The exact backstop holds the realized clearance at the floor, so the
            // stop distance is never breached on any tick.
            assert!(d >= D_STOP, "barrier breached: d={d:+.5}");
            // The whole realized path prev->governed, not just its endpoint, stays at
            // or above the floor (the step is small, so a coarse sweep resolves it).
            assert!(
                segment_min(&mut g, &prev, &q, 16) >= D_STOP - 1e-3,
                "the prev->governed path dipped below the stop"
            );
        }
        assert!(entered_band, "arms never approached into the band");
        // It should converge near the stop boundary, not stall far away.
        assert!(
            distance(&mut g, &q) < D_STOP + 4e-3,
            "did not settle near the stop distance"
        );
    }

    #[test]
    fn outside_band_large_jump_is_floored() {
        let mut g = governor(true);
        // Start clear (outside the band) and command a single oversized step that
        // would vault straight past the stop floor in one tick. The outside-band
        // fast path must still run the backstop and retract to the floor.
        let start = at(home());
        assert!(
            distance(&mut g, &start) >= D_SAFE,
            "start should sit outside the band"
        );
        let deep = at(wrists_inward(1.5));
        assert!(
            distance(&mut g, &deep) < D_STOP,
            "target should be past the stop floor"
        );
        let governed = g.govern(&start, &deep, &start, DT);
        assert_ne!(governed, deep, "oversized step passed unfloored");
        assert!(
            distance(&mut g, &governed) >= D_STOP,
            "large jump breached the stop floor"
        );
    }

    #[test]
    fn non_finite_candidate_holds_prev() {
        let mut g = governor(true);
        let prev = at(home());
        let mut bad = at(wrists_inward(0.2));
        bad.arms.left[0] = f64::NAN;
        // Enabled: the up-front guard holds prev rather than steering on NaN.
        assert_eq!(g.govern(&prev, &bad, &prev, DT), prev);
        assert_eq!(g.guard(), Guard::Stopped, "a fault hold reads stopped");
        // A non-finite OPENING is the same class of upstream glitch.
        let mut bad_opening = at(wrists_inward(0.2));
        bad_opening.openings.left = f64::NAN;
        assert_eq!(g.govern(&prev, &bad_opening, &prev, DT), prev);
        // Disabled fast path: still never passes a non-finite candidate through.
        g.set_enabled(false);
        assert_eq!(g.govern(&prev, &bad, &prev, DT), prev);
    }

    #[test]
    fn set_enabled_toggles_barrier() {
        let mut g = governor(true);
        // An in-band closing step is throttled when enabled, passed when disabled.
        let near = at(wrists_inward(1.0));
        let closer = at(wrists_inward(1.3));
        assert!(
            distance(&mut g, &near) < D_SAFE,
            "near pose should be in the band"
        );
        assert_ne!(g.govern(&near, &closer, &near, DT), closer);
        assert_ne!(
            g.guard(),
            Guard::Clear,
            "a limited step must read restricted"
        );
        g.set_enabled(false);
        assert_eq!(g.govern(&near, &closer, &near, DT), closer);
        assert_eq!(g.guard(), Guard::Clear, "disabling resets the readout");
    }

    /// Interpolate from `lo_pose` (clearance >= target) toward `hi_pose` (clearance <
    /// target) and return the configuration whose real clearance is ~`target`, by
    /// bisection on the distance query: a measured pose at a chosen clearance for the
    /// monitor tests.
    fn config_at_distance(
        g: &mut Governor,
        lo_pose: &GovState,
        hi_pose: &GovState,
        target: f64,
    ) -> GovState {
        let lo = concat(lo_pose);
        let hi = concat(hi_pose);
        let (mut a, mut b) = (0.0_f64, 1.0_f64);
        for _ in 0..50 {
            let m = 0.5 * (a + b);
            let q = split(&std::array::from_fn(|i| lo[i] + m * (hi[i] - lo[i])));
            if distance(g, &q) >= target {
                a = m
            } else {
                b = m
            }
        }
        split(&std::array::from_fn(|i| lo[i] + a * (hi[i] - lo[i])))
    }

    #[test]
    fn monitor_always_allows_separation_when_measured_breaches() {
        let mut g = governor(true);
        // The MEASURED arms are past the monitor floor (a real near-collision). A
        // command that opens the gap must ALWAYS pass: the operator can never be
        // trapped inside a near-collision, even while the monitor is tripped.
        let measured = at(wrists_inward(2.0));
        assert!(
            distance(&mut g, &measured) < MONITOR_TRIP_FRACTION * D_STOP,
            "measured pose must breach the monitor floor"
        );
        let prev = measured;
        let retreat = at(wrists_inward(1.4)); // a more-open configuration
        assert!(
            distance(&mut g, &retreat) > distance(&mut g, &measured),
            "retreat opens the gap"
        );
        let governed = g.govern(&prev, &retreat, &measured, DT);
        assert_ne!(
            governed, prev,
            "separation was blocked while the monitor was tripped"
        );
        assert!(
            distance(&mut g, &governed) > distance(&mut g, &measured),
            "the governed step did not open the gap"
        );
    }

    /// The left shoulder swung in is a deep self-collision; interpolating home toward
    /// it gives configurations at any chosen clearance for the monitor tests.
    fn deep_collision() -> GovState {
        let mut p = at(home());
        p.arms.left[1] = 1.4;
        p
    }

    #[test]
    fn barrier_frees_a_retreating_arm_while_the_other_pushes() {
        // The field scenario at the BARRIER (monitor untripped): parked at the
        // wall, one operator keeps pushing an arm in (its solo motion closes)
        // while the other retreats (its solo motion opens). The shared-segment
        // clip once retracted both to the same parameter, freezing the escape;
        // the separating-side hold must let the retreating arm move nearly its
        // full commanded step while the pushing arm is held at the floor.
        let mut g = governor(true);
        let q = drive_into_band(&mut g);
        let d_wall = distance(&mut g, &q);
        assert!(
            (D_STOP..D_SAFE).contains(&d_wall),
            "setup: parked in-band near the wall, got {d_wall:+.5}"
        );
        // Search a push/retreat pair that is genuinely mixed: solo-left closes
        // below the floor while solo-right opens.
        let mut found = None;
        for (push, pull) in [
            (0.05, 0.02),
            (0.1, 0.02),
            (0.15, 0.03),
            (0.2, 0.02),
            (0.2, 0.05),
        ] {
            let cl = chase(&q.arms, &wrists_inward(1.6), push).left;
            let cr = chase(&q.arms, &home(), pull).right;
            let solo_left = distance(
                &mut g,
                &GovState::new(ArmPair::new(cl, q.arms.right), q.openings),
            );
            let solo_right = distance(
                &mut g,
                &GovState::new(ArmPair::new(q.arms.left, cr), q.openings),
            );
            if solo_left < d_wall && solo_right > d_wall {
                found = Some((GovState::new(ArmPair::new(cl, cr), q.openings), solo_right));
                break;
            }
        }
        let (cand, solo_right) = found.expect("setup: some push/retreat pair is mixed");

        let governed = g.govern(&q, &cand, &q, DT);
        // The retreating (right) arm must actually move; the governed config
        // stays at or above the stop floor.
        assert_ne!(
            governed.arms.right, q.arms.right,
            "the retreating arm was frozen"
        );
        assert!(
            distance(&mut g, &governed) >= D_STOP - 1e-6,
            "the governed step breached the stop floor"
        );
        // It should recover most of the way to the solo-retreat clearance, not a
        // token sliver (the frozen-arm bug clipped it to a few percent).
        let opened = distance(
            &mut g,
            &GovState::new(ArmPair::new(q.arms.left, governed.arms.right), q.openings),
        );
        assert!(
            opened >= d_wall + 0.5 * (solo_right - d_wall),
            "retreat barely moved: opened to {opened:+.5} of solo {solo_right:+.5} from wall {d_wall:+.5}"
        );
    }

    #[test]
    fn monitor_frees_a_retreating_arm_while_the_other_pushes() {
        // The field scenario: with the monitor tripped, one operator keeps
        // pushing arm A into the breach while the other retreats arm B. The
        // joint candidate closes (A dominates), but B's own motion opens the
        // real gap, so the per-side gate must hold A and let B escape; a
        // whole-candidate hold would freeze B for as long as A pushes.
        let mut g = governor(true);
        // A shallow breach (positive clearance under the trip floor) keeps the
        // distance field smooth, and this asymmetric converging pose binds a
        // CROSS-ARM pair, so both arms' motion genuinely moves the gap (a
        // torso-bound pair would make the retreating arm irrelevant).
        let deep = at(ArmPair::new(
            [0.0, 0.0, 1.15, 0.4, 0.1, 0.0, 0.2],
            [0.0, 0.0, -1.25, 0.4, -0.1, 0.1, 0.0],
        ));
        assert!(
            distance(&mut g, &deep) < 0.5 * MONITOR_TRIP_FRACTION * D_STOP,
            "setup: the deep pose must pass the breach target"
        );
        let measured = config_at_distance(
            &mut g,
            &at(home()),
            &deep,
            0.5 * MONITOR_TRIP_FRACTION * D_STOP,
        );
        let d_measured = distance(&mut g, &measured);
        assert!(
            d_measured < MONITOR_TRIP_FRACTION * D_STOP,
            "setup: measured pose must breach the monitor floor"
        );
        let prev = measured;
        // Find a push/pull step pair where the joint candidate closes (the push
        // dominates) while the retreating arm alone opens: the mixed case the
        // per-side gate exists for.
        let mut found = None;
        for (push_step, pull_step) in [(0.05, 0.01), (0.1, 0.01), (0.15, 0.02), (0.2, 0.02)] {
            let push = chase(&measured.arms, &deep.arms, push_step);
            let pull = chase(&measured.arms, &home(), pull_step);
            let cand = GovState::new(ArmPair::new(push.left, pull.right), measured.openings);
            let solo_right = GovState::new(
                ArmPair::new(prev.arms.left, cand.arms.right),
                measured.openings,
            );
            let solo_left = GovState::new(
                ArmPair::new(cand.arms.left, prev.arms.right),
                measured.openings,
            );
            if distance(&mut g, &cand) <= d_measured
                && distance(&mut g, &solo_right) > d_measured
                && distance(&mut g, &solo_left) <= d_measured
            {
                found = Some(cand);
                break;
            }
        }
        let cand = found.expect("setup: some push/pull pair produces the mixed case");

        let governed = g.govern(&prev, &cand, &measured, DT);
        assert_eq!(
            governed.arms.left, prev.arms.left,
            "the pushing arm must be held"
        );
        assert_ne!(
            governed.arms.right, prev.arms.right,
            "the retreating arm was frozen"
        );
        assert!(
            distance(&mut g, &governed) > d_measured,
            "the freed motion must open the real gap"
        );
    }

    #[test]
    fn monitor_holds_a_closing_command_when_measured_breaches() {
        let mut g = governor(true);
        // Same breach, but the command would close the gap further: that is held.
        let deep = deep_collision();
        assert!(
            distance(&mut g, &deep) < 0.0,
            "deep pose must be in penetration"
        );
        let measured = config_at_distance(
            &mut g,
            &at(home()),
            &deep,
            0.5 * MONITOR_TRIP_FRACTION * D_STOP,
        );
        assert!(
            distance(&mut g, &measured) < MONITOR_TRIP_FRACTION * D_STOP,
            "measured must breach the floor"
        );
        let prev = measured;
        // `deep` is more closed than the measured pose: a closing command, held at prev.
        assert!(
            distance(&mut g, &deep) < distance(&mut g, &measured),
            "deep is a closing command"
        );
        assert_eq!(
            g.govern(&prev, &deep, &measured, DT),
            prev,
            "a closing command was not held on a measured breach"
        );
    }

    #[test]
    fn monitor_judges_closing_in_the_commanded_space_under_a_tracking_offset() {
        let mut g = governor(true);
        // A systematic tracking offset: the measured arms breach the monitor
        // floor while the commanded setpoints still read ~15 mm clear. Judged
        // against the measured clearance (the old cross-space baseline), every
        // velocity-limited closing candidate would read as "opening" (~15 mm vs
        // ~2 mm) and pass; judged in the commanded space, a candidate that
        // closes on the held setpoint is held.
        let deep = deep_collision();
        let measured = config_at_distance(
            &mut g,
            &at(home()),
            &deep,
            0.5 * MONITOR_TRIP_FRACTION * D_STOP,
        );
        let prev = config_at_distance(&mut g, &at(home()), &deep, 0.015);
        let cand = at(chase(&prev.arms, &deep.arms, 0.02));
        assert!(
            distance(&mut g, &cand) < distance(&mut g, &prev),
            "setup: the candidate closes on the held setpoint"
        );
        assert_eq!(
            g.govern(&prev, &cand, &measured, DT),
            prev,
            "a closing command passed the tripped monitor under a tracking offset"
        );
        // The escape is still free under the same offset: opening on the held
        // setpoint passes even though its clearance is far above the measured.
        let retreat = at(chase(&prev.arms, &home(), 0.02));
        assert!(
            distance(&mut g, &retreat) > distance(&mut g, &prev),
            "setup: the retreat opens on the held setpoint"
        );
        assert_ne!(
            g.govern(&prev, &retreat, &measured, DT),
            prev,
            "a separating command was frozen by the cross-space baseline"
        );
    }

    #[test]
    fn monitor_inert_under_good_tracking() {
        let mut g = governor(true);
        // Measured == commanded (perfect tracking), far apart: the monitor never
        // trips and the commanded step passes as it would without it.
        let prev = at(home());
        let cand = at(wrists_inward(0.2));
        assert!(
            distance(&mut g, &prev) >= D_SAFE,
            "precondition: home sits outside the band"
        );
        assert_eq!(
            g.govern(&prev, &cand, &prev, DT),
            cand,
            "monitor tripped under good tracking"
        );
    }

    #[test]
    fn monitor_hysteresis_holds_a_closing_command_until_recovered_past_d_stop() {
        let mut g = governor(true);
        // `deep` is a closing command vs every measured pose below, so the monitor's
        // hold, not a separation pass, is what is under test.
        let deep = deep_collision();
        let prev = at(home());
        let breaching = config_at_distance(
            &mut g,
            &at(home()),
            &deep,
            0.5 * MONITOR_TRIP_FRACTION * D_STOP,
        );
        assert!(distance(&mut g, &breaching) < MONITOR_TRIP_FRACTION * D_STOP);

        // A measured pose whose real clearance sits inside the hysteresis band
        // [trip floor, d_stop): below the commanded stop but above the trip floor.
        let in_band = config_at_distance(
            &mut g,
            &at(home()),
            &deep,
            0.5 * (MONITOR_TRIP_FRACTION * D_STOP + D_STOP),
        );
        assert!(
            (MONITOR_TRIP_FRACTION * D_STOP..D_STOP).contains(&distance(&mut g, &in_band)),
            "setup: in_band not in the hysteresis band"
        );

        // Breach trips the latch: the closing command is held at prev.
        assert_eq!(
            g.govern(&prev, &deep, &breaching, DT),
            prev,
            "closing command not held on a breach"
        );
        // In-band measurement (above the trip floor, below d_stop): still held (hysteresis).
        assert_eq!(
            g.govern(&prev, &deep, &in_band, DT),
            prev,
            "released before recovering past d_stop"
        );
        // Recovered past d_stop: the latch releases, so the command is governed
        // normally (clipped toward the floor), not force-held at prev.
        assert_ne!(
            g.govern(&prev, &deep, &at(home()), DT),
            prev,
            "did not release after recovery"
        );
        // The release must actually clear the LATCH, not just pass this call: a
        // later in-band measurement (inside the hysteresis band) must be judged
        // from the untripped threshold and not re-hold. A latch stuck set would
        // force-hold here forever.
        assert_ne!(
            g.govern(&prev, &deep, &in_band, DT),
            prev,
            "the latch did not clear on recovery"
        );
    }

    #[test]
    fn monitor_inert_when_disabled() {
        let mut g = governor(false);
        // Tied to the operator toggle: disabled is passthrough even though the
        // measured arms breach the floor.
        let cand = at(wrists_inward(0.2));
        let breaching = at(wrists_inward(2.0));
        assert_eq!(g.govern(&at(home()), &cand, &breaching, DT), cand);
    }

    #[test]
    fn monitor_defers_when_the_measured_query_fails_so_separation_is_never_blocked() {
        let mut g = governor(true);
        // A non-finite measured state makes the distance query fail. The monitor must
        // not hold (which would block escape); it defers to the main governing, so the
        // command is never force-held at prev.
        let prev = at(home());
        let cand = at(wrists_inward(0.2));
        let mut measured = at(home());
        measured.arms.left[0] = f64::NAN;
        assert_eq!(
            g.govern(&prev, &cand, &measured, DT),
            cand,
            "monitor blocked a command on a failed measured query"
        );
    }

    #[test]
    fn monitor_does_not_trip_from_clear_on_an_in_band_measurement() {
        let mut g = governor(true);
        // Hysteresis asymmetry: from an untripped state the trip threshold is the trip
        // floor, not d_stop, so a measurement in [trip floor, d_stop) must NOT trip. A
        // closing command is then governed normally, not force-held at prev.
        let deep = deep_collision();
        let in_band = config_at_distance(
            &mut g,
            &at(home()),
            &deep,
            0.5 * (MONITOR_TRIP_FRACTION * D_STOP + D_STOP),
        );
        assert!(
            (MONITOR_TRIP_FRACTION * D_STOP..D_STOP).contains(&distance(&mut g, &in_band)),
            "setup: in_band not in the hysteresis band"
        );
        assert_ne!(
            g.govern(&at(home()), &deep, &in_band, DT),
            at(home()),
            "an in-band measurement tripped from a clear state"
        );
    }

    #[test]
    fn concat_split_round_trip() {
        let state = GovState::new(
            ArmPair::new(
                [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0],
                [8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0],
            ),
            ArmPair::new(0.25, 0.75),
        );
        assert_eq!(split(&concat(&state)), state);
        let flat: [f64; GOV_DOF] = std::array::from_fn(|i| i as f64);
        assert_eq!(concat(&split(&flat)), flat);
    }

    /// Minimum clearance sampled along the straight segment `prev`->`cand`.
    fn segment_min(g: &mut Governor, prev: &GovState, cand: &GovState, n: usize) -> f64 {
        let p = concat(prev);
        let c = concat(cand);
        let mut m = f64::INFINITY;
        for i in 0..=n {
            let t = i as f64 / n as f64;
            m = m.min(distance(
                g,
                &split(&std::array::from_fn(|j| p[j] + t * (c[j] - p[j]))),
            ));
        }
        m
    }

    #[test]
    fn outside_band_segment_is_scanned_even_when_both_ends_are_clear() {
        let mut g = governor(true);
        // Bimanual distance is not monotone along a joint-space segment, and the
        // governor must not trust the endpoints even when both are clear of the band.
        // Sweeping the left shoulder (j1) swings the left arm around the right one:
        // from home the clearance dives into deep penetration near j1=1.4 and
        // resurfaces by j1~3.15. So home and a far shoulder angle are both clear of
        // d_safe, yet the straight segment between them crosses well below the stop.
        let prev = at(home());
        let cand = {
            let mut p = at(home());
            p.arms.left[1] = 3.2;
            p
        };
        assert!(
            distance(&mut g, &prev) >= D_SAFE,
            "home end is clear of the band"
        );
        assert!(
            distance(&mut g, &cand) >= D_SAFE,
            "far-shoulder end is clear of the band"
        );
        assert!(
            segment_min(&mut g, &prev, &cand, 128) < D_STOP,
            "the segment dips below the stop"
        );

        // Trusting the (clear) endpoints would pass `cand` through; the segment scan
        // must clip it to a setpoint that is itself clear of the stop.
        let governed = g.govern(&prev, &cand, &prev, DT);
        assert_ne!(
            governed, cand,
            "a clear-ended segment with a sub-stop interior was passed unclipped"
        );
        assert!(
            distance(&mut g, &governed) >= D_STOP - 1e-6,
            "the clipped setpoint is below the stop"
        );
    }

    // --- Gripper scenarios (v2, whose fingers travel the farthest) -----------
    //
    // The openings are ordinary governed DOF, so every arm guarantee above
    // already applies to them; these pin the finger-specific geometry paths:
    // opening into the other arm, bilateral openings sharing one clearance,
    // closing as separation, and the monitor judging measured fingers.

    /// A wrists-inward v2 pose whose left finger sweeps toward the right palm as
    /// the jaw opens: with the jaws closed the palms are ~23 mm clear, fully open
    /// the finger penetrates. The gripper scenarios open into it.
    fn finger_into_other_arm() -> ArmPair<JointVec> {
        ArmPair::new(
            [0.0, 0.0, 0.6, 0.4, -0.4, 0.4, -0.4],
            [0.0, 0.0, -0.6, 0.4, 0.0, 0.0, 0.0],
        )
    }

    /// Drive `start` toward `target` through the governor tick by tick with
    /// velocity-limited candidates (arms and openings alike), asserting the
    /// realized clearance never crosses the stop floor on any tick; measured
    /// tracks commanded (perfect tracking). Returns the settled state.
    fn drive(g: &mut Governor, start: GovState, target: &GovState, ticks: usize) -> GovState {
        let opening_step = g.max_opening_rate_frac_s() * DT;
        let chase_frac = |from: f64, to: f64| from + (to - from).clamp(-opening_step, opening_step);
        let mut s = start;
        for _ in 0..ticks {
            let cand = GovState::new(
                chase(&s.arms, &target.arms, 0.02),
                ArmPair::new(
                    chase_frac(s.openings.left, target.openings.left),
                    chase_frac(s.openings.right, target.openings.right),
                ),
            );
            s = g.govern(&s, &cand, &s, DT);
            let d = distance(g, &s);
            assert!(d >= D_STOP - 1e-9, "floor breached mid-drive: d={d:+.6}");
        }
        s
    }

    #[test]
    fn gripper_opening_into_the_other_arm_settles_at_the_stop() {
        let mut g = v2_governor(true);
        let arms = finger_into_other_arm();
        let start = GovState::new(arms, ArmPair::new(0.0, 0.0));
        assert!(
            distance(&mut g, &start) >= D_SAFE,
            "setup: closed jaws start clear of the band"
        );
        // Command the left jaw fully open; the barrier admits a useful partial
        // opening and parks it at the floor, holding the floor on every tick.
        let target = GovState::new(arms, ArmPair::new(1.0, 0.0));
        let settled = drive(&mut g, start, &target, 300);
        assert!(
            (1e-4..1.0 - 1e-4).contains(&settled.openings.left),
            "opening into the other arm should settle to a safe partial, got {}",
            settled.openings.left
        );
        assert_eq!(
            settled.openings.right, 0.0,
            "uncommanded right gripper opened"
        );
        // Settled NEAR the stop, not stalled far above it.
        let d = distance(&mut g, &settled);
        assert!(
            d < D_STOP + 4e-3,
            "did not settle near the stop distance: d={d:+.5}"
        );
        assert_ne!(
            g.guard(),
            Guard::Clear,
            "parked at the floor with open still commanded must read restricted"
        );
    }

    #[test]
    fn gripper_closing_recovers_clearance_under_the_same_barrier() {
        let mut g = v2_governor(true);
        let arms = finger_into_other_arm();
        // Park the left opening at the floor, then command fully closed. Closing
        // is NOT exempt from governing: a jaw has two fingers, and at an angled
        // pose retracting the near finger advances the far one, so a closing
        // sub-motion can itself reduce clearance and gets budgeted like any
        // other. The invariants are the floor holding on every tick (asserted
        // inside `drive`), steady progress to fully closed, and the clearance
        // recovering past the parked value.
        let parked = drive(
            &mut g,
            GovState::new(arms, ArmPair::new(0.0, 0.0)),
            &GovState::new(arms, ArmPair::new(1.0, 0.0)),
            300,
        );
        let d_parked = distance(&mut g, &parked);
        let closed = drive(
            &mut g,
            parked,
            &GovState::new(arms, ArmPair::new(0.0, 0.0)),
            300,
        );
        assert!(
            closed.openings.left < 1e-6,
            "the jaw did not close: settled at {}",
            closed.openings.left
        );
        assert!(
            distance(&mut g, &closed) > d_parked,
            "closing the jaw should recover clearance"
        );
    }

    #[test]
    fn bilateral_openings_share_one_clearance_budget() {
        let mut g = v2_governor(true);
        // Two grippers facing each other across the centerline: with the jaws
        // closed the pose is clear of the band, with both fully open the fingers
        // interpenetrate, so opening EITHER jaw closes the same gap.
        let pose = wrists_inward(0.55);
        let start = GovState::new(pose, ArmPair::new(0.0, 0.0));
        assert!(
            distance(&mut g, &start) >= D_SAFE,
            "setup: closed jaws start clear of the band"
        );
        assert!(
            distance(&mut g, &GovState::new(pose, ArmPair::new(1.0, 1.0))) < 0.0,
            "setup: both jaws open must interpenetrate"
        );
        // Command BOTH jaws fully open at once. One shared barrier budgets the
        // joint motion, so the combined opening still holds the floor on every
        // tick (asserted inside drive); two independent barriers would each
        // spend the full budget and jointly breach it.
        let settled = drive(
            &mut g,
            start,
            &GovState::new(pose, ArmPair::new(1.0, 1.0)),
            300,
        );
        let d = distance(&mut g, &settled);
        assert!(
            (D_STOP - 1e-9..D_STOP + 4e-3).contains(&d),
            "bilateral openings should park the shared clearance at the stop, got {d:+.5}"
        );
        // Both jaws made real progress: the budget was shared, not starved onto
        // one side.
        assert!(
            settled.openings.left > 1e-3 && settled.openings.right > 1e-3,
            "both jaws should open partially, got {:?}",
            settled.openings
        );
    }

    #[test]
    fn mixed_arm_push_and_opening_same_tick_holds_the_floor() {
        let mut g = v2_governor(true);
        let arms = finger_into_other_arm();
        let start = GovState::new(arms, ArmPair::new(0.0, 0.0));
        // Drive the left arm further inward AND its jaw open in the same ticks:
        // the shared barrier budgets the combined closing motion, and the floor
        // holds on every tick (asserted inside drive).
        let mut pushed = arms;
        pushed.left[2] += 0.3;
        let target = GovState::new(pushed, ArmPair::new(1.0, 0.0));
        let settled = drive(&mut g, start, &target, 300);
        let d = distance(&mut g, &settled);
        assert!(
            d < D_STOP + 4e-3,
            "the mixed push should converge near the stop, got {d:+.5}"
        );
    }

    #[test]
    fn opening_below_the_floor_recovers_by_closing_never_deepens() {
        let mut g = v2_governor(true);
        let arms = finger_into_other_arm();
        // Force a sub-floor state (finger opened into the other arm, as an
        // upstream fault or disabled-period motion would leave it).
        let stuck = GovState::new(arms, ArmPair::new(0.6, 0.0));
        let d_stuck = distance(&mut g, &stuck);
        assert!(
            d_stuck < D_STOP,
            "setup: the forced state breaches the floor"
        );
        // Opening further is fully frozen (the floor is the current clearance).
        let opening_step = g.max_opening_rate_frac_s() * DT;
        let deeper = GovState::new(arms, ArmPair::new(0.6 + opening_step, 0.0));
        let held = g.govern(&stuck, &deeper, &stuck, DT);
        assert!(
            distance(&mut g, &held) >= d_stuck - 1e-9,
            "an opening below the floor deepened the breach"
        );
        // Closing recovers: the escape passes and clearance increases.
        let closing = GovState::new(arms, ArmPair::new(0.6 - opening_step, 0.0));
        let governed = g.govern(&stuck, &closing, &stuck, DT);
        assert_eq!(governed, closing, "the closing escape was throttled");
        assert!(
            distance(&mut g, &governed) > d_stuck,
            "closing did not recover clearance"
        );
    }

    #[test]
    fn disabled_passes_a_colliding_opening_through() {
        let mut g = v2_governor(false);
        let arms = finger_into_other_arm();
        let prev = GovState::new(arms, ArmPair::new(0.0, 0.0));
        let open = GovState::new(arms, ArmPair::new(1.0, 0.0));
        assert_eq!(
            g.govern(&prev, &open, &prev, DT),
            open,
            "disabled governor throttled the opening"
        );
    }

    #[test]
    fn monitor_blocks_an_opening_on_a_measured_finger_breach() {
        let mut g = v2_governor(true);
        let arms = finger_into_other_arm();
        // The MEASURED fingers are open into the other arm past the trip floor
        // (tracking divergence: the commanded jaw is still nearly closed).
        let measured = config_at_distance(
            &mut g,
            &GovState::new(arms, ArmPair::new(0.0, 0.0)),
            &GovState::new(arms, ArmPair::new(1.0, 0.0)),
            0.5 * MONITOR_TRIP_FRACTION * D_STOP,
        );
        assert!(
            distance(&mut g, &measured) < MONITOR_TRIP_FRACTION * D_STOP,
            "setup: measured fingers breach the monitor floor"
        );
        let prev = GovState::new(arms, ArmPair::new(0.1, 0.0));
        let opening_step = g.max_opening_rate_frac_s() * DT;
        // Opening further closes the commanded-space gap: held.
        let open_more = GovState::new(arms, ArmPair::new(0.1 + opening_step, 0.0));
        assert!(
            distance(&mut g, &open_more) < distance(&mut g, &prev),
            "setup: opening closes the gap"
        );
        assert_eq!(
            g.govern(&prev, &open_more, &measured, DT),
            prev,
            "the monitor passed an opening during a measured finger breach"
        );
        assert_eq!(g.guard(), Guard::Stopped, "a monitor hold reads stopped");
        // Closing (separation) still passes: the operator is never trapped.
        let close = GovState::new(arms, ArmPair::new(0.1 - opening_step, 0.0));
        assert_ne!(
            g.govern(&prev, &close, &measured, DT),
            prev,
            "the monitor froze the closing escape"
        );
    }
}
