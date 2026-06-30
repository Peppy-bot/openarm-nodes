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
//! holds the last setpoint whenever the real clearance, from the measured joint
//! state, has closed past `MONITOR_TRIP_FRACTION * d_stop`, until it recovers past
//! `d_stop` (hysteresis, so jitter at the wall cannot chatter the hold). It shares
//! the governor enable, so the operator toggle gates the commanded barrier and this
//! tripwire together.

use bimanual_collision_model::{BimanualCollisionModel, CollisionError};
use tracing::{error, info, warn};

use crate::openarm_v10::{TORSO_BODY, torso_hulls};
use crate::{ARM_DOF, ArmPair, JointVec};

/// Joints across both arms, left (0..7) then right (7..14).
const DUAL_DOF: usize = 2 * ARM_DOF;

/// Approach speed (m/s) the barrier permits at the outer edge of the band
/// (`d_safe`); it ramps linearly to zero at `d_stop`, so the clearance decays no
/// faster than this as the arms close. A module constant (not a node parameter) so
/// the node builds without regenerating peppygen; promote it to a parameter when
/// tuning on hardware.
const APPROACH_VELOCITY_AT_SAFE_M_S: f64 = 0.15;

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

/// Interior samples the backstop walks along a per-tick segment to bracket the
/// first sub-floor crossing. Bimanual surface distance is not monotone along a
/// joint-space segment, so an endpoint check (or a bisection that assumes
/// monotonicity) can step over a thin pocket; scanning bounds a missed crossing to
/// ~1/`SEGMENT_SAMPLES` of the step, which the small velocity-limited tick resolves.
const SEGMENT_SAMPLES: usize = 16;

/// Bisection iterations within the bracketing interval (width `1/SEGMENT_SAMPLES`)
/// once the scan finds the first crossing: `1/16 / 2^8 ~= 1e-4` of the step, on par
/// with the prior whole-segment resolution.
const FLOOR_BISECT_ITERS: usize = 8;

/// Where the governor last sat, so throttle/stop/clear are logged on transition
/// rather than at the control rate.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Guard {
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
    Clipped([f64; DUAL_DOF]),
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
    /// misconfigured hub aborts at bringup instead of running ungoverned.
    pub fn build(
        urdf_path: &str,
        meshes_dir: &str,
        left_base: &str,
        right_base: &str,
        d_stop: f64,
        d_safe: f64,
        enabled: bool,
    ) -> Result<Self, String> {
        if !valid_band(d_stop, d_safe) {
            return Err(format!("invalid band: require 0 < d_stop ({d_stop}) < d_safe ({d_safe})"));
        }
        let model = BimanualCollisionModel::builder_from_file(urdf_path, meshes_dir, left_base, right_base)
            .map_err(|e| format!("build collision model from '{urdf_path}': {e}"))?
            .hulls(TORSO_BODY, torso_hulls())
            .build()
            .map_err(|e| format!("finalize collision model: {e}"))?;
        Ok(Self { model, d_stop, d_safe, enabled, guard: Guard::Clear, monitor_tripped: false })
    }

    /// Flip the governor on/off at runtime (the operator toggle). Disabling resets
    /// the transition state so the next throttle re-logs from clear.
    pub fn set_enabled(&mut self, enabled: bool) {
        if enabled == self.enabled {
            return;
        }
        info!("collision avoidance {}", if enabled { "ENABLED" } else { "DISABLED (passthrough)" });
        self.enabled = enabled;
        if !enabled {
            self.guard = Guard::Clear;
            self.monitor_tripped = false;
        }
    }

    /// The nearest checked pair's signed surface distance and link names at this
    /// configuration, for the operator readout. Excluded pairs are never returned
    /// (the model drops them), and this is independent of the enabled state so the
    /// readout is live even in passthrough. `None` if the distance query fails.
    pub fn proximity(&mut self, arms: &ArmPair<JointVec>) -> Option<NearestPair> {
        self.model
            .min_distance(&arms.left, &arms.right)
            .ok()
            .map(|p| NearestPair {
                distance: p.distance,
                link_a: p.link_a.to_string(),
                link_b: p.link_b.to_string(),
            })
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
    /// governed configuration. The gap-closing component of the step is limited so
    /// the clearance loses no more than `allowed_closing(d) * dt` this tick;
    /// tangential and separating motion pass unchanged, and a disabled governor
    /// passes `cand` straight through. Fails safe to holding `prev` if the distance
    /// query fails (the model rejects a non-finite configuration or coincident
    /// witnesses in deep penetration).
    ///
    /// `measured` is the arms' real joint state. Independently of the commanded
    /// barrier, if the measured clearance has closed past the monitor floor the last
    /// setpoint is held until it recovers (defense in depth, gated by the same
    /// enable, so a disabled governor skips it too).
    pub fn govern(&mut self, prev: &ArmPair<JointVec>, cand: &ArmPair<JointVec>, measured: &ArmPair<JointVec>, dt: f64) -> ArmPair<JointVec> {
        // Fail-safe up front: never stream a non-finite candidate (an upstream
        // glitch) to the followers. The in-band paths reach this via the distance
        // query, but the disabled and far-apart fast paths return `cand` directly,
        // so guard here so every path holds `prev` rather than passing it through.
        if concat(cand).iter().any(|x| !x.is_finite()) {
            return *prev;
        }
        if !self.enabled {
            return *cand;
        }
        // Measured-state tripwire: the commanded barrier below shapes only the
        // commanded stream and cannot see tracking error, so if the arms have
        // actually closed past the monitor floor, hold `prev` until they recover.
        if self.actual_breach(measured) {
            return *prev;
        }
        // One analytic query yields both the current clearance and its gradient.
        let (d_now, grad, link_a, link_b) = match self.model.distance_gradient(&prev.left, &prev.right) {
            Ok(g) => (g.proximity.distance, concat(&ArmPair::new(g.grad_left, g.grad_right)), g.proximity.link_a.to_string(), g.proximity.link_b.to_string()),
            Err(CollisionError::WitnessesCoincide { .. }) => {
                // No usable gradient (deep penetration: the witnesses coincide). Do
                // not freeze (that traps the operator inside the collision); fall
                // back to a gradient-free, distance-only guard that still lets them
                // escape and never lets penetration deepen.
                if self.guard != Guard::Stopped {
                    warn!("collision: deep penetration, distance-only escape guard");
                    self.guard = Guard::Stopped;
                }
                return self.govern_without_gradient(prev, cand);
            }
            Err(e) => {
                // NonFinite / NoPairs cannot arise from a finite, governed prev with
                // pairs configured; treat as a fault and hold rather than steer on it.
                error!("collision: distance_gradient: {e}; holding");
                return *prev;
            }
        };
        let prev14 = concat(prev);
        let cand14 = concat(cand);

        // Outside the influence zone the barrier imposes no closing limit, but the
        // candidate must still not cross the stop floor. Distance is not monotone
        // along the segment, so scan it rather than trusting either endpoint: a
        // single tick can pass through a pocket while both ends read clear.
        if d_now >= self.d_safe {
            return match self.clip_to_floor(&prev14, &cand14, self.d_stop) {
                Clip::Clear => {
                    self.log_transition(Guard::Clear, d_now, &link_a, &link_b);
                    *cand
                }
                Clip::Clipped(q) => {
                    self.log_transition(Guard::Stopped, d_now, &link_a, &link_b);
                    split(&q)
                }
            };
        }

        let step: [f64; DUAL_DOF] = std::array::from_fn(|i| cand14[i] - prev14[i]);
        // Predicted change in clearance over this tick if the full step is taken,
        // and the most clearance the barrier permits losing.
        let predicted_delta_d = dot(&grad, &step);
        let max_loss = self.allowed_closing(d_now) * dt;

        let norm_sq = dot(&grad, &grad);
        let (mut governed14, mut limited) = if predicted_delta_d >= -max_loss || norm_sq <= MIN_GRADIENT_NORM_SQ {
            (cand14, false)
        } else {
            // Subtract just enough of the closing component (along the distance
            // gradient) to land on the barrier `grad . step = -max_loss`.
            let excess = (predicted_delta_d + max_loss) / norm_sq;
            (std::array::from_fn(|i| prev14[i] + step[i] - excess * grad[i]), true)
        };

        // The barrier may only slow the commanded motion, never add motion a joint
        // was not commanded. The minimum-norm correction above spreads the closing
        // reduction along the gradient, which can jog the arm the operator is not
        // driving and even reverse the joint they are (fighting an escape). Clamp
        // each joint's governed step into [0, commanded step]: a held joint stays
        // put, no joint reverses, and separating motion is untouched. The backstop
        // below still guarantees the floor on the clamped step.
        for i in 0..DUAL_DOF {
            governed14[i] = prev14[i] + (governed14[i] - prev14[i]).clamp(step[i].min(0.0), step[i].max(0.0));
        }

        // Exact backstop: the first-order projection can still let surface curvature
        // carry the step past the stop floor, so walk prev->governed and clip at the
        // first floor crossing. The floor is d_stop, or the current clearance if
        // already inside it (so an in-penetration recovery is never forced to close
        // further).
        let floor = d_now.min(self.d_stop);
        if let Clip::Clipped(q) = self.clip_to_floor(&prev14, &governed14, floor) {
            governed14 = q;
            limited = true;
        }

        let guard = if !limited {
            Guard::Clear
        } else if d_now <= self.d_stop {
            Guard::Stopped
        } else {
            Guard::Throttling
        };
        self.log_transition(guard, d_now, &link_a, &link_b);
        split(&governed14)
    }

    /// Measured-state monitor (defense in depth): does the real clearance, from the
    /// measured joint state, sit past the monitor floor? Trips when it drops below
    /// `MONITOR_TRIP_FRACTION * d_stop` and stays tripped until it recovers past
    /// `d_stop` (hysteresis, latched in `monitor_tripped`), so a measurement hovering
    /// at the wall cannot chatter the hold. A failed distance query cannot confirm
    /// the arms are clear, so it counts as a breach (fail-safe). Logs each
    /// transition. Only ever consulted while the governor is enabled.
    fn actual_breach(&mut self, measured: &ArmPair<JointVec>) -> bool {
        let trip_floor = MONITOR_TRIP_FRACTION * self.d_stop;
        let threshold = if self.monitor_tripped { self.d_stop } else { trip_floor };
        let breached = match self.model.min_distance(&measured.left, &measured.right) {
            Ok(p) => p.distance < threshold,
            Err(_) => true,
        };
        if breached != self.monitor_tripped {
            if breached {
                warn!("collision MONITOR: measured clearance past {trip_floor:+.4} m (real divergence below the commanded floor), holding");
            } else {
                info!("collision MONITOR: measured clearance recovered past d_stop, resuming");
            }
            self.monitor_tripped = breached;
        }
        breached
    }

    /// Gradient-free fallback for deep penetration: with no usable gradient, allow
    /// the commanded step as long as the realized clearance does not drop below the
    /// floor (the current clearance, since we are already inside `d_stop`), else
    /// retract toward `prev`. Escape (which increases clearance) always passes;
    /// penetration never deepens; the operator is never frozen in place.
    fn govern_without_gradient(&mut self, prev: &ArmPair<JointVec>, cand: &ArmPair<JointVec>) -> ArmPair<JointVec> {
        let prev14 = concat(prev);
        let cand14 = concat(cand);
        let Some(d_now) = self.distance_at(&prev14) else { return *prev };
        let floor = d_now.min(self.d_stop);
        match self.clip_to_floor(&prev14, &cand14, floor) {
            Clip::Clear => split(&cand14),
            Clip::Clipped(q) => split(&q),
        }
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

    fn distance_at(&mut self, q: &[f64; DUAL_DOF]) -> Option<f64> {
        let pair = split(q);
        self.model.min_distance(&pair.left, &pair.right).ok().map(|p| p.distance)
    }

    /// Walk from `prev` toward `target` and return [`Clip::Clipped`] at the first
    /// point where the straight segment drops below `floor`, or [`Clip::Clear`] if
    /// every sampled point stays at or above it. Bimanual distance is not monotone
    /// along a joint-space segment, so this scans `SEGMENT_SAMPLES` interior points to
    /// bracket the first breach (an endpoint check alone can step over a pocket) and
    /// bisects within that bracket for the boundary. A failed query counts as a
    /// breach (so a model-rejected configuration is never returned), retracting
    /// conservatively. Assumes `prev` itself is clear (the caller's `d_now >= floor`).
    fn clip_to_floor(&mut self, prev: &[f64; DUAL_DOF], target: &[f64; DUAL_DOF], floor: f64) -> Clip {
        let point_at = |t: f64| -> [f64; DUAL_DOF] { std::array::from_fn(|i| prev[i] + t * (target[i] - prev[i])) };
        let mut last_clear = 0.0_f64;
        for s in 1..=SEGMENT_SAMPLES {
            let t = s as f64 / SEGMENT_SAMPLES as f64;
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
            Guard::Stopped => warn!("collision: STOP - motion halted at d={d:+.4} m between {link_a} and {link_b}"),
            Guard::Throttling => warn!("collision: throttling approach, d={d:+.4} m, pair {link_a}/{link_b}"),
            Guard::Clear => info!("collision: clear, resuming full speed"),
        }
        self.guard = next;
    }
}

/// A valid band requires finite `0 < d_stop < d_safe` (the ramp denominator
/// `d_safe - d_stop` is then positive).
fn valid_band(d_stop: f64, d_safe: f64) -> bool {
    d_stop.is_finite() && d_safe.is_finite() && d_stop > 0.0 && d_safe > d_stop
}

/// Pack a per-arm pair into one 14-vector, left then right.
fn concat(pair: &ArmPair<JointVec>) -> [f64; DUAL_DOF] {
    std::array::from_fn(|i| if i < ARM_DOF { pair.left[i] } else { pair.right[i - ARM_DOF] })
}

/// Split a 14-vector back into the per-arm pair.
fn split(q: &[f64; DUAL_DOF]) -> ArmPair<JointVec> {
    ArmPair::new(std::array::from_fn(|i| q[i]), std::array::from_fn(|i| q[ARM_DOF + i]))
}

fn dot(a: &[f64; DUAL_DOF], b: &[f64; DUAL_DOF]) -> f64 {
    a.iter().zip(b).map(|(x, y)| x * y).sum()
}

#[cfg(test)]
mod tests {
    use super::*;

    const FIXTURES: &str = env!("CARGO_MANIFEST_DIR");
    const D_STOP: f64 = 0.005;
    const D_SAFE: f64 = 0.02;
    const DT: f64 = 0.01;

    /// In-limit home; the elbow's one-sided lower limit is 0.05.
    fn home() -> ArmPair<JointVec> {
        ArmPair::new([0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0])
    }

    fn governor(enabled: bool) -> Governor {
        Governor::build(
            &format!("{FIXTURES}/openarm_v10.urdf"),
            &format!("{FIXTURES}/meshes"),
            "openarm_left_link0",
            "openarm_right_link0",
            D_STOP,
            D_SAFE,
            enabled,
        )
        .expect("build governor from vendored fixtures")
    }

    /// Both arms elbow-bent, j3 wrapping the wrists toward the centerline by `t`.
    fn wrists_inward(t: f64) -> ArmPair<JointVec> {
        ArmPair::new([0.0, 0.0, t, 0.4, 0.0, 0.0, 0.0], [0.0, 0.0, -t, 0.4, 0.0, 0.0, 0.0])
    }

    fn distance(g: &mut Governor, q: &ArmPair<JointVec>) -> f64 {
        g.model.min_distance(&q.left, &q.right).expect("finite config").distance
    }

    /// Step `from` toward `to` by at most `max` rad on each joint (a stand-in for
    /// the velocity-limited chase that feeds the governor in the real loop).
    fn chase(from: &ArmPair<JointVec>, to: &ArmPair<JointVec>, max: f64) -> ArmPair<JointVec> {
        let one = |f: &JointVec, t: &JointVec| std::array::from_fn(|i| f[i] + (t[i] - f[i]).clamp(-max, max));
        ArmPair::new(one(&from.left, &to.left), one(&from.right, &to.right))
    }

    /// Govern a chase from home toward a deeply folded pose until the clearance
    /// first drops just inside the band, so a test starts in the positive-clearance
    /// regime the barrier is designed for (not deep penetration).
    fn drive_into_band(g: &mut Governor) -> ArmPair<JointVec> {
        let target = wrists_inward(1.2);
        let mut q = home();
        for _ in 0..400 {
            if distance(g, &q) < D_SAFE {
                break;
            }
            let cand = chase(&q, &target, 0.05);
            q = g.govern(&q, &cand, &q, DT);
        }
        q
    }

    #[test]
    fn disabled_is_passthrough() {
        let mut g = governor(false);
        let deep = wrists_inward(1.2);
        assert_eq!(g.govern(&home(), &deep, &home(), DT), deep);
    }

    #[test]
    fn far_apart_is_unthrottled() {
        let mut g = governor(true);
        // Home clearance is outside the band, so any step passes untouched.
        let cand = wrists_inward(0.2);
        assert!(distance(&mut g, &home()) >= D_SAFE, "home should sit outside the band");
        assert_eq!(g.govern(&home(), &cand, &home(), DT), cand);
    }

    #[test]
    fn separating_motion_always_passes() {
        let mut g = governor(true);
        // Drive just into the band, then step back toward home: separating motion
        // (clearance increasing) is never throttled.
        let q = drive_into_band(&mut g);
        let cand = chase(&q, &home(), 0.02);
        assert_eq!(g.govern(&q, &cand, &q, DT), cand);
    }

    #[test]
    fn gradient_free_guard_allows_escape_never_deepens() {
        let mut g = governor(true);
        // Deeply folded pose, the regime where the analytic gradient can degrade.
        let deep = wrists_inward(1.5);
        let d0 = distance(&mut g, &deep);
        let floor = d0.min(D_STOP);
        // Escape toward home increases clearance: allowed, never frozen.
        let escape = chase(&deep, &home(), 0.02);
        let out = g.govern_without_gradient(&deep, &escape);
        assert_ne!(out, deep, "escape was frozen in place");
        assert!(distance(&mut g, &out) >= floor - 2e-3, "escape dropped below the floor");
        // A deeper command is held at the floor, never pushed past it.
        let deeper = chase(&deep, &wrists_inward(2.0), 0.02);
        let held = g.govern_without_gradient(&deep, &deeper);
        assert!(distance(&mut g, &held) >= floor - 2e-3, "guard let penetration deepen");
    }

    #[test]
    fn held_arm_is_not_jogged_and_commanded_joints_never_reverse() {
        let mut g = governor(true);
        let q = drive_into_band(&mut g);
        // Command only the left arm further toward the centerline (closing); hold
        // the right exactly where it is.
        let pushed = chase(&q, &wrists_inward(1.5), 0.02);
        let cand = ArmPair::new(pushed.left, q.right);
        let governed = g.govern(&q, &cand, &q, DT);
        // The held right arm must not be jogged by the barrier's correction.
        assert_eq!(governed.right, q.right, "held right arm was jogged");
        // Each commanded left joint's governed step stays within [0, commanded]:
        // same sign as the command, never larger, never reversed.
        for i in 0..ARM_DOF {
            let cmd = cand.left[i] - q.left[i];
            let gov = governed.left[i] - q.left[i];
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
        let grad_pair = g.model.distance_gradient(&q.left, &q.right).expect("gradient");
        let grad = concat(&ArmPair::new(grad_pair.grad_left, grad_pair.grad_right));
        let raw: [f64; DUAL_DOF] = std::array::from_fn(|i| ((i % 3) as f64 - 1.0) * 0.01);
        let comp = dot(&raw, &grad) / dot(&grad, &grad);
        let tangential: [f64; DUAL_DOF] = std::array::from_fn(|i| raw[i] - comp * grad[i]);
        let q14 = concat(&q);
        let cand = split(&std::array::from_fn(|i| q14[i] + tangential[i]));
        let governed = g.govern(&q, &cand, &q, DT);
        for i in 0..ARM_DOF {
            assert!((governed.left[i] - cand.left[i]).abs() < 1e-9, "left tangential joint {i} was throttled");
            assert!((governed.right[i] - cand.right[i]).abs() < 1e-9, "right tangential joint {i} was throttled");
        }
    }

    #[test]
    fn barrier_keeps_clearance_above_stop() {
        let mut g = governor(true);
        let target = wrists_inward(1.5);
        let mut q = home();
        let mut entered_band = false;
        for _ in 0..250 {
            let cand = chase(&q, &target, 0.02);
            q = g.govern(&q, &cand, &q, DT);
            let d = distance(&mut g, &q);
            entered_band |= d < D_SAFE;
            // The exact backstop holds the realized clearance at the floor, so the
            // stop distance is never breached on any tick.
            assert!(d >= D_STOP, "barrier breached: d={d:+.5}");
        }
        assert!(entered_band, "arms never approached into the band");
        // It should converge near the stop boundary, not stall far away.
        assert!(distance(&mut g, &q) < D_STOP + 4e-3, "did not settle near the stop distance");
    }

    #[test]
    fn outside_band_large_jump_is_floored() {
        let mut g = governor(true);
        // Start clear (outside the band) and command a single oversized step that
        // would vault straight past the stop floor in one tick. The outside-band
        // fast path must still run the backstop and retract to the floor.
        let start = home();
        assert!(distance(&mut g, &start) >= D_SAFE, "start should sit outside the band");
        let deep = wrists_inward(1.5);
        assert!(distance(&mut g, &deep) < D_STOP, "target should be past the stop floor");
        let governed = g.govern(&start, &deep, &start, DT);
        assert_ne!(governed, deep, "oversized step passed unfloored");
        assert!(distance(&mut g, &governed) >= D_STOP, "large jump breached the stop floor");
    }

    #[test]
    fn non_finite_candidate_holds_prev() {
        let mut g = governor(true);
        let prev = home();
        let mut bad = wrists_inward(0.2);
        bad.left[0] = f64::NAN;
        // Enabled: the up-front guard holds prev rather than steering on NaN.
        assert_eq!(g.govern(&prev, &bad, &prev, DT), prev);
        // Disabled fast path: still never passes a non-finite candidate through.
        g.set_enabled(false);
        assert_eq!(g.govern(&prev, &bad, &prev, DT), prev);
    }

    #[test]
    fn set_enabled_toggles_barrier() {
        let mut g = governor(true);
        // An in-band closing step is throttled when enabled, passed when disabled.
        let near = wrists_inward(1.0);
        let closer = wrists_inward(1.3);
        assert!(distance(&mut g, &near) < D_SAFE, "near pose should be in the band");
        assert_ne!(g.govern(&near, &closer, &near, DT), closer);
        g.set_enabled(false);
        assert_eq!(g.govern(&near, &closer, &near, DT), closer);
    }

    /// Interpolate from `lo_pose` (clearance >= target) toward `hi_pose` (clearance <
    /// target) and return the configuration whose real clearance is ~`target`, by
    /// bisection on the distance query: a measured pose at a chosen clearance for the
    /// monitor tests.
    fn config_at_distance(
        g: &mut Governor,
        lo_pose: &ArmPair<JointVec>,
        hi_pose: &ArmPair<JointVec>,
        target: f64,
    ) -> ArmPair<JointVec> {
        let lo = concat(lo_pose);
        let hi = concat(hi_pose);
        let (mut a, mut b) = (0.0_f64, 1.0_f64);
        for _ in 0..50 {
            let m = 0.5 * (a + b);
            let q = split(&std::array::from_fn(|i| lo[i] + m * (hi[i] - lo[i])));
            if distance(g, &q) >= target { a = m } else { b = m }
        }
        split(&std::array::from_fn(|i| lo[i] + a * (hi[i] - lo[i])))
    }

    #[test]
    fn monitor_holds_when_measured_breaches_while_command_is_safe() {
        let mut g = governor(true);
        // The command is a safe, far-apart step, but the MEASURED arms have actually
        // closed past the monitor floor (tracking divergence the commanded barrier
        // cannot see): the last setpoint is held regardless of the safe command.
        let prev = home();
        let cand = wrists_inward(0.2);
        let breaching = wrists_inward(2.0);
        assert!(distance(&mut g, &breaching) < MONITOR_TRIP_FRACTION * D_STOP, "measured pose must breach the monitor floor");
        assert_eq!(g.govern(&prev, &cand, &breaching, DT), prev, "monitor did not hold on a measured breach");
    }

    #[test]
    fn monitor_inert_under_good_tracking() {
        let mut g = governor(true);
        // Measured == commanded (perfect tracking), far apart: the monitor never
        // trips and the commanded step passes as it would without it.
        let prev = home();
        let cand = wrists_inward(0.2);
        assert_eq!(g.govern(&prev, &cand, &prev, DT), cand, "monitor tripped under good tracking");
    }

    #[test]
    fn monitor_hysteresis_holds_until_recovered_past_d_stop() {
        let mut g = governor(true);
        let prev = home();
        let cand = wrists_inward(0.2);
        let breaching = wrists_inward(2.0);
        assert!(distance(&mut g, &breaching) < MONITOR_TRIP_FRACTION * D_STOP);

        // A measured pose whose real clearance sits inside the hysteresis band
        // [trip floor, d_stop): below the commanded stop but above the trip floor.
        let in_band = config_at_distance(&mut g, &home(), &breaching, 0.5 * (MONITOR_TRIP_FRACTION * D_STOP + D_STOP));
        let d_band = distance(&mut g, &in_band);
        assert!((MONITOR_TRIP_FRACTION * D_STOP..D_STOP).contains(&d_band), "setup: in_band d={d_band}");

        // From clear, an in-band measurement is above the trip floor, so it does not trip.
        assert_eq!(g.govern(&prev, &cand, &in_band, DT), cand, "in-band measurement tripped from clear");
        // A real breach past the trip floor trips it.
        assert_eq!(g.govern(&prev, &cand, &breaching, DT), prev, "did not trip past the floor");
        // The same in-band measurement (recovered past the trip floor but not d_stop)
        // stays held: hysteresis.
        assert_eq!(g.govern(&prev, &cand, &in_band, DT), prev, "released before recovering past d_stop");
        // Fully recovered past d_stop: releases and the command passes.
        assert_eq!(g.govern(&prev, &cand, &prev, DT), cand, "did not release after recovery");
    }

    #[test]
    fn monitor_inert_when_disabled() {
        let mut g = governor(false);
        // Tied to the operator toggle: disabled is passthrough even though the
        // measured arms breach the floor.
        let cand = wrists_inward(0.2);
        let breaching = wrists_inward(2.0);
        assert_eq!(g.govern(&home(), &cand, &breaching, DT), cand);
    }

    /// Minimum clearance sampled along the straight segment `prev`->`cand`.
    fn segment_min(g: &mut Governor, prev: &ArmPair<JointVec>, cand: &ArmPair<JointVec>, n: usize) -> f64 {
        let p = concat(prev);
        let c = concat(cand);
        let mut m = f64::INFINITY;
        for i in 0..=n {
            let t = i as f64 / n as f64;
            m = m.min(distance(g, &split(&std::array::from_fn(|j| p[j] + t * (c[j] - p[j])))));
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
        let prev = home();
        let cand = {
            let mut p = home();
            p.left[1] = 3.2;
            p
        };
        assert!(distance(&mut g, &prev) >= D_SAFE, "home end is clear of the band");
        assert!(distance(&mut g, &cand) >= D_SAFE, "far-shoulder end is clear of the band");
        assert!(segment_min(&mut g, &prev, &cand, 128) < D_STOP, "the segment dips below the stop");

        // Trusting the (clear) endpoints would pass `cand` through; the segment scan
        // must clip it to a setpoint that is itself clear of the stop.
        let governed = g.govern(&prev, &cand, &prev, DT);
        assert_ne!(governed, cand, "a clear-ended segment with a sub-stop interior was passed unclipped");
        assert!(distance(&mut g, &governed) >= D_STOP - 1e-6, "the clipped setpoint is below the stop");
    }
}
