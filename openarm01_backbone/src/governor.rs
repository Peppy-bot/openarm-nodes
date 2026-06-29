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
//! control-rate step and absorbed by the exact line-search backstop, which
//! guarantees the realized clearance never falls below `d_stop` regardless of
//! curvature.

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

/// Where the governor last sat, so throttle/stop/clear are logged on transition
/// rather than at the control rate.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Guard {
    Clear,
    Throttling,
    Stopped,
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
        Ok(Self { model, d_stop, d_safe, enabled, guard: Guard::Clear })
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
    pub fn govern(&mut self, prev: &ArmPair<JointVec>, cand: &ArmPair<JointVec>, dt: f64) -> ArmPair<JointVec> {
        if !self.enabled {
            return *cand;
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
        // Outside the influence zone: no closing constraint, take the full step.
        if d_now >= self.d_safe {
            self.log_transition(Guard::Clear, d_now, &link_a, &link_b);
            return *cand;
        }

        let prev14 = concat(prev);
        let cand14 = concat(cand);
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
        // carry the step past the stop floor, so verify the realized clearance and
        // retract along prev->governed until it is back at the floor. The floor is
        // d_stop, or the current clearance if already inside it (so an in-penetration
        // recovery is never forced to close further).
        let floor = d_now.min(self.d_stop);
        match self.distance_at(&governed14) {
            Some(d) if d >= floor => {}
            Some(_) => {
                governed14 = self.retract_to_floor(&prev14, &governed14, floor);
                limited = true;
            }
            None => return *prev,
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
        let governed14 = match self.distance_at(&cand14) {
            Some(d) if d >= floor => cand14,
            Some(_) => self.retract_to_floor(&prev14, &cand14, floor),
            None => return *prev,
        };
        split(&governed14)
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

    /// Retract along `prev`->`target` to the furthest fraction whose actual
    /// clearance is at least `floor`, by bisection on the real distance query
    /// (exact up to the bisection resolution). Assumes `d(prev) >= floor` and
    /// `d(target) < floor`; a failed query retracts further, fail-safe.
    fn retract_to_floor(&mut self, prev: &[f64; DUAL_DOF], target: &[f64; DUAL_DOF], floor: f64) -> [f64; DUAL_DOF] {
        let mut lo = 0.0_f64;
        let mut hi = 1.0_f64;
        for _ in 0..12 {
            let mid = 0.5 * (lo + hi);
            let q: [f64; DUAL_DOF] = std::array::from_fn(|i| prev[i] + mid * (target[i] - prev[i]));
            match self.distance_at(&q) {
                Some(d) if d >= floor => lo = mid,
                _ => hi = mid,
            }
        }
        std::array::from_fn(|i| prev[i] + lo * (target[i] - prev[i]))
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
            q = g.govern(&q, &cand, DT);
        }
        q
    }

    #[test]
    fn disabled_is_passthrough() {
        let mut g = governor(false);
        let deep = wrists_inward(1.2);
        assert_eq!(g.govern(&home(), &deep, DT), deep);
    }

    #[test]
    fn far_apart_is_unthrottled() {
        let mut g = governor(true);
        // Home clearance is outside the band, so any step passes untouched.
        let cand = wrists_inward(0.2);
        assert!(distance(&mut g, &home()) >= D_SAFE, "home should sit outside the band");
        assert_eq!(g.govern(&home(), &cand, DT), cand);
    }

    #[test]
    fn separating_motion_always_passes() {
        let mut g = governor(true);
        // Drive just into the band, then step back toward home: separating motion
        // (clearance increasing) is never throttled.
        let q = drive_into_band(&mut g);
        let cand = chase(&q, &home(), 0.02);
        assert_eq!(g.govern(&q, &cand, DT), cand);
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
        let governed = g.govern(&q, &cand, DT);
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
        let governed = g.govern(&q, &cand, DT);
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
            q = g.govern(&q, &cand, DT);
            let d = distance(&mut g, &q);
            entered_band |= d < D_SAFE;
            // Small linearization slack below d_stop; the next tick recovers.
            assert!(d >= D_STOP - 1e-3, "barrier breached: d={d:+.5}");
        }
        assert!(entered_band, "arms never approached into the band");
        // It should converge near the stop boundary, not stall far away.
        assert!(distance(&mut g, &q) < D_STOP + 4e-3, "did not settle near the stop distance");
    }

    #[test]
    fn set_enabled_toggles_barrier() {
        let mut g = governor(true);
        // An in-band closing step is throttled when enabled, passed when disabled.
        let near = wrists_inward(1.0);
        let closer = wrists_inward(1.3);
        assert!(distance(&mut g, &near) < D_SAFE, "near pose should be in the band");
        assert_ne!(g.govern(&near, &closer, DT), closer);
        g.set_enabled(false);
        assert_eq!(g.govern(&near, &closer, DT), closer);
    }
}
