//! Freshness of follower state: how old a measured sample may be before the
//! backbone must stop acting on it (streaming setpoints, running trajectories,
//! or accepting motion goals).

use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant};

use tokio::sync::watch;

/// Floor on the staleness limit, in control periods. A healthy follower
/// delivers state every period (state_rate == control_rate in this stack), so
/// the floor tolerates four consecutive missed deliveries plus scheduler
/// jitter (worst observed pacer overrun is single-digit milliseconds) before
/// declaring the side stale; a stream that quiet is broken, not jittery.
const STALE_FLOOR_PERIODS: u32 = 4;

/// How old a measured sample may be before its side is stale: the time the
/// world would need to traverse the governor's safety band (`d_safe - d_stop`)
/// at the commanded end-effector speed cap, floored by
/// [`STALE_FLOOR_PERIODS`] control periods. Acting on older data means the
/// governor could be reasoning an entire band-width away from reality. Every
/// term is an existing operator parameter or the control rate, so the limit
/// follows live governor retunes.
pub fn stale_limit(
    d_stop: f64,
    d_safe: f64,
    max_ee_velocity_m_s: f64,
    cycle_period: Duration,
) -> Duration {
    let band_m = (d_safe - d_stop).max(0.0);
    let physics = Duration::from_secs_f64(band_m / max_ee_velocity_m_s);
    physics.max(cycle_period * STALE_FLOOR_PERIODS)
}

/// The live staleness limit, written by the coordinator (the owner of the live
/// governor parameters) and read by the goal-acceptance closures.
#[derive(Clone)]
pub struct SharedStaleLimit(Arc<AtomicU64>);

impl SharedStaleLimit {
    pub fn new(initial: Duration) -> Self {
        Self(Arc::new(AtomicU64::new(initial.as_nanos() as u64)))
    }

    pub fn set(&self, limit: Duration) {
        self.0.store(limit.as_nanos() as u64, Ordering::Relaxed);
    }

    pub fn get(&self) -> Duration {
        Duration::from_nanos(self.0.load(Ordering::Relaxed))
    }
}

/// A goal-acceptance probe for one limb: fresh only when the limb has reported
/// state within the live staleness limit. `received_at` extracts the ingestion
/// instant from the limb's stored state type.
pub struct FreshnessProbe<T: Copy> {
    latest: watch::Receiver<Option<T>>,
    limit: SharedStaleLimit,
    received_at: fn(&T) -> Instant,
}

impl<T: Copy> FreshnessProbe<T> {
    pub fn new(
        latest: watch::Receiver<Option<T>>,
        limit: SharedStaleLimit,
        received_at: fn(&T) -> Instant,
    ) -> Self {
        Self {
            latest,
            limit,
            received_at,
        }
    }

    pub fn is_fresh(&self) -> bool {
        self.latest
            .borrow()
            .as_ref()
            .is_some_and(|state| (self.received_at)(state).elapsed() <= self.limit.get())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const CYCLE_100HZ: Duration = Duration::from_millis(10);

    #[test]
    fn limit_is_the_band_traversal_time_when_it_dominates() {
        // (0.02 - 0.005) / 0.25 = 60 ms > 4 * 10 ms floor.
        let limit = stale_limit(0.005, 0.02, 0.25, CYCLE_100HZ);
        assert_eq!(limit, Duration::from_millis(60));
    }

    #[test]
    fn limit_floors_at_four_control_periods() {
        // A fast EE cap shrinks the physics bound below the jitter floor.
        let limit = stale_limit(0.005, 0.02, 10.0, CYCLE_100HZ);
        assert_eq!(limit, CYCLE_100HZ * 4);
    }

    #[test]
    fn degenerate_band_falls_back_to_the_floor() {
        let limit = stale_limit(0.02, 0.02, 0.25, CYCLE_100HZ);
        assert_eq!(limit, CYCLE_100HZ * 4);
        let inverted = stale_limit(0.03, 0.02, 0.25, CYCLE_100HZ);
        assert_eq!(inverted, CYCLE_100HZ * 4);
    }

    #[test]
    fn shared_limit_round_trips() {
        let shared = SharedStaleLimit::new(Duration::from_millis(60));
        assert_eq!(shared.get(), Duration::from_millis(60));
        shared.set(Duration::from_millis(40));
        assert_eq!(shared.get(), Duration::from_millis(40));
    }

    #[test]
    fn probe_tracks_watch_and_age() {
        #[derive(Clone, Copy)]
        struct S(Instant);
        let (tx, rx) = watch::channel(None::<S>);
        let limit = SharedStaleLimit::new(Duration::from_millis(50));
        let probe = FreshnessProbe::new(rx, limit, |s: &S| s.0);
        assert!(!probe.is_fresh(), "absent state is never fresh");
        tx.send_replace(Some(S(Instant::now())));
        assert!(probe.is_fresh());
        tx.send_replace(Some(S(Instant::now() - Duration::from_millis(200))));
        assert!(!probe.is_fresh(), "an old sample is stale");
    }
}
