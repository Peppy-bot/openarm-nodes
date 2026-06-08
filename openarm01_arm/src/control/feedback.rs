//! Feedback cadence and failure policy shared by the motion states.

use std::time::{Duration, Instant};

/// Publish feedback at the goal's requested period, and surface only the first
/// publish failure of a motion (so the caller warns once, then stays quiet).
pub(super) struct Feedback {
    period: Duration,
    last: Instant,
    failures: u32,
}

impl Feedback {
    pub(super) fn new(period: Duration) -> Self {
        Self { period, last: Instant::now(), failures: 0 }
    }

    /// Whether a feedback period has elapsed since the last publish; advances the
    /// cadence when it has, so the caller publishes iff this returns true.
    pub(super) fn should_publish(&mut self, now: Instant) -> bool {
        if now.duration_since(self.last) < self.period {
            return false;
        }
        self.last = now;
        true
    }

    /// Filter a publish result down to the error the caller should warn about:
    /// the first failure of this motion. Repeats are swallowed.
    pub(super) fn first_failure<E>(&mut self, result: Result<(), E>) -> Option<E> {
        let e = result.err()?;
        self.failures += 1;
        (self.failures == 1).then_some(e)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const PERIOD: Duration = Duration::from_millis(100);

    #[test]
    fn publishes_once_per_period() {
        let start = Instant::now();
        let mut f = Feedback { period: PERIOD, last: start, failures: 0 };
        assert!(!f.should_publish(start + PERIOD / 2));
        assert!(f.should_publish(start + PERIOD));
        // Cadence advanced: the same instant does not publish twice.
        assert!(!f.should_publish(start + PERIOD));
        assert!(f.should_publish(start + 2 * PERIOD));
    }

    #[test]
    fn surfaces_only_the_first_failure() {
        let mut f = Feedback::new(PERIOD);
        assert!(f.first_failure(Ok::<(), &str>(())).is_none());
        assert_eq!(f.first_failure(Err::<(), &str>("boom")), Some("boom"));
        assert!(f.first_failure(Err::<(), &str>("again")).is_none());
        // Success between failures does not reset the once-only policy.
        assert!(f.first_failure(Ok::<(), &str>(())).is_none());
        assert!(f.first_failure(Err::<(), &str>("third")).is_none());
    }
}
