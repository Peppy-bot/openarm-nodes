//! Operator alerts for the UI. Consumes every producer bound to the alerts
//! slot (zero_or_more), validates each message, and hands it to the owner,
//! which holds one alert per (source, kind) and drops it on a severity-0
//! clear or when its producer stops re-emitting.

use std::sync::Arc;
use std::time::{Instant, SystemTime};

use peppygen::NodeRunner;
use peppygen::consumed_topics::alerts::alerts;
use peppylib::messaging::ProducerRef;
use peppylib::runtime::CancellationToken;
use tokio::sync::mpsc;
use tracing::error;

use crate::consumer;
use crate::owner::Feedback;
use crate::state::{ALERT_STALE_AFTER, Alert, parse_stamped_validity};

/// The alert contract's severity ceiling: 0 clear, 1 warning, 2 critical,
/// 3 fault.
const MAX_ALERT_SEVERITY: u8 = 3;

/// Whether this deployment binds any alerts producer.
pub fn available(runner: &NodeRunner) -> bool {
    !alerts::bound_producers(runner).is_empty()
}

impl consumer::Subscription for alerts::Subscription {
    type Message = alerts::Message;
    async fn recv(&mut self) -> peppygen::Result<Option<(ProducerRef, Self::Message)>> {
        self.next().await
    }
}

pub async fn run(
    runner: Arc<NodeRunner>,
    feedback: mpsc::Sender<Feedback>,
    token: CancellationToken,
) {
    let subscription = match alerts::subscribe(&runner).await {
        Ok(subscription) => subscription,
        Err(e) => {
            error!(error = %e, "alerts subscribe");
            return;
        }
    };
    consumer::forward_parsed(
        "alerts",
        token,
        feedback,
        subscription,
        // An unresolved daemon clock cannot certify a stamp's age, so the
        // alert drops on the same throttled-warn path as a malformed one.
        |msg| parse_alert(msg, consumer::clock_now()?, Instant::now()).map(Feedback::Alert),
    )
    .await;
}

/// Parse one wire alert: a non-empty identity, a defined severity, and a
/// stamp not already past the aging window (a backlogged consumer must not
/// re-stamp a stale alert as fresh).
fn parse_alert(
    msg: &alerts::Message,
    clock_now: SystemTime,
    received_at: Instant,
) -> Result<Alert, String> {
    if msg.source.is_empty() || msg.kind.is_empty() {
        return Err("empty source or kind".to_string());
    }
    if msg.severity > MAX_ALERT_SEVERITY {
        return Err(format!("undefined severity {}", msg.severity));
    }
    Ok(Alert {
        source: msg.source.clone(),
        kind: msg.kind.clone(),
        severity: msg.severity,
        message: msg.message.clone(),
        validity: parse_stamped_validity(msg.stamp, clock_now, received_at, ALERT_STALE_AFTER)?,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state::STAMP_SKEW_ALLOWANCE;
    use std::time::Duration;

    fn msg(severity: u8) -> alerts::Message {
        alerts::Message {
            stamp: SystemTime::now(),
            source: "left arm j2".to_string(),
            kind: "motor_overload".to_string(),
            severity,
            message: "holding 93% of rated torque".to_string(),
        }
    }

    /// Parse against a clock equal to the stamp, so only shape can fail.
    fn parse_fresh(msg: &alerts::Message) -> Result<Alert, String> {
        parse_alert(msg, msg.stamp, Instant::now())
    }

    #[test]
    fn well_formed_alerts_parse_including_clears() {
        let raised = parse_fresh(&msg(2)).unwrap();
        assert_eq!(raised.severity, 2);
        assert_eq!(raised.source, "left arm j2");
        let cleared = parse_fresh(&msg(0)).unwrap();
        assert_eq!(cleared.severity, 0);
    }

    #[test]
    fn malformed_alerts_reject() {
        let mut empty_source = msg(1);
        empty_source.source = String::new();
        assert!(parse_fresh(&empty_source).is_err());

        let mut empty_kind = msg(1);
        empty_kind.kind = String::new();
        assert!(parse_fresh(&empty_kind).is_err());

        assert!(parse_fresh(&msg(4)).is_err());
    }

    #[test]
    fn a_pre_aged_stamp_rejects_the_alert() {
        let m = msg(2);
        let just_inside = m.stamp + ALERT_STALE_AFTER + STAMP_SKEW_ALLOWANCE;
        assert!(parse_alert(&m, just_inside, Instant::now()).is_ok());
        let past = just_inside + Duration::from_millis(1);
        assert!(parse_alert(&m, past, Instant::now()).is_err());
    }
}
