//! Per-component motor health for the UI. Consumes every producer bound to
//! the motor_health slot (zero_or_more; the launcher wires the arm and
//! gripper instances), parses each report into per-motor readings, and hands
//! it to the owner keyed by side, so the panel can badge each motor and
//! banner an overloading component.

use std::sync::Arc;
use std::time::{Instant, SystemTime};

use peppygen::NodeRunner;
use peppygen::consumed_topics::motor_health::motor_health;
use peppylib::messaging::ProducerRef;
use peppylib::runtime::CancellationToken;
use tokio::sync::mpsc;
use tracing::error;

use crate::consumer;
use crate::owner::Feedback;
use crate::state::{
    HEALTH_STALE_AFTER, HealthLevel, HealthReport, MotorHealthReading, Side,
    health_level_from_wire, parse_stamped_validity,
};

/// The component names this panel renders. The producers extend these with
/// the motor on the alert wire ("left arm j2"), correlated by prefix.
const SOURCE_LEFT_ARM: &str = "left arm";
const SOURCE_RIGHT_ARM: &str = "right arm";
const SOURCE_LEFT_GRIPPER: &str = "left gripper";
const SOURCE_RIGHT_GRIPPER: &str = "right gripper";

/// Whether this deployment binds any motor_health producer.
pub fn available(runner: &NodeRunner) -> bool {
    !motor_health::bound_producers(runner).is_empty()
}

impl consumer::Subscription for motor_health::Subscription {
    type Message = motor_health::Message;
    async fn recv(&mut self) -> peppygen::Result<Option<(ProducerRef, Self::Message)>> {
        self.next().await
    }
}

pub async fn run(
    runner: Arc<NodeRunner>,
    feedback: mpsc::Sender<Feedback>,
    token: CancellationToken,
) {
    let subscription = match motor_health::subscribe(&runner).await {
        Ok(subscription) => subscription,
        Err(e) => {
            error!(error = %e, "motor_health subscribe");
            return;
        }
    };
    consumer::forward_parsed(
        "motor_health",
        token,
        feedback,
        subscription,
        // An unresolved daemon clock cannot certify a stamp's age, so the
        // report drops on the same throttled-warn path as a malformed one.
        |msg| parse_report(msg, consumer::clock_now()?, Instant::now()),
    )
    .await;
}

/// Parse one wire report into the owner feedback it routes to: a source this
/// panel has a slot for, one level per motor of that component each a defined
/// severity, reading vectors either absent (empty) or motor-count-length with
/// finite values, and a stamp not already past the aging window.
///
/// The contract names components by string, so a report about hardware this
/// panel does not render (another robot's limb) is dropped here rather than
/// shown in the wrong slot.
fn parse_report(
    msg: &motor_health::Message,
    clock_now: SystemTime,
    received_at: Instant,
) -> Result<Feedback, String> {
    let arm = |side| -> Result<Feedback, String> {
        Ok(Feedback::MotorHealth {
            side,
            health: Box::new(parse_component(msg, clock_now, received_at)?),
        })
    };
    let gripper = |side| -> Result<Feedback, String> {
        Ok(Feedback::GripperMotorHealth {
            side,
            health: parse_component(msg, clock_now, received_at)?,
        })
    };
    match msg.source.as_str() {
        SOURCE_LEFT_ARM => arm(Side::Left),
        SOURCE_RIGHT_ARM => arm(Side::Right),
        SOURCE_LEFT_GRIPPER => gripper(Side::Left),
        SOURCE_RIGHT_GRIPPER => gripper(Side::Right),
        other => Err(format!("report from an unrendered source {other:?}")),
    }
}

/// One component's report, transposed from the wire's struct-of-arrays into
/// per-motor readings at this parse boundary so everything downstream works
/// motor-wise.
fn parse_component<const MOTORS: usize>(
    msg: &motor_health::Message,
    clock_now: SystemTime,
    received_at: Instant,
) -> Result<HealthReport<MOTORS>, String> {
    let validity = parse_stamped_validity(msg.stamp, clock_now, received_at, HEALTH_STALE_AFTER)?;
    let wire_levels: [u8; MOTORS] = msg
        .level
        .as_slice()
        .try_into()
        .map_err(|_| format!("expected {MOTORS} levels, got {}", msg.level.len()))?;
    let mut levels = [HealthLevel::Nominal; MOTORS];
    for (parsed, wire) in levels.iter_mut().zip(&wire_levels) {
        *parsed = health_level_from_wire(*wire).ok_or_else(|| format!("undefined level {wire}"))?;
    }
    let vector = |values, name| parse_reading_vector::<MOTORS>(values, name);
    let effort_fraction = vector(&msg.effort_fraction, "effort_fraction")?;
    let effort_fraction_sustained =
        vector(&msg.effort_fraction_sustained, "effort_fraction_sustained")?;
    let effort_fraction_peak = vector(&msg.effort_fraction_peak, "effort_fraction_peak")?;
    let driver_temp_c = vector(&msg.driver_temp_c, "driver_temp_c")?;
    let winding_temp_c = vector(&msg.winding_temp_c, "winding_temp_c")?;
    let readings = std::array::from_fn(|i| MotorHealthReading {
        level: levels[i],
        effort_fraction: effort_fraction.map(|v| v[i]),
        effort_fraction_sustained: effort_fraction_sustained.map(|v| v[i]),
        effort_fraction_peak: effort_fraction_peak.map(|v| v[i]),
        driver_temp_c: driver_temp_c.map(|v| v[i]),
        winding_temp_c: winding_temp_c.map(|v| v[i]),
    });
    Ok(HealthReport { readings, validity })
}

/// A reading vector is empty (the producer senses nothing) or exactly one
/// finite value per motor.
fn parse_reading_vector<const MOTORS: usize>(
    values: &[f64],
    name: &str,
) -> Result<Option<[f64; MOTORS]>, String> {
    if values.is_empty() {
        return Ok(None);
    }
    let full: [f64; MOTORS] = values
        .try_into()
        .map_err(|_| format!("expected 0 or {MOTORS} {name} values, got {}", values.len()))?;
    if !full.iter().all(|v| v.is_finite()) {
        return Err(format!("non-finite {name}"));
    }
    Ok(Some(full))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state::{ARM_DOF, ArmHealth, GripperHealth, STAMP_SKEW_ALLOWANCE};
    use std::time::Duration;

    fn msg(source: &str) -> motor_health::Message {
        motor_health::Message {
            stamp: SystemTime::now(),
            source: source.to_string(),
            level: vec![0; ARM_DOF],
            effort_fraction: (0..ARM_DOF).map(|i| 0.10 + 0.01 * i as f64).collect(),
            effort_fraction_sustained: (0..ARM_DOF).map(|i| 0.20 + 0.01 * i as f64).collect(),
            effort_fraction_peak: (0..ARM_DOF).map(|i| 0.30 + 0.01 * i as f64).collect(),
            driver_temp_c: (0..ARM_DOF).map(|i| 40.0 + i as f64).collect(),
            winding_temp_c: (0..ARM_DOF).map(|i| 35.0 + i as f64).collect(),
        }
    }

    /// Parse against a clock equal to the stamp, so only shape can fail.
    fn parse_fresh(msg: &motor_health::Message) -> Result<Feedback, String> {
        parse_report(msg, msg.stamp, Instant::now())
    }

    /// Unwrap an arm parse, panicking on any other routing.
    fn arm(feedback: Feedback) -> (Side, ArmHealth) {
        match feedback {
            Feedback::MotorHealth { side, health } => (side, *health),
            _ => panic!("routed off the arm slot"),
        }
    }

    fn gripper(feedback: Feedback) -> (Side, GripperHealth) {
        match feedback {
            Feedback::GripperMotorHealth { side, health } => (side, health),
            _ => panic!("routed off the gripper slot"),
        }
    }

    #[test]
    fn a_full_report_transposes_every_vector_per_motor() {
        let (side, health) = arm(parse_fresh(&msg(SOURCE_RIGHT_ARM)).unwrap());
        assert_eq!(side, Side::Right);
        for (i, reading) in health.readings.iter().enumerate() {
            assert_eq!(reading.level, HealthLevel::Nominal);
            assert_eq!(reading.effort_fraction, Some(0.10 + 0.01 * i as f64));
            assert_eq!(
                reading.effort_fraction_sustained,
                Some(0.20 + 0.01 * i as f64)
            );
            assert_eq!(reading.effort_fraction_peak, Some(0.30 + 0.01 * i as f64));
            assert_eq!(reading.driver_temp_c, Some(40.0 + i as f64));
            assert_eq!(reading.winding_temp_c, Some(35.0 + i as f64));
        }
        assert_eq!(health.worst(), HealthLevel::Nominal);
    }

    #[test]
    fn a_not_sensed_report_parses_with_every_reading_absent() {
        let mut m = msg(SOURCE_LEFT_ARM);
        m.effort_fraction = vec![];
        m.effort_fraction_sustained = vec![];
        m.effort_fraction_peak = vec![];
        m.driver_temp_c = vec![];
        m.winding_temp_c = vec![];
        let (side, health) = arm(parse_fresh(&m).unwrap());
        assert_eq!(side, Side::Left);
        for reading in &health.readings {
            assert_eq!(reading.effort_fraction, None);
            assert_eq!(reading.effort_fraction_sustained, None);
            assert_eq!(reading.effort_fraction_peak, None);
            assert_eq!(reading.driver_temp_c, None);
            assert_eq!(reading.winding_temp_c, None);
        }
    }

    #[test]
    fn mixed_sensing_is_legal_per_vector() {
        // A producer that measures torque but not temperature sends empty
        // temperature vectors alongside full torque ones.
        let mut m = msg(SOURCE_LEFT_ARM);
        m.winding_temp_c = vec![];
        let (_, health) = arm(parse_fresh(&m).unwrap());
        assert_eq!(health.readings[2].winding_temp_c, None);
        assert_eq!(health.readings[2].driver_temp_c, Some(42.0));
        assert_eq!(health.readings[2].effort_fraction, Some(0.10 + 0.01 * 2.0));
    }

    #[test]
    fn worst_reports_the_most_severe_joint() {
        let mut m = msg(SOURCE_LEFT_ARM);
        m.level[4] = 2;
        let (_, health) = arm(parse_fresh(&m).unwrap());
        assert_eq!(health.worst(), HealthLevel::Critical);
    }

    #[test]
    fn a_silent_joint_does_not_cost_the_arm_its_whole_report() {
        // A motor that stops answering is exactly when the panel is needed;
        // refusing the message would blank the six joints still talking.
        let mut m = msg(SOURCE_LEFT_ARM);
        m.level[5] = 4;
        let (_, health) = arm(parse_fresh(&m).unwrap());
        assert_eq!(health.readings[5].level, HealthLevel::NotReporting);
        assert_eq!(health.readings[0].level, HealthLevel::Nominal);
        assert_eq!(health.worst(), HealthLevel::NotReporting);
    }

    #[test]
    fn silence_outranks_every_condition_a_motor_can_report() {
        let mut m = msg(SOURCE_LEFT_ARM);
        m.level = vec![3; ARM_DOF];
        m.level[2] = 4;
        let (_, health) = arm(parse_fresh(&m).unwrap());
        assert_eq!(health.worst(), HealthLevel::NotReporting);
    }

    #[test]
    fn a_pre_aged_stamp_rejects_the_report() {
        // A backlogged consumer must not re-stamp a stale report as fresh.
        let m = msg(SOURCE_LEFT_ARM);
        let just_inside = m.stamp + HEALTH_STALE_AFTER + STAMP_SKEW_ALLOWANCE;
        assert!(parse_report(&m, just_inside, Instant::now()).is_ok());
        let past = just_inside + Duration::from_millis(1);
        assert!(parse_report(&m, past, Instant::now()).is_err());
    }

    #[test]
    fn malformed_reports_reject() {
        let mut short_levels = msg(SOURCE_LEFT_ARM);
        short_levels.level = vec![0; ARM_DOF - 1];
        assert!(parse_fresh(&short_levels).is_err());

        let mut bad_level = msg(SOURCE_LEFT_ARM);
        bad_level.level[0] = 5;
        assert!(parse_fresh(&bad_level).is_err());

        let mut unrendered = msg(SOURCE_LEFT_ARM);
        unrendered.source = "left forklift".to_string();
        assert!(parse_fresh(&unrendered).is_err());
    }

    #[test]
    fn wrong_length_and_non_finite_readings_reject_per_vector() {
        type Field = fn(&mut motor_health::Message) -> &mut Vec<f64>;
        let fields: [(&str, Field); 5] = [
            ("effort_fraction", |m| &mut m.effort_fraction),
            ("effort_fraction_sustained", |m| {
                &mut m.effort_fraction_sustained
            }),
            ("effort_fraction_peak", |m| &mut m.effort_fraction_peak),
            ("driver_temp_c", |m| &mut m.driver_temp_c),
            ("winding_temp_c", |m| &mut m.winding_temp_c),
        ];
        for (name, field) in fields {
            let mut short = msg(SOURCE_LEFT_ARM);
            field(&mut short).truncate(3);
            assert!(parse_fresh(&short).is_err(), "short {name} must reject");
            for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
                let mut poisoned = msg(SOURCE_LEFT_ARM);
                field(&mut poisoned)[2] = bad;
                assert!(
                    parse_fresh(&poisoned).is_err(),
                    "{bad} in {name} must reject"
                );
            }
        }
    }

    #[test]
    fn a_gripper_report_parses_as_its_single_motor() {
        let m = motor_health::Message {
            stamp: SystemTime::now(),
            source: SOURCE_LEFT_GRIPPER.to_string(),
            level: vec![1],
            effort_fraction: vec![0.4],
            effort_fraction_sustained: vec![0.35],
            effort_fraction_peak: vec![0.2],
            driver_temp_c: vec![41.0],
            winding_temp_c: vec![37.0],
        };
        let (side, health) = gripper(parse_fresh(&m).unwrap());
        assert_eq!(side, Side::Left);
        let [reading] = health.readings;
        assert_eq!(reading.level, HealthLevel::Warning);
        assert_eq!(reading.effort_fraction, Some(0.4));
        assert_eq!(reading.effort_fraction_sustained, Some(0.35));
        assert_eq!(reading.effort_fraction_peak, Some(0.2));
    }

    #[test]
    fn a_gripper_report_with_arm_shaped_vectors_rejects() {
        // Seven levels under a one-motor source is a producer bug, not a
        // layout to guess at.
        let mut m = msg(SOURCE_RIGHT_GRIPPER);
        assert!(parse_fresh(&m).is_err());
        m.level = vec![0];
        assert!(
            parse_fresh(&m).is_err(),
            "readings must be empty or exactly one"
        );
    }
}
