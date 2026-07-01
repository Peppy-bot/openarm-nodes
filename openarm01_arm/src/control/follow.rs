//! The ambient follow state: the arm's default whenever no move goal is
//! running. Each tick it either preempts into a pending move (resuming follow
//! when the move ends), chases the locked joint stream, or holds the last
//! setpoint when nothing is streaming. Cartesian intent stays upstream: a
//! task-space commander solves IK and streams joints, so the arm's real-time
//! loop is a pure joint-space servo.
//!
//! Exactly one producer drives the arm at a time. The first instance seen while
//! unlocked claims the lock and holds it until that producer goes quiet for
//! `stream_timeout`, after which the next fresh command re-arms. A fresh command
//! from a different instance violates the single-source contract; it is ignored
//! and warned once per streak. The setpoint chases the locked target at the
//! per-joint velocity limits (clamped to the joint limits), so the first command,
//! a jump, or a producer switch is a bounded-velocity catch-up rather than a
//! torque spike, and the velocity feedforward is the chase velocity itself (so it
//! is consistent with the motion and zero once the setpoint has caught up).

use std::time::{Duration, Instant};

use tracing::{info, warn};

use super::cartesian_move::CartesianMove;
use super::chase::{chase_step, clamp_to_limits};
use super::joint_move::JointMove;
use super::{Mode, TickIo, ZERO, command, fmt_joints};
use crate::actions::Goal;
use crate::stream::JointCommand;
use crate::{ARM_DOF, JointVec};
use peppylib::messaging::ProducerRef;
use srs_model::Jacobian;
use srs_model::nalgebra::SVector;

/// The locked joint producer and the chase state it drives.
struct Lock {
    producer: ProducerRef,
    /// Joint target the setpoint chases, refreshed (clamped) from each new command.
    target: JointVec,
    last_seq: u64,
    /// When a matching command was last consumed; the lock releases when this
    /// ages past `stream_timeout`.
    last_fresh: Instant,
}

pub(super) struct Follow {
    /// Setpoint commanded each tick: chases the lock's target, or held when there
    /// is no active producer.
    setpoint: JointVec,
    lock: Option<Lock>,
    /// Suppresses repeat contract-violation warnings while a conflict persists.
    conflict_warned: bool,
}

impl Follow {
    /// Enter follow holding `setpoint`, with no producer yet (re-armed on the next
    /// fresh command). Used at startup and after every move.
    pub(super) fn idle(setpoint: JointVec) -> Self {
        Self {
            setpoint,
            lock: None,
            conflict_warned: false,
        }
    }

    /// Preempt into a pending move, else chase the locked producer (or hold), and
    /// command the motors once.
    pub(super) async fn tick(mut self, io: &mut TickIo<'_>) -> Mode {
        match io.goals.try_recv() {
            Ok(Goal::JointMove(g)) => return Mode::JointMove(JointMove::start(g, io)),
            Ok(Goal::CartesianMove(g)) => {
                return match CartesianMove::start(g, io).await {
                    Some(m) => Mode::CartesianMove(m),
                    // Goal rejected at start (unreachable path), already completed
                    // there; stay in Follow.
                    None => Mode::Follow(self),
                };
            }
            Err(_) => {}
        }

        self.arbitrate(io);

        let dq_des = match &self.lock {
            Some(lock) => {
                let dt = io.cfg.cycle_period.as_secs_f64();
                // Rate-limit to the motor maxima for continuity (no setpoint jump
                // that would slam the PD), then scale the whole step so the hand's
                // linear speed stays under the cap. A step null to the linear
                // Jacobian (e.g. self-motion) is left at the motor rate.
                let stepped = chase_step(
                    &self.setpoint,
                    &lock.target,
                    &io.cfg.max_joint_velocity_rad_s,
                    dt,
                );
                let next = cap_ee_speed(
                    &self.setpoint,
                    &stepped,
                    &io.jacobian,
                    io.cfg.max_ee_velocity_m_s,
                    dt,
                );
                let dq = std::array::from_fn(|i| (next[i] - self.setpoint[i]) / dt);
                self.setpoint = next;
                dq
            }
            None => ZERO,
        };
        command(io, &self.setpoint, &dq_des);
        Mode::Follow(self)
    }

    /// Acquire, maintain, or release the stream lock for this tick, refreshing the
    /// chase target from a fresh matching command and surfacing a contract
    /// violation (a second producer) as a rate-limited warning.
    fn arbitrate(&mut self, io: &TickIo<'_>) {
        let now = io.now;
        let timeout = io.cfg.stream_timeout;
        let joint = io.joint_stream.borrow().clone();

        let mut lock = self.lock.take();
        let conflict;
        if let Some(l) = lock.as_mut() {
            conflict = maintain(l, &joint, io);
            let stale = now.duration_since(l.last_fresh) > timeout;
            if stale {
                info!(
                    "openarm01_arm: joint stream quiet for {timeout:?}, releasing lock; holding at {}",
                    fmt_joints(&self.setpoint)
                );
                lock = None;
            }
        } else if let Some(jc) = joint.as_ref().filter(|c| fresh(c.recv_at, now, timeout)) {
            lock = Some(acquire(jc, io));
            conflict = false;
        } else {
            conflict = false;
        }
        self.lock = lock;
        self.warn_conflict(conflict);
    }

    /// Emit the contract-violation warning once per streak, naming the locked
    /// producer the foreign command is being ignored in favor of.
    fn warn_conflict(&mut self, conflict: bool) {
        if !conflict {
            self.conflict_warned = false;
            return;
        }
        if self.conflict_warned {
            return;
        }
        let locked = self.lock.as_ref().map_or_else(
            || "no producer".to_string(),
            |l| format!("{:?}", l.producer),
        );
        warn!(
            "openarm01_arm: a second joint producer is publishing while locked to {locked}; ignoring it (single-source contract violated)"
        );
        self.conflict_warned = true;
    }
}

/// Lock onto a producer, taking its clamped positions as the first target.
fn acquire(cmd: &JointCommand, io: &TickIo<'_>) -> Lock {
    info!(
        "openarm01_arm: following joint stream from {:?} (at {})",
        cmd.producer,
        fmt_joints(&io.q)
    );
    Lock {
        producer: cmd.producer.clone(),
        target: clamp_to_limits(&cmd.positions, &io.cfg.limits),
        last_seq: cmd.seq,
        last_fresh: io.now,
    }
}

/// Update the lock's target from a fresh matching command and report whether a
/// fresh command from a different producer (a contract violation) was seen.
fn maintain(lock: &mut Lock, joint: &Option<JointCommand>, io: &TickIo<'_>) -> bool {
    if let Some(jc) = joint
        && jc.producer == lock.producer
        && jc.seq != lock.last_seq
    {
        lock.target = clamp_to_limits(&jc.positions, &io.cfg.limits);
        lock.last_seq = jc.seq;
        lock.last_fresh = io.now;
    }
    foreign_producer(joint, &lock.producer, io.now, io.cfg.stream_timeout)
}

/// True if a fresh joint command this tick comes from a producer other than the
/// locked one.
fn foreign_producer(
    joint: &Option<JointCommand>,
    locked: &ProducerRef,
    now: Instant,
    timeout: Duration,
) -> bool {
    joint
        .as_ref()
        .is_some_and(|c| &c.producer != locked && fresh(c.recv_at, now, timeout))
}

/// Whether a command that arrived at `recv_at` is still within the watchdog
/// window, distinguishing a live stream from a stale leftover in the channel.
fn fresh(recv_at: Instant, now: Instant, timeout: Duration) -> bool {
    now.duration_since(recv_at) <= timeout
}

/// Scale the chase step `setpoint -> stepped` so the end-effector's linear speed
/// stays under `max_ee` (m/s), using the Jacobian's linear rows. A step that
/// barely moves the hand is left unscaled, so self-motion and near-singular
/// directions run at the motor rate (`stepped` is already motor-rate-limited).
fn cap_ee_speed(
    setpoint: &JointVec,
    stepped: &JointVec,
    jacobian: &Jacobian,
    max_ee: f64,
    dt: f64,
) -> JointVec {
    let delta: JointVec = std::array::from_fn(|i| stepped[i] - setpoint[i]);
    let twist = jacobian * SVector::<f64, ARM_DOF>::from_column_slice(&delta);
    let ee_speed = twist.fixed_rows::<3>(0).norm() / dt;
    let scale = if ee_speed.is_finite() && ee_speed > max_ee {
        max_ee / ee_speed
    } else {
        1.0
    };
    std::array::from_fn(|i| setpoint[i] + delta[i] * scale)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ARM_DOF;

    fn producer(instance: &str) -> ProducerRef {
        ProducerRef::new("core".to_string(), instance.to_string())
    }
    fn joint_cmd(instance: &str, recv_at: Instant) -> JointCommand {
        JointCommand {
            seq: 1,
            producer: producer(instance),
            recv_at,
            positions: [0.0; ARM_DOF],
        }
    }

    #[test]
    fn fresh_tracks_the_watchdog_window() {
        let now = Instant::now();
        let timeout = Duration::from_millis(100);
        assert!(fresh(now, now, timeout));
        assert!(fresh(now - Duration::from_millis(50), now, timeout));
        assert!(!fresh(now - Duration::from_millis(150), now, timeout));
    }

    #[test]
    fn foreign_producer_flags_only_a_fresh_other_instance() {
        let now = Instant::now();
        let timeout = Duration::from_millis(100);
        let locked = producer("teleop");

        // No command, or the locked producer itself, is not foreign.
        assert!(!foreign_producer(&None, &locked, now, timeout));
        assert!(!foreign_producer(
            &Some(joint_cmd("teleop", now)),
            &locked,
            now,
            timeout
        ));

        // A fresh command from a different producer is foreign.
        assert!(foreign_producer(
            &Some(joint_cmd("other", now)),
            &locked,
            now,
            timeout
        ));

        // A stale command from a different producer is not (it is not live).
        let stale = now - Duration::from_millis(150);
        assert!(!foreign_producer(
            &Some(joint_cmd("other", stale)),
            &locked,
            now,
            timeout
        ));
    }

    #[test]
    fn cap_ee_speed_throttles_a_hand_moving_step_to_the_cap() {
        // Joint 0 moves the hand 1 m per rad along x; the rest do not move it.
        let mut jac = Jacobian::zeros();
        jac[(0, 0)] = 1.0;
        let dt = 0.01;
        let max_ee = 0.25;
        // A 0.1 rad step on joint 0 is 10 m/s of hand speed, well over the cap.
        let next = cap_ee_speed(
            &[0.0; ARM_DOF],
            &{
                let mut s = [0.0; ARM_DOF];
                s[0] = 0.1;
                s
            },
            &jac,
            max_ee,
            dt,
        );
        // Scaled so the hand moves at exactly the cap: step = max_ee * dt.
        assert!((next[0] - max_ee * dt).abs() < 1e-12);
        assert_eq!(next[1..], [0.0; ARM_DOF - 1]);
    }

    #[test]
    fn cap_ee_speed_leaves_a_step_that_does_not_move_the_hand() {
        // A zero linear Jacobian: no joint moves the hand, so nothing is scaled.
        let jac = Jacobian::zeros();
        let mut stepped = [0.0; ARM_DOF];
        stepped[3] = 0.2;
        let next = cap_ee_speed(&[0.0; ARM_DOF], &stepped, &jac, 0.25, 0.01);
        assert_eq!(next, stepped);
    }
}
