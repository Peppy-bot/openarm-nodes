// Ambient following of a streamed joint setpoint, mirroring the real arm's
// Follow mode. Between moves, chase the latest fresh command at the per-joint
// velocity limits and stream (q_des, dq_des) to the sim; hold when no fresh
// command exists or its stream times out. A move_arm_joints goal owns the arm
// while it runs (the shared busy gate), and follow re-anchors on the measured
// pose when it resumes, so a move never fights the stream.
//
// Unlike the real arm there is no end-effector speed cap: that needs the URDF
// Jacobian the sim does not load. The per-joint velocity limit plus the sim's
// own dynamics bound motion instead.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use peppylib::TopicPublisher;
use peppylib::runtime::CancellationToken;
use tokio::sync::watch;
use tokio::time::MissedTickBehavior;
use tracing::warn;

use crate::config::ControlParams;
use crate::setctrl;
use crate::state::{self, SharedState};
use crate::stream::JointCommand;
use crate::trajectory::{ARM_DOF as DOF, JointVec};

pub async fn run(
    set_ctrl_pub: TopicPublisher,
    actuator_names: Arc<[String; DOF]>,
    busy: Arc<AtomicBool>,
    state: Arc<SharedState>,
    cmd: watch::Receiver<Option<JointCommand>>,
    params: ControlParams,
    token: CancellationToken,
) {
    let dt = params.control_period.as_secs_f64();
    // Last commanded setpoint while following; re-anchored on the measured pose
    // whenever we are not actively chasing.
    let mut setpoint: Option<JointVec> = None;
    let mut failing = false;

    // interval (not sleep) so the chase cadence holds at control_rate_hz instead
    // of drifting by the per-tick work time; Delay avoids a catch-up burst.
    let mut ticker = tokio::time::interval(params.control_period);
    ticker.set_missed_tick_behavior(MissedTickBehavior::Delay);

    loop {
        tokio::select! {
            _ = token.cancelled() => return,
            _ = ticker.tick() => {}
        }

        // A move owns the arm: yield and keep the anchor on the measured pose so
        // we resume from where the move left it.
        if busy.load(Ordering::Acquire) {
            setpoint = state::snapshot_positions(&state);
            continue;
        }

        // Follow only a command still within the stream timeout; otherwise hold
        // (publish nothing — the sim keeps the last setpoint) and track measured.
        let fresh = {
            let guard = cmd.borrow();
            guard
                .as_ref()
                .filter(|c| c.recv_at.elapsed() <= params.stream_timeout)
                .map(|c| c.positions)
        };
        let (Some(target), Some(from)) = (fresh, setpoint.or_else(|| state::snapshot_positions(&state)))
        else {
            setpoint = state::snapshot_positions(&state);
            continue;
        };
        let Some(target) = state::clamp_to_limits(&state, target) else {
            setpoint = state::snapshot_positions(&state);
            continue;
        };

        let next = chase_step(from, target, &params.max_joint_velocity, dt);
        let dq = velocity(from, next, dt);
        setpoint = Some(next);

        match setctrl::publish(&set_ctrl_pub, &actuator_names, &next, &dq).await {
            Ok(()) => failing = false,
            Err(e) if !failing => {
                failing = true;
                warn!("follow set_ctrl publish failing, suppressing repeats: {e}");
            }
            Err(_) => {}
        }
    }
}

// Advance each joint toward its target by at most max_velocity * dt, so a far
// setpoint (or a producer hiccup) eases over instead of snapping.
fn chase_step(from: JointVec, target: JointVec, max_velocity: &JointVec, dt: f64) -> JointVec {
    let mut next = from;
    for i in 0..DOF {
        let step = max_velocity[i] * dt;
        next[i] = from[i] + (target[i] - from[i]).clamp(-step, step);
    }
    next
}

// Velocity feedforward from the chase step, matching the real arm's dq_des.
fn velocity(from: JointVec, next: JointVec, dt: f64) -> JointVec {
    let mut dq = [0.0_f64; DOF];
    for i in 0..DOF {
        dq[i] = (next[i] - from[i]) / dt;
    }
    dq
}
