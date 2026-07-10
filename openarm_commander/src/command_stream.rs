// Always-on command publisher. For each enabled arm, streams its 7-joint target
// on `arm_joint_commands`; for each enabled gripper, streams its opening (m) on
// `gripper_commands`. Both are tagged with their id (arm_id / gripper_id) and go
// to the hub, which governs each and re-streams the governed value the followers
// track. A disabled side emits nothing, so the hub's stream timeout lapses and it
// holds. Re-publishing every tick (even an unchanged target) keeps the hub's
// stream watchdog alive between operator inputs; the hub clamps and rate-limits
// what it receives, so this only has to deliver the latest setpoint.
//
// Each side+stream runs its own publish task on its own interval, cloning the
// shared per-topic publisher. A single shared loop publishing Left then Right
// would leave Right permanently second (zenoh publish resolves synchronously), so
// independent tasks avoid that bias.

use std::sync::Arc;
use std::time::Duration;

use peppygen::NodeRunner;
use peppygen::emitted_topics::openarm_arm_joint_commands::v1::arm_joint_commands;
use peppygen::emitted_topics::openarm_governor_control::v1::governor_control;
use peppygen::emitted_topics::openarm_gripper_commands::v1::gripper_commands;
use peppylib::runtime::CancellationToken;
use peppylib::{Payload, TopicPublisher};
use tokio::time::MissedTickBehavior;
use tracing::{error, info, warn};

use crate::pose::{ArmModels, Jog, JogCaps, JogMode, JogStep, Pose, jog_tick};
use crate::state::{ARM_DOF, ArmTarget, SharedState, Side, UiState};

pub async fn run(
    runner: Arc<NodeRunner>,
    state: SharedState,
    command_rate_hz: u32,
    token: CancellationToken,
    models: ArmModels,
) {
    // A failed publisher declaration leaves the node serving UI/health but unable to
    // command anything, so cancel the node to restart it rather than returning quietly.
    let arm_pub = match arm_joint_commands::declare_publisher(&runner).await {
        Ok(p) => p,
        Err(e) => {
            error!("declare arm_joint_commands publisher: {e}");
            return token.cancel();
        }
    };
    // One shared gripper publisher, cloned per side like the arm publisher; each
    // side's stream tags its own gripper_id, so the hub tells them apart.
    let gripper_pub = match gripper_commands::declare_publisher(&runner).await {
        Ok(p) => p,
        Err(e) => {
            error!("declare gripper_commands publisher: {e}");
            return token.cancel();
        }
    };
    let governor_pub = match governor_control::declare_publisher(&runner).await {
        Ok(p) => p,
        Err(e) => {
            error!("declare governor_control publisher: {e}");
            return token.cancel();
        }
    };

    let mut tasks = tokio::task::JoinSet::new();

    // Re-publish the operator's governor controls every tick. Unlike the arm/gripper
    // streams these have no deadman: the hub's governor must always know the
    // operator's intent, and the lossy QoS means a one-shot publish could be
    // dropped, so the latest state is re-sent continuously.
    let governor_state = state.clone();
    tasks.spawn(stream_setpoints(
        governor_pub,
        command_rate_hz,
        token.clone(),
        "governor control".to_string(),
        move || {
            let s = governor_state.lock().unwrap_or_else(|p| p.into_inner());
            Some(
                governor_control::build_message(
                    s.collision_enabled,
                    s.d_stop,
                    s.d_safe,
                    s.max_ee_velocity_m_s,
                )
                .map_err(|e| e.to_string()),
            )
        },
    ));
    // The tick period every stream runs at; jog steps derive from it so a different
    // command_rate_hz changes the step size, never the jog speed.
    let tick_dt_s = 1.0 / command_rate_hz as f64;
    for side in [Side::Left, Side::Right] {
        // Arm: advance any active jog (joint or Cartesian) one step, then stream the
        // 7-joint setpoint while enabled.
        let arm_state = state.clone();
        let arm_models = models.clone();
        tasks.spawn(stream_setpoints(
            arm_pub.clone(),
            command_rate_hz,
            token.clone(),
            format!("{} arm", side.label()),
            move || {
                let target = {
                    let mut s = arm_state.lock().unwrap_or_else(|p| p.into_inner());
                    if !s.enabled[side] {
                        return None;
                    }
                    // Caps re-derive each tick from the operator's live EE speed cap,
                    // so retuning the knob mid-jog changes the jog speed with it.
                    let caps = JogCaps::per_tick(tick_dt_s, s.max_ee_velocity_m_s);
                    let advance = advance_jog(&s.arms[side], side, &arm_models, caps);
                    apply_jog(&mut s, side, advance);
                    s.arms[side].joints
                };
                Some(
                    arm_joint_commands::build_message(side.arm_id(), target)
                        .map_err(|e| e.to_string()),
                )
            },
        ));
        // Gripper: stream the opening (m) while enabled, tagged with gripper_id
        // for the hub to demux (mirror of the arm stream above).
        let gripper_state = state.clone();
        tasks.spawn(stream_setpoints(
            gripper_pub.clone(),
            command_rate_hz,
            token.clone(),
            format!("{} gripper", side.label()),
            move || {
                let position = {
                    let s = gripper_state.lock().unwrap_or_else(|p| p.into_inner());
                    if !s.enabled[side] {
                        return None;
                    }
                    s.grippers[side].position
                };
                Some(
                    gripper_commands::build_message(side.gripper_id(), position)
                        .map_err(|e| e.to_string()),
                )
            },
        ));
    }
    // join_next surfaces tasks in completion order, so a panicked stream is
    // seen immediately. A dead channel would silently hold its side while the
    // node reports healthy, which is worse than a restart: cancel the node.
    while let Some(result) = tasks.join_next().await {
        if let Err(e) = result {
            error!("command stream task died: {e}; cancelling the node");
            token.cancel();
        }
    }
}

/// The outcome of advancing one side's jog a tick, ready for the caller to apply: the
/// reconciled joint setpoint, the jog to retain (`None` once it retires or reaches its
/// target), whether it is held at the reach boundary, and any moving <-> blocked
/// transition to announce. Pure, so the tick decision is testable without a live
/// [`UiState`] and the mutation is a straight assignment in [`apply_jog`].
#[derive(Clone, Copy, Debug)]
struct JogAdvance {
    joints: [f64; ARM_DOF],
    jog: Option<Jog>,
    blocked: bool,
    event: Option<JogEvent>,
}

/// A one-shot jog status transition. Emitted only on the edge, so a held boundary
/// reports once, not at the command rate.
#[derive(Clone, Copy, Debug)]
enum JogEvent {
    Moving { mode: JogMode },
    Blocked { mode: JogMode, desired: Pose },
}

// Advance one side's active jog by one tick. A joint jog reconciles in a single step
// (the streamed joints are the target; the hub governs the ramp); a Cartesian jog
// steps the joint target a capped increment toward the desired pose, holds it at the
// reach boundary, and retires once converged. Pure: `jog_tick` briefly takes the model
// lock inside it, but no UiState lock is held here, so the caller applies the result.
fn advance_jog(arm: &ArmTarget, side: Side, models: &ArmModels, caps: JogCaps) -> JogAdvance {
    let hold = |jog, blocked| JogAdvance {
        joints: arm.joints,
        jog,
        blocked,
        event: None,
    };
    let cartesian = match arm.jog {
        None => return hold(None, arm.jog_blocked),
        // The hub governs the joint ramp, so reconcile in one step and retire.
        Some(Jog::Joints(target)) => {
            return JogAdvance {
                joints: target,
                jog: None,
                blocked: false,
                event: None,
            };
        }
        Some(Jog::Cartesian(cartesian)) => cartesian,
    };
    match jog_tick(models, side, &arm.joints, &cartesian, caps) {
        JogStep::Converged => hold(None, false),
        JogStep::Stepped(joints) => JogAdvance {
            joints,
            jog: arm.jog,
            blocked: false,
            // Announce resumption only when leaving a held boundary.
            event: arm.jog_blocked.then_some(JogEvent::Moving {
                mode: cartesian.mode,
            }),
        },
        JogStep::Blocked => JogAdvance {
            joints: arm.joints,
            jog: arm.jog,
            blocked: true,
            // Announce the boundary once, on entry.
            event: (!arm.jog_blocked).then_some(JogEvent::Blocked {
                mode: cartesian.mode,
                desired: cartesian.desired,
            }),
        },
    }
}

// Apply a computed jog advance to the side's target and emit its status transition, if
// any. The only mutation half of the pure/apply split above.
fn apply_jog(s: &mut UiState, side: Side, adv: JogAdvance) {
    s.arms[side].joints = adv.joints;
    s.arms[side].jog = adv.jog;
    s.arms[side].jog_blocked = adv.blocked;
    match adv.event {
        Some(JogEvent::Moving { mode }) => {
            s.set_status(format!("{}: pose jog moving", side.label()));
            info!(side = side.label(), ?mode, "pose jog resumed");
        }
        Some(JogEvent::Blocked { mode, desired }) => {
            s.set_status(format!("{}: pose at reach limit, holding", side.label()));
            info!(
                side = side.label(),
                ?mode,
                ?desired,
                "pose jog at reach limit"
            );
        }
        None => {}
    }
}

// Publish the latest setpoint from `next_message` at command_rate_hz, skipping a
// tick whenever it returns None (the side is disabled). Failures latch so a
// stuck side warns once, not every tick.
async fn stream_setpoints(
    publisher: TopicPublisher,
    command_rate_hz: u32,
    token: CancellationToken,
    label: String,
    mut next_message: impl FnMut() -> Option<Result<Payload, String>>,
) {
    let period = Duration::from_micros(1_000_000 / command_rate_hz as u64);
    // interval (not sleep) so the publish cadence holds at command_rate_hz
    // instead of drifting by the per-tick work time; Delay avoids a catch-up
    // burst after a scheduling hiccup.
    let mut ticker = tokio::time::interval(period);
    ticker.set_missed_tick_behavior(MissedTickBehavior::Delay);
    let mut failing = false;

    loop {
        tokio::select! {
            _ = token.cancelled() => return,
            _ = ticker.tick() => {}
        }

        let Some(built) = next_message() else {
            continue;
        };
        let result = match built {
            Ok(msg) => publisher.publish(msg).await.map_err(|e| e.to_string()),
            Err(e) => Err(e),
        };
        match result {
            Ok(()) => failing = false,
            Err(e) if !failing => {
                failing = true;
                warn!("{label} command publish failing, suppressing repeats: {e}");
            }
            Err(_) => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pose::CartesianJog;
    use openarm_description::HardwareVersion;

    fn models() -> ArmModels {
        ArmModels::from_version(HardwareVersion::V2)
    }

    // The caps a 100 Hz tick at the sim launcher's 0.5 m/s knob derives.
    fn caps() -> JogCaps {
        JogCaps::per_tick(0.01, 0.5)
    }

    fn arm(joints: [f64; ARM_DOF], jog: Option<Jog>, jog_blocked: bool) -> ArmTarget {
        let mut a = ArmTarget::home();
        a.joints = joints;
        a.jog = jog;
        a.jog_blocked = jog_blocked;
        a
    }

    fn position_jog(desired: Pose) -> Jog {
        Jog::Cartesian(CartesianJog {
            mode: JogMode::Position,
            desired,
            arm_angle: 0.0,
        })
    }

    #[test]
    fn joint_jog_reconciles_in_one_step_and_retires() {
        let target = [0.1, -0.2, 0.3, 0.9, -0.1, 0.2, 0.05];
        let a = arm([0.0; ARM_DOF], Some(Jog::Joints(target)), false);
        let adv = advance_jog(&a, Side::Left, &models(), caps());
        assert_eq!(
            adv.joints, target,
            "a joint jog snaps the setpoint to the target"
        );
        assert!(
            adv.jog.is_none(),
            "a joint jog retires after its single step"
        );
        assert!(!adv.blocked);
        assert!(adv.event.is_none());
    }

    #[test]
    fn idle_arm_holds_its_setpoint() {
        let q = [0.3, 0.1, 0.2, 0.8, 0.3, 0.2, 0.15];
        let adv = advance_jog(&arm(q, None, false), Side::Left, &models(), caps());
        assert_eq!(adv.joints, q, "an idle side keeps its setpoint");
        assert!(adv.jog.is_none());
        assert!(adv.event.is_none());
    }

    #[test]
    fn cartesian_jog_on_the_current_pose_converges() {
        let m = models();
        let q = [0.3, 0.1, 0.2, 0.8, 0.3, 0.2, 0.15];
        let here = m.ee_pose_world(Side::Left, &q);
        let adv = advance_jog(
            &arm(q, Some(position_jog(here)), false),
            Side::Left,
            &m,
            caps(),
        );
        assert!(
            adv.jog.is_none(),
            "reaching the desired pose retires the jog"
        );
        assert!(!adv.blocked);
        assert!(adv.event.is_none());
    }

    #[test]
    fn boundary_is_announced_on_entry_not_while_held() {
        let m = models();
        let start = [0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0];
        let here = m.ee_pose_world(Side::Left, &start);
        let far = position_jog([here[0] + 2.0, here[1], here[2], here[3], here[4], here[5]]);
        // Drive the jog to the envelope, threading each advance back into the arm
        // exactly as the stream does, until it first reports the boundary.
        let mut a = arm(start, Some(far), false);
        let entry = loop_to_boundary(&m, &mut a);
        assert!(
            matches!(entry.event, Some(JogEvent::Blocked { .. })),
            "the boundary is announced when first entered"
        );
        assert!(
            a.jog.is_some(),
            "the jog stays armed so pulling back into reach resumes it"
        );
        // Held at the boundary (jog_blocked now set): the joints are pinned, so the
        // next tick blocks identically but must stay quiet.
        let held = advance_jog(&a, Side::Left, &m, caps());
        assert!(held.blocked, "the pinned arm stays at the boundary");
        assert!(held.event.is_none(), "a held boundary does not re-announce");
    }

    // Advance until the jog first reports the boundary, applying each result to `a`
    // like [`apply_jog`] does. Returns the entering advance; panics if it never blocks.
    fn loop_to_boundary(m: &ArmModels, a: &mut ArmTarget) -> JogAdvance {
        for _ in 0..5000 {
            let adv = advance_jog(a, Side::Left, m, caps());
            a.joints = adv.joints;
            a.jog = adv.jog;
            a.jog_blocked = adv.blocked;
            if adv.blocked {
                return adv;
            }
            assert!(
                a.jog.is_some(),
                "the jog retired before reaching the boundary"
            );
        }
        panic!("jog never reached the boundary within 5000 ticks");
    }

    #[test]
    fn resuming_from_a_held_boundary_announces_moving() {
        let m = models();
        let q = [0.3, 0.1, 0.2, 0.8, 0.3, 0.2, 0.15];
        let here = m.ee_pose_world(Side::Left, &q);
        // A reachable nudge so the step advances, flagged as previously held.
        let near = position_jog([here[0] + 0.02, here[1], here[2], here[3], here[4], here[5]]);
        let adv = advance_jog(&arm(q, Some(near), true), Side::Left, &m, caps());
        assert!(!adv.blocked);
        assert!(
            matches!(adv.event, Some(JogEvent::Moving { .. })),
            "leaving the boundary announces movement"
        );
    }
}
