//! openarm_backbone - bimanual coordinator. It owns all arm motion: it
//! consumes the leading node's joint or pose stream (per `upstream_mode`) and
//! exposes the joint / Cartesian move actions, generates the trajectories,
//! runs the self-collision governor over both arms together, and streams the
//! governed per-arm setpoints the arms follow. Grippers run through the
//! backbone the same way: the leading node's gripper stream and move_gripper
//! goals both feed the coordinator, the grippers ride the same governed
//! configuration as the arm joints (a gripper cannot open its
//! fingers into the other arm), and the governed opening streams to each
//! gripper over its gripper_link pairing slot. The governor is URDF-based, so
//! it runs identically for the sim and the real arms.

mod actions;
mod arm_pair;
mod chase;
mod coordinator;
mod governor;
mod liveness;
mod planner;
mod publish;
mod servo;
mod startup;
mod streams;
mod torso;
mod trajectory;
mod types;
mod upstream;

pub(crate) use arm_pair::ArmPair;
pub(crate) use types::{
    ARM_DOF, JointVec, MOTION_TIMEOUT_FACTOR, Side, motion_timed_out, pose_from_wire,
    world_pose_arrays,
};

use std::sync::Arc;
use std::sync::atomic::AtomicBool;
use std::time::Duration;

use openarm_description::HardwareVersion;
use peppygen::consumed_topics::collision_ctrl::governor_control;
use peppygen::paired_topics::{
    leader_left_arm, leader_left_arm_pose, leader_right_arm, leader_right_arm_pose,
};
use peppygen::{NodeBuilder, NodeRunner, Parameters, Result};
use tokio::sync::{mpsc, watch};
use tokio::task::JoinSet;
use tracing::{error, info, warn};

use coordinator::ArmChannels;
use planner::{PlanConfig, Planner};
use servo::EeCaps;
use upstream::UpstreamMode;

/// Spawn a never-returning inbound listener into the backbone's supervised task set,
/// adapting its `()` output to the set's `Result` so its exit trips the
/// fatal-first-exit like any other backbone task.
fn spawn_listener<F>(set: &mut JoinSet<Result<()>>, listener: F)
where
    F: std::future::Future<Output = ()> + Send + 'static,
{
    set.spawn(async move {
        listener.await;
        Ok(())
    });
}

/// Warn about any paired upstream slot of the kind `mode` does not follow:
/// exactly one kind is subscribed, so a leader linked to the other kind
/// streams into a slot this instance never reads.
fn warn_unfollowed_upstream_slots(runner: &NodeRunner, mode: UpstreamMode) {
    fn is_paired<T>(slot: Result<Option<T>>) -> Result<bool> {
        slot.map(|pairing| pairing.is_some())
    }
    let (unfollowed, consequence) = match mode {
        UpstreamMode::Joints => (
            [
                (
                    "leader_left_arm_pose",
                    is_paired(leader_left_arm_pose::pose_setpoints::paired(runner)),
                ),
                (
                    "leader_right_arm_pose",
                    is_paired(leader_right_arm_pose::pose_setpoints::paired(runner)),
                ),
            ],
            // The whole pose pairing is dead: no reads, no pose_states back.
            "never reads or answers it",
        ),
        UpstreamMode::Pose => (
            [
                (
                    "leader_left_arm",
                    is_paired(leader_left_arm::joint_setpoints::paired(runner)),
                ),
                (
                    "leader_right_arm",
                    is_paired(leader_right_arm::joint_setpoints::paired(runner)),
                ),
            ],
            // Joint states still relay back; only the command side is dead.
            "never reads it",
        ),
    };
    for (slot, paired) in unfollowed {
        match paired {
            Ok(true) => warn!("{slot} is paired, but upstream_mode={mode} {consequence}"),
            Ok(false) => {}
            Err(e) => warn!("{slot} pairing state unknown: {e}"),
        }
    }
}

/// Build one arm model from the embedded OpenArm description, with the elbow
/// singularity margin applied. The description carries no solver dep and exports the
/// margin as a constant; applying it here is the single site the backbone imposes it, so the
/// model's `limits()` carry it for IK seeding, trajectory sizing, and the chase clamp.
pub(crate) fn arm_model(
    version: HardwareVersion,
    base_link: &str,
) -> std::result::Result<srs_model::Arm, srs_model::SrsError> {
    Ok(
        srs_model::Arm::from_urdf(version.urdf(), base_link)?.with_lower_floor(
            version.elbow_joint_index(),
            version.elbow_singularity_floor_rad(),
        ),
    )
}

fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::INFO)
        .init();

    NodeBuilder::new().run(|params: Parameters, node_runner| async move {
        // Pairing stamps read the daemon-resolved clock (sim time under a
        // simulated clock), so setpoint consumers age samples on one timeline.
        peppygen::clock::init(&node_runner).await?;

        assert!(params.control_rate_hz > 0, "control_rate_hz must be > 0");
        let max_joint_velocity_rad_s: JointVec = [
            params.max_joint_velocity_rad_s_1,
            params.max_joint_velocity_rad_s_2,
            params.max_joint_velocity_rad_s_3,
            params.max_joint_velocity_rad_s_4,
            params.max_joint_velocity_rad_s_5,
            params.max_joint_velocity_rad_s_6,
            params.max_joint_velocity_rad_s_7,
        ];
        assert!(
            max_joint_velocity_rad_s
                .iter()
                .all(|v| v.is_finite() && *v > 0.0),
            "all max_joint_velocity_rad_s_N must be finite and > 0"
        );
        assert!(
            params.max_ee_velocity_m_s.is_finite() && params.max_ee_velocity_m_s > 0.0,
            "max_ee_velocity_m_s must be a positive finite number"
        );
        assert!(
            params.max_ee_angular_velocity_rad_s.is_finite()
                && params.max_ee_angular_velocity_rad_s > 0.0,
            "max_ee_angular_velocity_rad_s must be a positive finite number"
        );
        // Enforce the documented contract: the cutoff must sit below the control loop's
        // Nyquist frequency, or the low-pass does not attenuate (and above it is
        // meaningless). A hard bound at Nyquist; the node default sits well under it.
        let nyquist_hz = params.control_rate_hz as f64 / 2.0;
        assert!(
            params.velocity_filter_cutoff_hz.is_finite()
                && params.velocity_filter_cutoff_hz > 0.0
                && params.velocity_filter_cutoff_hz < nyquist_hz,
            "velocity_filter_cutoff_hz ({}) must be in (0, Nyquist = control_rate_hz/2 = {})",
            params.velocity_filter_cutoff_hz,
            nyquist_hz
        );
        // The governor and the commander UI must reject the same bands; validate here
        // (reusing the governor's own predicate) so a bad launcher value fails at
        // bringup with a clear message rather than deep inside model construction.
        assert!(
            governor::valid_band(params.d_stop_m, params.d_safe_m),
            "collision band invalid: require 0 < d_stop_m ({}) < d_safe_m ({}), both finite",
            params.d_stop_m,
            params.d_safe_m
        );
        // Governor controls are optional and exclusive: zero producers leaves
        // the launch-time band standing, more than one is a mis-wired launch.
        let governor_producers = governor_control::bound_producers(&node_runner);
        assert!(
            governor_producers.len() <= 1,
            "governor_control accepts at most one producer, got {}",
            governor_producers.len()
        );
        if governor_producers.is_empty() {
            info!("no governor_control producer bound; the launch-time band stands");
        }

        let cycle_period = Duration::from_micros(1_000_000 / params.control_rate_hz as u64);

        // Which OpenArm generation the arms are; selects the embedded description for both
        // the srs_model arms and the bimanual collision model.
        let hardware_version: HardwareVersion = params
            .hardware_version
            .parse()
            .unwrap_or_else(|e| panic!("{e}"));

        // Which upstream command kind this instance follows; parsed once, and
        // only that kind's listener is spawned below.
        let upstream_mode: UpstreamMode = params
            .upstream_mode
            .parse()
            .unwrap_or_else(|e: String| panic!("{e}"));
        info!("following upstream {upstream_mode} commands");

        // Two arm models (FK/IK/Jacobian/limits, with the elbow singularity margin)
        // and the bimanual collision model, all from the embedded OpenArm description.
        // The per-side chain base link is a fact of the generation's URDF, resolved from
        // the description rather than configured, so a v2 launch cannot inherit a v1 name.
        let left_base = hardware_version.base_link(openarm_description::Side::Left);
        let right_base = hardware_version.base_link(openarm_description::Side::Right);
        let left_model = arm_model(hardware_version, left_base)
            .unwrap_or_else(|e| panic!("build left arm model from base '{left_base}': {e}"));
        let right_model = arm_model(hardware_version, right_base)
            .unwrap_or_else(|e| panic!("build right arm model from base '{right_base}': {e}"));
        info!("arm models loaded (left '{left_base}', right '{right_base}')");

        // The collision model needs the URDF string (joint limits are irrelevant to it,
        // so no margin) and the meshes on disk; the file-based builder reads the meshes
        // materialized from the embedded description into a per-process scratch dir. A
        // unique tempdir (not a fixed shared path) avoids a start/restart race on the
        // files; `Governor::build` reads them synchronously, so the handle can drop right
        // after and self-clean.
        let meshes_tmp = tempfile::tempdir()
            .unwrap_or_else(|e| panic!("create scratch dir for collision meshes: {e}"));
        hardware_version
            .write_meshes_to(meshes_tmp.path())
            .unwrap_or_else(|e| panic!("materialize collision meshes: {e}"));
        let meshes_dir = meshes_tmp.path().to_str().unwrap_or_else(|| {
            panic!(
                "meshes dir path is not valid UTF-8: {:?}",
                meshes_tmp.path()
            )
        });

        let governor = governor::Governor::build(
            hardware_version.urdf(),
            meshes_dir,
            left_base,
            right_base,
            params.d_stop_m,
            params.d_safe_m,
            max_joint_velocity_rad_s
                .iter()
                .copied()
                .fold(0.0_f64, f64::max),
            params.max_ee_velocity_m_s,
            params.collision_governor_enabled,
        )
        .unwrap_or_else(|e| panic!("build self-collision governor: {e}"));
        info!(
            "self-collision governor ready (d_stop_m={} d_safe_m={} default {})",
            params.d_stop_m,
            params.d_safe_m,
            if params.collision_governor_enabled {
                "ENABLED"
            } else {
                "DISABLED"
            },
        );

        let left_limits = left_model.limits();
        let right_limits = right_model.limits();
        // The chase clamps every streamed/planned target into these limits with
        // `f64::clamp`, which is total only for finite, well-ordered bounds. Assert
        // it here so a malformed URDF aborts at bringup, not mid-tick.
        assert!(
            left_limits
                .iter()
                .chain(right_limits.iter())
                .all(|l| l.lo.is_finite() && l.hi.is_finite() && l.lo <= l.hi),
            "joint position limits must be finite and well-ordered (lo <= hi)"
        );
        let plan_cfg = |limits| PlanConfig {
            cycle_period,
            max_joint_velocity_rad_s,
            ee: EeCaps {
                linear_m_s: params.max_ee_velocity_m_s,
                angular_rad_s: params.max_ee_angular_velocity_rad_s,
            },
            limits,
        };
        let planners = ArmPair::new(
            Planner::new(Side::Left, left_model, plan_cfg(left_limits)),
            Planner::new(Side::Right, right_model, plan_cfg(right_limits)),
        );

        // Per-arm channels. Listeners fill the watch slots; action handlers send
        // accepted goals; the coordinator reads all of it and, while a move runs,
        // clears that side's command watch. The command streams are held by the
        // coordinator as their sender (read + clear); the listener keeps a clone,
        // so no separate receiver is needed. State streams stay reader-side.
        let (cmd_tx0, _) = watch::channel(None);
        let (cmd_tx1, _) = watch::channel(None);
        let (gripcmd_tx0, _) = watch::channel(None);
        let (gripcmd_tx1, _) = watch::channel(None);
        let (meas_tx0, meas_rx0) = watch::channel(None);
        let (meas_tx1, meas_rx1) = watch::channel(None);
        let (grip_tx0, grip_rx0) = watch::channel(None);
        let (grip_tx1, grip_rx1) = watch::channel(None);
        let (goal_tx0, goal_rx0) = mpsc::channel(1);
        let (goal_tx1, goal_rx1) = mpsc::channel(1);
        let (grip_goal_tx0, grip_goal_rx0) = mpsc::channel(1);
        let (grip_goal_tx1, grip_goal_rx1) = mpsc::channel(1);
        let busy = [
            Arc::new(AtomicBool::new(false)),
            Arc::new(AtomicBool::new(false)),
        ];
        let gripper_busy = [
            Arc::new(AtomicBool::new(false)),
            Arc::new(AtomicBool::new(false)),
        ];
        let (config_tx, config_rx) = watch::channel(streams::GovernorConfig {
            enabled: params.collision_governor_enabled,
            d_stop: params.d_stop_m,
            d_safe: params.d_safe_m,
            max_ee_velocity_m_s: params.max_ee_velocity_m_s,
        });

        let channels = ArmPair::new(
            ArmChannels {
                command: cmd_tx0.clone(),
                gripper_command: gripcmd_tx0.clone(),
                measured: meas_rx0,
                gripper: grip_rx0,
                goals: goal_rx0,
                busy: busy[0].clone(),
                gripper_goals: grip_goal_rx0,
                gripper_busy: gripper_busy[0].clone(),
            },
            ArmChannels {
                command: cmd_tx1.clone(),
                gripper_command: gripcmd_tx1.clone(),
                measured: meas_rx1,
                gripper: grip_rx1,
                goals: goal_rx1,
                busy: busy[1].clone(),
                gripper_goals: grip_goal_rx1,
                gripper_busy: gripper_busy[1].clone(),
            },
        );

        // Gate exposing actions + streaming on the robot being ready, in a spawned
        // task so this setup closure returns promptly for the health probe.
        let runner = node_runner.clone();
        let token = node_runner.cancellation_token().clone();
        let goal_busy = [busy[0].clone(), busy[1].clone()];
        tokio::spawn(async move {
            startup::wait_until_ready(&runner, &token).await;
            warn_unfollowed_upstream_slots(&runner, upstream_mode);

            // The coordination loop (owns the governor, both planners, the channels;
            // streams governed setpoints once both arms report) and the action
            // handlers are all meant to run for the life of the node.
            let mut set = JoinSet::new();
            set.spawn(coordinator::run(
                runner.clone(),
                governor,
                planners,
                channels,
                config_rx,
                coordinator::RunConfig {
                    cycle_period,
                    velocity_filter_cutoff_hz: params.velocity_filter_cutoff_hz,
                    upstream_mode,
                },
                token.clone(),
            ));
            set.spawn(actions::arm::run_move_arm_joints(
                runner.clone(),
                [goal_tx0.clone(), goal_tx1.clone()],
                [goal_busy[0].clone(), goal_busy[1].clone()],
                [left_limits, right_limits],
            ));
            set.spawn(actions::arm::run_move_arm(
                runner.clone(),
                [goal_tx0.clone(), goal_tx1.clone()],
                [goal_busy[0].clone(), goal_busy[1].clone()],
            ));
            set.spawn(actions::ready::run_move_to_ready(
                runner.clone(),
                [goal_tx0, goal_tx1],
                [goal_busy[0].clone(), goal_busy[1].clone()],
            ));
            set.spawn(actions::gripper::run_move_gripper(
                runner.clone(),
                [grip_goal_tx0, grip_goal_tx1],
                [gripper_busy[0].clone(), gripper_busy[1].clone()],
            ));

            // Inbound listeners buffer the latest message into the watch slots. They
            // run under the same fatal-first-exit supervision as the rest of the backbone,
            // so a listener that dies takes the node down instead of leaving the
            // coordinator streaming on stale measured state or governor controls while
            // the node still reports healthy.
            // Exactly one upstream listener; the other slot kind is never
            // subscribed.
            match upstream_mode {
                UpstreamMode::Joints => spawn_listener(
                    &mut set,
                    streams::run_joint_command_listener(runner.clone(), [cmd_tx0, cmd_tx1]),
                ),
                UpstreamMode::Pose => spawn_listener(
                    &mut set,
                    streams::run_pose_command_listener(runner.clone(), [cmd_tx0, cmd_tx1]),
                ),
            }
            spawn_listener(
                &mut set,
                streams::run_gripper_command_listener(runner.clone(), [gripcmd_tx0, gripcmd_tx1]),
            );
            spawn_listener(
                &mut set,
                streams::run_joint_state_listener(runner.clone(), [meas_tx0, meas_tx1]),
            );
            spawn_listener(
                &mut set,
                streams::run_gripper_state_listener(runner.clone(), [grip_tx0, grip_tx1]),
            );
            spawn_listener(
                &mut set,
                streams::run_governor_config_listener(runner.clone(), config_tx),
            );

            // The first task to finish is fatal: cancel the node so the daemon
            // restarts a clean process rather than running on with a dead
            // coordination loop or a missing action handler while reporting healthy.
            if let Some(joined) = set.join_next().await {
                match joined {
                    Ok(Ok(())) => error!("backbone task exited; shutting node down"),
                    Ok(Err(e)) => error!(error = %e, "backbone task failed; shutting node down"),
                    Err(e) if e.is_panic() => {
                        error!(error = %e, "backbone task panicked; shutting node down")
                    }
                    Err(e) => error!(error = %e, "backbone task join failed; shutting node down"),
                }
            }
            token.cancel();
            set.shutdown().await;
        });

        Ok(())
    })
}
