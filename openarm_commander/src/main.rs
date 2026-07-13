mod collision_status;
mod command_stream;
mod error;
mod gripper_states;
mod joint_states;
mod move_arm;
mod move_arm_joints;
mod move_gripper;
mod owner;
mod pose;
mod state;
mod ui;

use openarm_description::HardwareVersion;
use peppygen::{NodeBuilder, Parameters, Result};
use tokio::sync::{mpsc, watch};
use tracing::error;

// Channel depths: commands are operator-paced (small); feedback bursts across the
// state streams and goal tasks (larger), but the owner drains both far faster than
// they fill.
const COMMAND_CAP: usize = 64;
const FEEDBACK_CAP: usize = 256;

fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .with_writer(std::io::stderr)
        .init();

    NodeBuilder::new().run(|params: Parameters, node_runner| async move {
        let token = node_runner.cancellation_token().clone();
        // The generation picks the panel's joint/gripper ranges (URDF limits +
        // jaw width); everything else in the commander is version-blind.
        let version: HardwareVersion = params
            .hardware_version
            .parse()
            .unwrap_or_else(|e| panic!("hardware_version: {e}"));
        ui::init_limits(version);
        // Arm models for the panel's Cartesian pose fields (pose <-> joints), built
        // from the same generation the ranges came from so FK/IK match the backbone's chain.
        let models = pose::ArmModels::from_version(version);
        // The operator streams the governor controls live; their launch defaults are
        // node parameters, kept in step with the backbone's, so the real arm starts
        // conservative (tight band, slow cap) and the sim launchers start fast.
        assert!(
            params.max_ee_velocity_m_s.is_finite() && params.max_ee_velocity_m_s > 0.0,
            "max_ee_velocity_m_s must be a positive finite number"
        );
        assert!(
            params.d_stop.is_finite()
                && params.d_safe.is_finite()
                && params.d_stop > 0.0
                && params.d_stop < params.d_safe,
            "governor band must satisfy 0 < d_stop < d_safe"
        );
        let state = state::UiState::new(
            params.collision_governor_enabled,
            params.d_stop,
            params.d_safe,
            params.max_ee_velocity_m_s,
        );

        // Rate feeds `Duration::from_micros(1_000_000 / rate)`, so a rate above
        // 1 MHz would round to a 0 µs period; no real deployment approaches that,
        // so just guard against zero.
        assert!(params.command_rate_hz > 0, "command_rate_hz must be > 0");

        // The state owner is the one task that touches UiState; everything else holds a
        // channel end. Commands flow in from the WS, feedback in from the state streams
        // and the goal tasks, and the owner publishes the browser snapshot and the
        // per-tick command frame the publishers stream.
        let (command_tx, command_rx) = mpsc::channel::<owner::UiMsg>(COMMAND_CAP);
        let (feedback_tx, feedback_rx) = mpsc::channel::<owner::Feedback>(FEEDBACK_CAP);
        let (frame_tx, frame_rx) = watch::channel(owner::CommandFrame::from_state(&state));
        let (snapshot_tx, snapshot_rx) = watch::channel(String::new());

        // Feed the owner live arm + gripper + proximity state off the always-on streams.
        tokio::spawn(joint_states::run(
            node_runner.clone(),
            feedback_tx.clone(),
            token.clone(),
        ));
        tokio::spawn(gripper_states::run(
            node_runner.clone(),
            feedback_tx.clone(),
            token.clone(),
        ));
        tokio::spawn(collision_status::run(
            node_runner.clone(),
            feedback_tx.clone(),
            token.clone(),
        ));

        // The always-on publisher: streams each enabled side's governed setpoint from
        // the owner's command frame at command_rate_hz. A disabled side has None in the
        // frame, so nothing is published and the backbone holds its last setpoint.
        tokio::spawn(command_stream::run(
            node_runner.clone(),
            params.command_rate_hz,
            token.clone(),
            frame_rx,
        ));

        // The owner: advances jogs, reduces commands + feedback, and publishes both
        // watches. It owns `state` and `models` and holds the runner to spawn goal tasks.
        tokio::spawn(owner::run(
            state,
            models,
            node_runner,
            params.command_rate_hz,
            token.clone(),
            command_rx,
            feedback_rx,
            feedback_tx,
            frame_tx,
            snapshot_tx,
        ));

        // ui::run is the long-lived HTTP + WebSocket server. It must be spawned rather
        // than awaited here: peppylib registers `node_health` only after the setup
        // closure returns, so awaiting a forever-task starves the health probe and the
        // daemon SIGKILLs the instance after ~10s.
        tokio::spawn(async move {
            if let Err(e) = ui::run(command_tx, snapshot_rx, token).await {
                error!(error = %e, "ui server exited with error");
            }
        });
        Ok(())
    })
}
