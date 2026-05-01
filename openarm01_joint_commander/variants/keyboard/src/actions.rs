use std::sync::Arc;

use peppygen::NodeRunner;
use tokio::sync::mpsc;
use tracing::info;

use crate::config::ArmConfig;
use crate::input::HELP_TEXT;
use crate::types::{CartesianTarget, Command};

const FIXED_ORIENTATION: [f64; 4] = [0.0, 0.0, 0.0, 1.0];
const FEEDBACK_FREQUENCY: u32 = 10;

pub async fn run_command_loop(
    runner: Arc<NodeRunner>,
    config: Arc<ArmConfig>,
    mut rx: mpsc::Receiver<Command>,
) {
    let mut target = CartesianTarget::zero();
    let mut step = config.default_step;

    println!("arm: {}  |  step: {step:.4} m  |  type 'help' for commands", config.label);

    'outer: while let Some(cmd) = rx.recv().await {
        match cmd {
            Command::Help => { println!("{HELP_TEXT}"); continue; }
            Command::Quit => { info!("quit received"); break; }
            Command::SetStep(s) => {
                if s.is_finite() && s > 0.0 {
                    step = s;
                    println!("step → {step:.4} m");
                } else {
                    println!("step: invalid value {s} — must be finite and positive");
                }
                continue;
            }
            Command::Nudge { axis, delta } => target.nudge(axis, delta.unwrap_or(step)),
            Command::Goto { x, y, z } => { target.x = x; target.y = y; target.z = z; }
            Command::Reset => target = CartesianTarget::zero(),
        }

        // Drain any commands that queued while the previous goal was in flight
        // so we send only the latest target instead of replaying stale inputs.
        loop {
            match rx.try_recv() {
                Ok(Command::Quit) => { info!("quit received"); break 'outer; }
                Ok(Command::Help) => println!("{HELP_TEXT}"),
                Ok(Command::SetStep(s)) => {
                    if s.is_finite() && s > 0.0 { step = s; }
                    else { println!("step: invalid value {s} — must be finite and positive"); }
                }
                Ok(Command::Nudge { axis, delta }) => target.nudge(axis, delta.unwrap_or(step)),
                Ok(Command::Goto { x, y, z }) => { target.x = x; target.y = y; target.z = z; }
                Ok(Command::Reset) => target = CartesianTarget::zero(),
                Err(_) => break,
            }
        }

        println!("target: x={:+.4}  y={:+.4}  z={:+.4}", target.x, target.y, target.z);
        send_move_arm(&runner, &config, target).await;
    }
}

async fn send_move_arm(
    runner: &Arc<NodeRunner>,
    config: &ArmConfig,
    target: CartesianTarget,
) {
    use peppygen::consumed_actions::openarm01_backbone_move_arm as move_arm;
    use peppylib::config::QoSProfile;
    use std::time::Duration;

    let pos = target.as_array();

    info!(arm = %config.label, x = pos[0], y = pos[1], z = pos[2], "move_arm: sending goal");

    let mut handle = match move_arm::ActionHandle::fire_goal(
        runner,
        Duration::from_secs(5),
        None,
        None,
        move_arm::GoalRequest {
            feedback_frequency: FEEDBACK_FREQUENCY,
            desired_position: pos,
            desired_orientation: FIXED_ORIENTATION,
        },
        QoSProfile::SensorData,
    )
    .await
    {
        Ok(h) if h.data.accepted => h,
        Ok(_) => {
            tracing::warn!("move_arm: goal rejected by backbone");
            return;
        }
        Err(e) => {
            tracing::warn!("move_arm: fire_goal failed — {e}");
            return;
        }
    };

    loop {
        match handle.on_next_feedback_message().await {
            Ok(fb) => {
                let ee = fb.current_ee_position;
                info!(x = ee[0], y = ee[1], z = ee[2], t = fb.action_time, "move_arm: feedback");
            }
            Err(_) => break,
        }
    }

    match handle.get_result(Duration::from_secs(30)).await {
        Ok(r) => info!(
            success = r.data.success,
            action_time = r.data.action_time,
            "move_arm: done"
        ),
        Err(e) => tracing::warn!("move_arm: get_result failed — {e}"),
    }
}
