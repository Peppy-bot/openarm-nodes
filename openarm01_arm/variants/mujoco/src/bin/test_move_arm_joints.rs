// Standalone harness that fires move_arm_joints at running arm instances and
// prints feedback + result. Mirrors test_move_gripper but for 7-DOF joint
// targets. Uses the raw peppylib ActionMessenger path so no Python peppygen
// or venv is needed.
//
// Run from variants/mujoco/:
//     cargo run --release --features test-tools --bin test_move_arm_joints -- \
//         [--side left|right|both] \
//         [--positions "q1,q2,q3,q4,q5,q6,q7"] \
//         [--feedback-hz N]
use std::sync::Arc;
use std::time::Duration;

use peppylib::config::QoSProfile;
use peppylib::runtime::{NodeBuilder, NodeRunner, StandaloneConfig};
use peppylib::{ActionMessenger, Payload};
use tracing::{error, info, warn};

use peppygen::capnp::{
    emit_move_arm_joints_feedback_message_capnp::move_arm_joints_feedback_message,
    move_arm_joints_goal_message_capnp::move_arm_joints_goal_message,
    move_arm_joints_goal_response_message_capnp::move_arm_joints_goal_response_message,
    move_arm_joints_result_response_message_capnp::move_arm_joints_result_response_message,
};

const TARGET_NODE: &str = "openarm01_arm";
const ACTION_NAME: &str = "move_arm_joints";
const DOF: usize = 7;
// Default neutral pose — all joints at zero. Replaces gripper's `position`
// default (0.044 = fully open). For the arm "fully neutral" = zeros.
const DEFAULT_POSITIONS: [f64; DOF] = [0.0; DOF];
const JOINT_MIN_RAD: f64 = -std::f64::consts::PI;
const JOINT_MAX_RAD: f64 =  std::f64::consts::PI;

#[derive(serde::Deserialize, schemars::JsonSchema, Default)]
struct NoParams {}

#[derive(Clone, Copy)]
enum SideArg {
    Left,
    Right,
    Both,
}

struct Args {
    side: SideArg,
    positions: [f64; DOF],
    feedback_hz: u32,
}

impl Args {
    fn instances(&self) -> Vec<(&'static str, &'static str)> {
        match self.side {
            SideArg::Left => vec![("left", "left_arm")],
            SideArg::Right => vec![("right", "right_arm")],
            SideArg::Both => vec![("left", "left_arm"), ("right", "right_arm")],
        }
    }
}

fn parse_positions(raw: &str) -> [f64; DOF] {
    let parts: Vec<&str> = raw.split(',').map(|s| s.trim()).collect();
    if parts.len() != DOF {
        eprintln!(
            "--positions: expected {DOF} comma-separated values, got {}",
            parts.len()
        );
        std::process::exit(2);
    }
    let mut out = [0.0_f64; DOF];
    for (i, p) in parts.iter().enumerate() {
        let v: f64 = p.parse().unwrap_or_else(|_| {
            eprintln!("--positions: '{p}' is not a valid f64");
            std::process::exit(2);
        });
        if !(JOINT_MIN_RAD..=JOINT_MAX_RAD).contains(&v) {
            eprintln!(
                "--positions: joint {} value {v} out of range [{JOINT_MIN_RAD}, {JOINT_MAX_RAD}]",
                i + 1,
            );
            std::process::exit(2);
        }
        out[i] = v;
    }
    out
}

fn parse_args() -> Args {
    let mut side = SideArg::Both;
    let mut positions = DEFAULT_POSITIONS;
    let mut feedback_hz: u32 = 10;
    let mut it = std::env::args().skip(1);
    while let Some(a) = it.next() {
        match a.as_str() {
            "--side" => {
                side = match it.next().as_deref() {
                    Some("left") => SideArg::Left,
                    Some("right") => SideArg::Right,
                    Some("both") => SideArg::Both,
                    other => {
                        eprintln!("--side: expected left|right|both, got {other:?}");
                        std::process::exit(2);
                    }
                }
            }
            "--positions" => {
                positions = parse_positions(&it.next().unwrap_or_else(|| {
                    eprintln!("--positions: expected comma-separated f64s");
                    std::process::exit(2);
                }));
            }
            "--feedback-hz" => {
                feedback_hz = it.next().and_then(|v| v.parse().ok()).unwrap_or_else(|| {
                    eprintln!("--feedback-hz: expected u32");
                    std::process::exit(2);
                });
            }
            "-h" | "--help" => {
                println!(
                    "usage: test_move_arm_joints [--side left|right|both] \
                     [--positions q1,q2,q3,q4,q5,q6,q7] [--feedback-hz N]"
                );
                std::process::exit(0);
            }
            other => {
                eprintln!("unknown arg: {other}");
                std::process::exit(2);
            }
        }
    }
    Args {
        side,
        positions,
        feedback_hz,
    }
}

fn encode_goal(feedback_hz: u32, positions: &[f64; DOF]) -> Vec<u8> {
    let mut msg = capnp::message::Builder::new_default();
    {
        let mut root = msg.init_root::<move_arm_joints_goal_message::Builder>();
        root.set_feedback_frequency(feedback_hz);
        let mut list = root.reborrow().init_joint_positions(DOF as u32);
        for (i, &q) in positions.iter().enumerate() {
            list.set(i as u32, q);
        }
    }
    let mut buf = Vec::new();
    capnp::serialize::write_message(&mut buf, &msg).expect("encode goal");
    buf
}

fn decode_goal_response(payload: &[u8]) -> capnp::Result<bool> {
    let mut slice = payload;
    let r = capnp::serialize::read_message_from_flat_slice(&mut slice, Default::default())?;
    Ok(r.get_root::<move_arm_joints_goal_response_message::Reader>()?
        .get_accepted())
}

fn decode_feedback(payload: &[u8]) -> capnp::Result<(Vec<f64>, f64)> {
    let mut slice = payload;
    let r = capnp::serialize::read_message_from_flat_slice(&mut slice, Default::default())?;
    let fb = r.get_root::<move_arm_joints_feedback_message::Reader>()?;
    let qpos: Vec<f64> = fb.get_joint_positions()?.iter().collect();
    Ok((qpos, fb.get_action_time()))
}

fn decode_result(payload: &[u8]) -> capnp::Result<(bool, String, Vec<f64>, f64)> {
    let mut slice = payload;
    let r = capnp::serialize::read_message_from_flat_slice(&mut slice, Default::default())?;
    let res = r.get_root::<move_arm_joints_result_response_message::Reader>()?;
    let message = res.get_message()?.to_str().unwrap_or("").to_string();
    let final_pos: Vec<f64> = res.get_final_joint_positions()?.iter().collect();
    Ok((res.get_success(), message, final_pos, res.get_action_time()))
}

async fn fire_one(
    runner: &Arc<NodeRunner>,
    side: &str,
    target_instance: &str,
    positions: &[f64; DOF],
    feedback_hz: u32,
) -> bool {
    info!(
        "[{side}] sending move_arm_joints positions={positions:?} feedback_hz={feedback_hz}"
    );
    let payload = Payload::from(encode_goal(feedback_hz, positions));
    let mut handle = match ActionMessenger::send_goal(
        runner.messenger(),
        runner.processor().bound_core_node(),
        runner.processor().bound_instance_id(),
        TARGET_NODE,
        ACTION_NAME,
        None,
        Some(target_instance),
        payload,
        QoSProfile::Standard,
        Duration::from_secs(5),
    )
    .await
    {
        Ok(h) => h,
        Err(e) => {
            error!("[{side}] send_goal failed: {e:?}");
            return false;
        }
    };

    let resp_payload = handle.goal_response().payload();
    let accepted = match decode_goal_response(resp_payload.as_ref()) {
        Ok(b) => b,
        Err(e) => {
            error!("[{side}] decode goal response failed: {e:?}");
            return false;
        }
    };
    if !accepted {
        warn!("[{side}] goal rejected by node");
        return false;
    }
    info!("[{side}] goal accepted");

    let mut count = 0u32;
    loop {
        let next = tokio::time::timeout(Duration::from_secs(2), handle.on_next_feedback()).await;
        let fb_msg = match next {
            Ok(Ok(m)) => m,
            Ok(Err(e)) => {
                let s = format!("{e:?}").to_lowercase();
                if s.contains("closed") {
                    info!("[{side}] feedback channel closed");
                } else {
                    error!("[{side}] feedback error: {s}");
                }
                break;
            }
            Err(_) => {
                info!("[{side}] feedback drained (no msg in 2s)");
                break;
            }
        };
        match decode_feedback(fb_msg.payload().as_ref()) {
            Ok((qpos, t)) => info!("[{side}] feedback #{count}: qpos={qpos:?} t={t:.3}s"),
            Err(e) => warn!("[{side}] decode feedback failed: {e:?}"),
        }
        count += 1;
        if count > 2000 {
            warn!("[{side}] exceeded 2000 feedbacks — bailing");
            break;
        }
    }

    match ActionMessenger::request_result(runner.messenger(), &handle, Duration::from_secs(30)).await
    {
        Ok(res) => match decode_result(res.payload().as_ref()) {
            Ok((success, message, final_pos, t)) => {
                info!(
                    "[{side}] RESULT success={success} msg={message:?} final={final_pos:?} t={t:.3}s"
                );
                success
            }
            Err(e) => {
                error!("[{side}] decode result failed: {e:?}");
                false
            }
        },
        Err(e) => {
            error!("[{side}] request_result failed: {e:?}");
            false
        }
    }
}

fn main() -> peppylib::PeppyResult<()> {
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::INFO)
        .init();
    let args = parse_args();

    // Standalone runtime loads variants/mujoco/peppy.json5 for param validation
    // and the manifest declares arm_id as required. We're not actually an arm
    // instance, but passing a dummy 0 satisfies the validator. The test code
    // below ignores `_params`.
    let config = StandaloneConfig::new()
        .with_node_name("test_move_arm_joints_caller")
        .with_messaging("127.0.0.1", 7448)
        .with_instance_id("test-caller")
        .with_parameters_json(serde_json::json!({ "arm_id": 0 }));

    NodeBuilder::<NoParams>::new()
        .standalone(config)
        .run(move |_params, runner: Arc<NodeRunner>| async move {
            let mut all_ok = true;
            for (side, instance_id) in args.instances() {
                let ok = fire_one(
                    &runner, side, instance_id, &args.positions, args.feedback_hz,
                ).await;
                all_ok = all_ok && ok;
            }
            info!("OVERALL: {}", if all_ok { "PASS" } else { "FAIL" });
            if !all_ok {
                std::process::exit(1);
            }
            Ok(())
        })
}
