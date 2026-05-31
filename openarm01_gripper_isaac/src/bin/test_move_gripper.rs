// Standalone harness that fires move_gripper at running gripper instances and
// prints feedback + result. Rust port of test_move_gripper.py — uses the same
// raw peppylib ActionMessenger path so no Python peppygen / venv is needed.
//
// Run from variants/isaac/:
//     cargo run --release --bin test_move_gripper -- \
//         [--side left|right|both] [--position 0..=0.044] [--feedback-hz N]
use std::sync::Arc;
use std::time::Duration;

use peppylib::config::QoSProfile;
use peppylib::runtime::{NodeBuilder, NodeRunner, StandaloneConfig};
use peppylib::{ActionMessenger, Payload};
use tracing::{error, info, warn};

use peppygen::capnp::{
    emit_move_gripper_feedback_message_capnp::move_gripper_feedback_message,
    move_gripper_goal_message_capnp::move_gripper_goal_message,
    move_gripper_goal_response_message_capnp::move_gripper_goal_response_message,
    move_gripper_result_response_message_capnp::move_gripper_result_response_message,
};

const TARGET_NODE: &str = "openarm01_gripper";
const ACTION_NAME: &str = "move_gripper";

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
    position: f64,
    feedback_hz: u32,
}

impl Args {
    fn instances(&self) -> Vec<(&'static str, &'static str)> {
        match self.side {
            SideArg::Left => vec![("left", "left_gripper")],
            SideArg::Right => vec![("right", "right_gripper")],
            SideArg::Both => vec![("left", "left_gripper"), ("right", "right_gripper")],
        }
    }
}

fn parse_args() -> Args {
    let mut side = SideArg::Both;
    let mut position = 0.044_f64;
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
            "--position" => {
                position = it.next().and_then(|v| v.parse().ok()).unwrap_or_else(|| {
                    eprintln!("--position: expected f64");
                    std::process::exit(2);
                });
            }
            "--feedback-hz" => {
                feedback_hz = it.next().and_then(|v| v.parse().ok()).unwrap_or_else(|| {
                    eprintln!("--feedback-hz: expected u32");
                    std::process::exit(2);
                });
            }
            "-h" | "--help" => {
                println!(
                    "usage: test_move_gripper [--side left|right|both] \
                     [--position 0..=0.044] [--feedback-hz N]"
                );
                std::process::exit(0);
            }
            other => {
                eprintln!("unknown arg: {other}");
                std::process::exit(2);
            }
        }
    }
    if !(0.0..=0.044).contains(&position) {
        eprintln!("position out of range [0.0, 0.044]: {position}");
        std::process::exit(2);
    }
    Args {
        side,
        position,
        feedback_hz,
    }
}

fn encode_goal(feedback_hz: u32, position_m: f64) -> Vec<u8> {
    let mut msg = capnp::message::Builder::new_default();
    {
        let mut root = msg.init_root::<move_gripper_goal_message::Builder>();
        root.set_feedback_frequency(feedback_hz);
        root.set_position(position_m);
    }
    let mut buf = Vec::new();
    capnp::serialize::write_message(&mut buf, &msg).expect("encode goal");
    buf
}

fn decode_goal_response(payload: &[u8]) -> capnp::Result<bool> {
    let mut slice = payload;
    let r = capnp::serialize::read_message_from_flat_slice(&mut slice, Default::default())?;
    Ok(r.get_root::<move_gripper_goal_response_message::Reader>()?
        .get_accepted())
}

fn decode_feedback(payload: &[u8]) -> capnp::Result<(Vec<f64>, f64)> {
    let mut slice = payload;
    let r = capnp::serialize::read_message_from_flat_slice(&mut slice, Default::default())?;
    let fb = r.get_root::<move_gripper_feedback_message::Reader>()?;
    let qpos: Vec<f64> = fb.get_joint_positions()?.iter().collect();
    Ok((qpos, fb.get_action_time()))
}

fn decode_result(payload: &[u8]) -> capnp::Result<(bool, String, Vec<f64>, f64)> {
    let mut slice = payload;
    let r = capnp::serialize::read_message_from_flat_slice(&mut slice, Default::default())?;
    let res = r.get_root::<move_gripper_result_response_message::Reader>()?;
    let message = res.get_message()?.to_str().unwrap_or("").to_string();
    let final_pos: Vec<f64> = res.get_final_joint_positions()?.iter().collect();
    Ok((res.get_success(), message, final_pos, res.get_action_time()))
}

async fn fire_one(
    runner: &Arc<NodeRunner>,
    side: &str,
    target_instance: &str,
    position: f64,
    feedback_hz: u32,
) -> bool {
    info!("[{side}] sending move_gripper position={position} feedback_hz={feedback_hz}");
    let payload = Payload::from(encode_goal(feedback_hz, position));
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
        if count > 1000 {
            warn!("[{side}] exceeded 1000 feedbacks — bailing");
            break;
        }
    }

    match ActionMessenger::request_result(runner.messenger(), &handle, Duration::from_secs(10)).await
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

    // Standalone runtime loads variants/isaac/peppy.json5 for param validation
    // and the manifest declares gripper_id as required. We're not actually a
    // gripper instance, but passing a dummy 0 satisfies the validator. The
    // test code below ignores `_params`.
    let config = StandaloneConfig::new()
        .with_node_name("test_move_gripper_caller")
        .with_messaging("127.0.0.1", 7448)
        .with_instance_id("test-caller")
        .with_parameters_json(serde_json::json!({ "gripper_id": 0 }));

    NodeBuilder::<NoParams>::new()
        .standalone(config)
        .run(move |_params, runner: Arc<NodeRunner>| async move {
            let mut all_ok = true;
            for (side, instance_id) in args.instances() {
                let ok =
                    fire_one(&runner, side, instance_id, args.position, args.feedback_hz).await;
                all_ok = all_ok && ok;
            }
            info!("OVERALL: {}", if all_ok { "PASS" } else { "FAIL" });
            if !all_ok {
                std::process::exit(1);
            }
            Ok(())
        })
}
