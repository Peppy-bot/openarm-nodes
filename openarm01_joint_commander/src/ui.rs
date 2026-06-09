// HTTP+WS UI on 127.0.0.1:PEPPY_JC_PORT (default 8765). Loopback only because
// the WS exposes unauthenticated motion control — set PEPPY_JC_BIND_IP for a
// remote operator on a trusted network.

use std::env;
use std::net::{IpAddr, Ipv4Addr, SocketAddr};
use std::sync::Arc;
use std::time::Duration;

use axum::Router;
use axum::extract::State;
use axum::extract::ws::{Message, Utf8Bytes, WebSocket, WebSocketUpgrade};
use axum::response::{Html, IntoResponse};
use axum::routing::get;
use peppygen::NodeRunner;
use peppylib::runtime::CancellationToken;
use serde::{Deserialize, Serialize};
use tokio::net::TcpListener;
use tracing::{info, warn};

use crate::actions::{move_arm_joints, move_gripper};
use crate::error::Result;
use crate::state::{
    ARM_DOF, ArmTarget, GRIPPER_CLOSED_M, GRIPPER_OPEN_M, GripperTarget, JOINT_LIMIT_RAD,
    SharedState, Side, UiState,
};

const DEFAULT_PORT: u16 = 8765;
const SNAPSHOT_INTERVAL: Duration = Duration::from_millis(100);
const FEEDBACK_HZ: u32 = 20;
const INDEX_HTML: &str = include_str!("../static/index.html");

#[derive(Clone)]
struct AppState {
    runner: Arc<NodeRunner>,
    state: SharedState,
    token: CancellationToken,
}

pub async fn run(
    runner: Arc<NodeRunner>,
    state: SharedState,
    token: CancellationToken,
) -> Result<()> {
    let port = env::var("PEPPY_JC_PORT")
        .ok()
        .and_then(|s| s.parse::<u16>().ok())
        .unwrap_or(DEFAULT_PORT);
    let bind_ip = env::var("PEPPY_JC_BIND_IP")
        .ok()
        .and_then(|s| s.parse::<IpAddr>().ok())
        .unwrap_or(IpAddr::V4(Ipv4Addr::LOCALHOST));
    let addr = SocketAddr::new(bind_ip, port);

    let app_state = AppState {
        runner,
        state,
        token: token.clone(),
    };
    let app = Router::new()
        .route("/", get(index))
        .route("/ws", get(ws_upgrade))
        .with_state(app_state);

    let listener = TcpListener::bind(addr).await?;
    info!("joint commander UI at http://localhost:{port}");

    let shutdown_token = token.clone();
    axum::serve(listener, app)
        .with_graceful_shutdown(async move { shutdown_token.cancelled().await })
        .await?;
    Ok(())
}

async fn index() -> impl IntoResponse {
    Html(INDEX_HTML)
}

async fn ws_upgrade(ws: WebSocketUpgrade, State(app): State<AppState>) -> impl IntoResponse {
    ws.on_upgrade(move |socket| ws_handle(socket, app))
}

async fn ws_handle(mut socket: WebSocket, app: AppState) {
    let mut tick = tokio::time::interval(SNAPSHOT_INTERVAL);
    loop {
        tokio::select! {
            _ = app.token.cancelled() => break,
            _ = tick.tick() => {
                let snap = {
                    let s = app.state.lock().unwrap_or_else(|p| p.into_inner());
                    Snapshot::from(&*s)
                };
                let json = match serde_json::to_string(&snap) {
                    Ok(j) => j,
                    Err(e) => { warn!(error = %e, "ws: serialize snapshot"); continue; }
                };
                if socket.send(Message::Text(Utf8Bytes::from(json))).await.is_err() {
                    break;
                }
            }
            msg = socket.recv() => match msg {
                Some(Ok(Message::Text(text))) => handle_command(text.as_str(), &app).await,
                Some(Ok(Message::Close(_))) | None => break,
                Some(Err(e)) => { warn!(error = %e, "ws: recv"); break; }
                _ => {}
            }
        }
    }
}

async fn handle_command(text: &str, app: &AppState) {
    let cmd: Command = match serde_json::from_str(text) {
        Ok(c) => c,
        Err(e) => {
            warn!(error = %e, payload = text, "ws: bad command");
            return;
        }
    };
    match cmd {
        Command::FireArm { side, mut joints } => {
            for j in &mut joints {
                *j = j.clamp(-JOINT_LIMIT_RAD, JOINT_LIMIT_RAD);
            }
            fire_arm(app, side.into(), joints).await;
        }
        Command::FireGripper { side, position } => {
            let position = position.clamp(GRIPPER_CLOSED_M, GRIPPER_OPEN_M);
            fire_gripper(app, side.into(), position).await;
        }
    }
}

async fn fire_arm(app: &AppState, side: Side, joints: [f64; ARM_DOF]) {
    {
        let mut s = app.state.lock().unwrap_or_else(|p| p.into_inner());
        if s.arm(side).in_flight {
            s.set_status(format!(
                "{} arm: previous goal still in flight",
                side.label()
            ));
            return;
        }
        s.arm_mut(side).in_flight = true;
        s.arm_mut(side).joints = joints;
        s.set_status(format!("{} arm: firing move_arm_joints", side.label()));
    }
    move_arm_joints::spawn(
        app.runner.clone(),
        app.state.clone(),
        app.token.clone(),
        side,
        joints,
        FEEDBACK_HZ,
    );
}

async fn fire_gripper(app: &AppState, side: Side, position_m: f64) {
    {
        let mut s = app.state.lock().unwrap_or_else(|p| p.into_inner());
        if s.gripper(side).in_flight {
            s.set_status(format!(
                "{} gripper: previous goal still in flight",
                side.label()
            ));
            return;
        }
        s.gripper_mut(side).in_flight = true;
        s.gripper_mut(side).position = position_m;
        s.set_status(format!(
            "{} gripper: firing move_gripper ({position_m:.4} m)",
            side.label()
        ));
    }
    move_gripper::spawn(
        app.runner.clone(),
        app.state.clone(),
        app.token.clone(),
        side,
        position_m,
        FEEDBACK_HZ,
    );
}

// --------------------------- wire protocol ---------------------------

#[derive(Serialize)]
struct Snapshot {
    left_arm: ArmView,
    right_arm: ArmView,
    left_gripper: GripperView,
    right_gripper: GripperView,
    status: String,
}

#[derive(Serialize)]
struct ArmView {
    joints: [f64; ARM_DOF],
    feedback: Option<[f64; ARM_DOF]>,
    in_flight: bool,
}

#[derive(Serialize)]
struct GripperView {
    position: f64,
    feedback: Option<Vec<f64>>,
    in_flight: bool,
    min: f64,
    max: f64,
}

impl From<&UiState> for Snapshot {
    fn from(s: &UiState) -> Self {
        Self {
            left_arm: ArmView::from(&s.left_arm),
            right_arm: ArmView::from(&s.right_arm),
            left_gripper: GripperView::from(&s.left_gripper),
            right_gripper: GripperView::from(&s.right_gripper),
            status: s.status.clone(),
        }
    }
}

impl From<&ArmTarget> for ArmView {
    fn from(a: &ArmTarget) -> Self {
        Self {
            joints: a.joints,
            feedback: a.last_feedback,
            in_flight: a.in_flight,
        }
    }
}

impl From<&GripperTarget> for GripperView {
    fn from(g: &GripperTarget) -> Self {
        Self {
            position: g.position,
            feedback: g.last_feedback.clone(),
            in_flight: g.in_flight,
            min: GRIPPER_CLOSED_M,
            max: GRIPPER_OPEN_M,
        }
    }
}

#[derive(Deserialize)]
#[serde(tag = "cmd", rename_all = "snake_case")]
enum Command {
    FireArm {
        side: SideWire,
        joints: [f64; ARM_DOF],
    },
    FireGripper {
        side: SideWire,
        position: f64,
    },
}

#[derive(Deserialize, Copy, Clone)]
#[serde(rename_all = "lowercase")]
enum SideWire {
    Left,
    Right,
}

impl From<SideWire> for Side {
    fn from(s: SideWire) -> Self {
        match s {
            SideWire::Left => Side::Left,
            SideWire::Right => Side::Right,
        }
    }
}
