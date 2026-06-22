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
use crate::state::{ARM_DOF, ArmTarget, GripperTarget, SharedState, Side, UiState};

const DEFAULT_PORT: u16 = 8765;
const SNAPSHOT_INTERVAL: Duration = Duration::from_millis(100);
const FEEDBACK_HZ: u32 = 20;
const INDEX_HTML: &str = include_str!("../static/index.html");

// Joint + gripper ranges from the robot model — the single source for slider
// bounds (via the WS snapshot) and for clamping incoming commands.
const JOINT_LIMITS_SRC: &str = include_str!("../config/joint_limits.json5");

#[derive(Clone, Copy, Deserialize)]
struct JointLimits {
    gripper: [f64; 2],
    left: [[f64; 2]; ARM_DOF],
    right: [[f64; 2]; ARM_DOF],
}

impl JointLimits {
    fn arm(&self, side: Side) -> &[[f64; 2]; ARM_DOF] {
        match side {
            Side::Left => &self.left,
            Side::Right => &self.right,
        }
    }
}

fn joint_limits() -> &'static JointLimits {
    static LIMITS: std::sync::OnceLock<JointLimits> = std::sync::OnceLock::new();
    LIMITS.get_or_init(|| {
        json5::from_str(JOINT_LIMITS_SRC).expect("config/joint_limits.json5 must parse")
    })
}

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
    let limits = joint_limits();
    match cmd {
        Command::FireArm { side, mut joints, duration_s } => {
            let side: Side = side.into();
            for (j, &[lo, hi]) in joints.iter_mut().zip(limits.arm(side).iter()) {
                *j = j.clamp(lo, hi);
            }
            // The arm floors the duration at its velocity-limit minimum; this
            // guard only catches garbage input (NaN, negative, absurd).
            let duration_s = if duration_s.is_finite() { duration_s.clamp(0.0, 30.0) } else { 0.0 };
            fire_arm(app, side, joints, duration_s).await;
        }
        Command::FireGripper { side, position } => {
            let position = position.clamp(limits.gripper[0], limits.gripper[1]);
            fire_gripper(app, side.into(), position).await;
        }
    }
}

async fn fire_arm(app: &AppState, side: Side, joints: [f64; ARM_DOF], duration_s: f64) {
    // Preempt: a Send while a goal is in flight cancels the old one (the arm's
    // single-flight gate would otherwise reject the new goal) and waits for it
    // to finalize before firing. The cancelled goal's feedback loop exits
    // promptly, so in_flight clears within the cancel round-trip.
    let preempt = {
        let s = app.state.lock().unwrap_or_else(|p| p.into_inner());
        if s.arm(side).in_flight { s.arm(side).preempt.clone() } else { None }
    };
    if let Some(tok) = preempt {
        tok.cancel();
        for _ in 0..50 {
            tokio::time::sleep(std::time::Duration::from_millis(20)).await;
            let clear = !app
                .state
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .arm(side)
                .in_flight;
            if clear {
                break;
            }
        }
        // Grace for the arm to release its busy gate after the result lands.
        tokio::time::sleep(std::time::Duration::from_millis(50)).await;
    }
    let goal_preempt = tokio_util::sync::CancellationToken::new();
    {
        let mut s = app.state.lock().unwrap_or_else(|p| p.into_inner());
        if s.arm(side).in_flight {
            s.set_status(format!("{} arm: previous goal still finishing", side.label()));
            return;
        }
        s.arm_mut(side).in_flight = true;
        s.arm_mut(side).joints = joints;
        s.arm_mut(side).preempt = Some(goal_preempt.clone());
        s.set_status(format!("{} arm: firing move_arm_joints", side.label()));
    }
    move_arm_joints::spawn(
        app.runner.clone(),
        app.state.clone(),
        app.token.clone(),
        goal_preempt,
        side,
        joints,
        duration_s,
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
    // Per-joint [min, max] (rad) — the browser bounds its sliders with these.
    limits: [[f64; 2]; ARM_DOF],
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
            left_arm: arm_view(&s.left_arm, Side::Left),
            right_arm: arm_view(&s.right_arm, Side::Right),
            left_gripper: gripper_view(&s.left_gripper),
            right_gripper: gripper_view(&s.right_gripper),
            status: s.status.clone(),
        }
    }
}

fn arm_view(a: &ArmTarget, side: Side) -> ArmView {
    ArmView {
        joints: a.joints,
        feedback: a.last_feedback,
        in_flight: a.in_flight,
        limits: *joint_limits().arm(side),
    }
}

fn gripper_view(g: &GripperTarget) -> GripperView {
    let [min, max] = joint_limits().gripper;
    GripperView {
        position: g.position,
        feedback: g.last_feedback.clone(),
        in_flight: g.in_flight,
        min,
        max,
    }
}

#[derive(Deserialize)]
#[serde(tag = "cmd", rename_all = "snake_case")]
enum Command {
    FireArm {
        side: SideWire,
        joints: [f64; ARM_DOF],
        // Requested move duration (s); 0 = fastest safe. Default keeps older
        // clients without the field working.
        #[serde(default)]
        duration_s: f64,
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
