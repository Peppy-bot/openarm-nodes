// HTTP+WS UI on 0.0.0.0:PEPPY_JC_PORT (default 8765). The WS exposes
// unauthenticated motion control, so only run on a trusted network; set
// PEPPY_JC_BIND_IP=127.0.0.1 to restrict to loopback.

use std::env;
use std::net::{IpAddr, Ipv4Addr, SocketAddr};
use std::sync::Arc;
use std::time::{Duration, Instant};

use axum::Router;
use axum::extract::State;
use axum::extract::ws::{Message, Utf8Bytes, WebSocket, WebSocketUpgrade};
use axum::response::{Html, IntoResponse};
use axum::routing::get;
use openarm_description::HardwareVersion;
use peppygen::NodeRunner;
use peppylib::runtime::CancellationToken;
use serde::{Deserialize, Serialize};
use tokio::net::TcpListener;
use tracing::{info, warn};

use crate::actions::move_arm_joints;
use crate::error::Result;
use crate::state::{ARM_DOF, ArmTarget, GripperTarget, Proximity, SharedState, Side, UiState};

const DEFAULT_PORT: u16 = 8765;
const SNAPSHOT_INTERVAL: Duration = Duration::from_millis(100);
// The hub publishes the proximity readout at ~20 Hz; treat it as stale after this
// long with no update (a dead hub) so the panel falls back to n/a instead of
// latching the last distance.
const PROXIMITY_STALE_AFTER: Duration = Duration::from_millis(500);
const INDEX_HTML: &str = include_str!("../static/index.html");

// Joint + gripper ranges from the generation's bundled URDF and jaw width; the
// single source for slider bounds (via the WS snapshot) and for clamping
// incoming commands. Resolved once at startup by [`init_limits`].
#[derive(Clone, Copy)]
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

    fn resolve(version: HardwareVersion) -> Self {
        Self {
            gripper: [0.0, version.jaw_open_m()],
            left: version.joint_limits(openarm_description::Side::Left),
            right: version.joint_limits(openarm_description::Side::Right),
        }
    }
}

static LIMITS: std::sync::OnceLock<JointLimits> = std::sync::OnceLock::new();

/// Resolve the panel's clamp/display ranges from the generation's description:
/// arm joints via its `joint_limits` (URDF limits with the elbow held off its
/// singularity floor, matching the hub's clamp) and the gripper from the jaw
/// width. Must run before the UI serves.
pub fn init_limits(version: HardwareVersion) {
    assert!(
        LIMITS.set(JointLimits::resolve(version)).is_ok(),
        "init_limits must run exactly once"
    );
}

fn joint_limits() -> &'static JointLimits {
    LIMITS
        .get()
        .expect("init_limits must run before the UI serves")
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
        .unwrap_or(IpAddr::V4(Ipv4Addr::UNSPECIFIED));
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
                    Snapshot::build(&s, Instant::now())
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
    let mut s = app.state.lock().unwrap_or_else(|p| p.into_inner());
    on_operator_disconnect(&mut s);
}

/// Reset on operator disconnect: drop the streaming deadman for both sides (each
/// node's stream timeout then releases to hold) and restore the governor enable to
/// its launch default.
fn on_operator_disconnect(s: &mut UiState) {
    for side in [Side::Left, Side::Right] {
        s.set_enabled(side, false);
    }
    s.collision_enabled = s.collision_enabled_default;
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
        Command::FireArm {
            side,
            mut joints,
            duration_s,
        } => {
            let side: Side = side.into();
            // A discrete move preempts the live stream, so refuse one while enabled
            // rather than relying on the UI to hide the button.
            {
                let mut s = app.state.lock().unwrap_or_else(|p| p.into_inner());
                if s.enabled(side) {
                    s.set_status(format!(
                        "{} arm: disable before a discrete move",
                        side.label()
                    ));
                    return;
                }
            }
            clamp_to_limits(&mut joints, side);
            // The arm floors the duration at its velocity-limit minimum; this
            // guard only catches garbage input (NaN, negative, absurd).
            let duration_s = if duration_s.is_finite() {
                duration_s.clamp(0.0, 30.0)
            } else {
                0.0
            };
            fire_arm(app, side, joints, duration_s).await;
        }
        Command::SetEnabled { side, on } => {
            let side: Side = side.into();
            let mut s = app.state.lock().unwrap_or_else(|p| p.into_inner());
            if on {
                // Enabling streams both the arm and the gripper for this side, so
                // seed each target on its measured value first; refuse until both
                // exist so the first emitted command holds position instead of
                // streaming a stale default.
                let (Some(arm_measured), Some(gripper_measured)) =
                    (s.arm(side).last_feedback, s.gripper(side).last_feedback)
                else {
                    s.set_status(format!(
                        "{}: no measured pose yet, not enabling",
                        side.label()
                    ));
                    return;
                };
                s.arm_mut(side).joints = arm_measured;
                s.gripper_mut(side).position = gripper_measured;
            }
            s.set_enabled(side, on);
            s.set_status(format!(
                "{}: {}",
                side.label(),
                if on {
                    "ENABLED, streaming arm + gripper"
                } else {
                    "disabled"
                }
            ));
        }
        Command::SetArmTarget { side, mut joints } => {
            let side: Side = side.into();
            clamp_to_limits(&mut joints, side);
            let mut s = app.state.lock().unwrap_or_else(|p| p.into_inner());
            if s.enabled(side) {
                s.arm_mut(side).joints = joints;
            }
        }
        Command::SetGripperTarget { side, position } => {
            let side: Side = side.into();
            let [lo, hi] = joint_limits().gripper;
            let position = position.clamp(lo, hi);
            let mut s = app.state.lock().unwrap_or_else(|p| p.into_inner());
            if s.enabled(side) {
                s.gripper_mut(side).position = position;
            }
        }
        Command::SetCollision { enabled } => {
            let mut s = app.state.lock().unwrap_or_else(|p| p.into_inner());
            s.collision_enabled = enabled;
            s.set_status(format!(
                "collision avoidance {}",
                if enabled { "ON" } else { "OFF" }
            ));
        }
        Command::SetGovernorParams {
            d_stop,
            d_safe,
            max_ee_velocity_m_s,
        } => {
            // The hub validates again before applying; reject a degenerate band here
            // so the UI cannot stream one (d_stop must stay below d_safe).
            if !valid_governor_band(d_stop, d_safe, max_ee_velocity_m_s) {
                let mut s = app.state.lock().unwrap_or_else(|p| p.into_inner());
                s.set_status("governor params ignored: require 0 < d_stop < d_safe and speed > 0");
                return;
            }
            let mut s = app.state.lock().unwrap_or_else(|p| p.into_inner());
            s.d_stop = d_stop;
            s.d_safe = d_safe;
            s.max_ee_velocity_m_s = max_ee_velocity_m_s;
            s.set_status(format!(
                "governor: d_stop={d_stop} d_safe={d_safe} max_ee={max_ee_velocity_m_s} m/s"
            ));
        }
    }
}

// A governor band the UI may stream: all finite and positive, with d_stop below
// d_safe. The hub validates again before applying.
fn valid_governor_band(d_stop: f64, d_safe: f64, max_ee_velocity_m_s: f64) -> bool {
    [d_stop, d_safe, max_ee_velocity_m_s]
        .iter()
        .all(|v| v.is_finite() && *v > 0.0)
        && d_stop < d_safe
}

// Clamp each joint setpoint into its configured [min, max]. The single clamp
// path for every operator-driven arm command; the arm clamps again on its side.
fn clamp_to_limits(joints: &mut [f64; ARM_DOF], side: Side) {
    for (j, &[lo, hi]) in joints.iter_mut().zip(joint_limits().arm(side).iter()) {
        *j = j.clamp(lo, hi);
    }
}

async fn fire_arm(app: &AppState, side: Side, joints: [f64; ARM_DOF], duration_s: f64) {
    // Preempt: a Send while a goal is in flight cancels the old one (the arm's
    // single-flight gate would otherwise reject the new goal) and waits for it
    // to finalize before firing. The cancelled goal's feedback loop exits
    // promptly, so in_flight clears within the cancel round-trip.
    let preempt = {
        let s = app.state.lock().unwrap_or_else(|p| p.into_inner());
        if s.arm(side).in_flight {
            s.arm(side).preempt.clone()
        } else {
            None
        }
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
            s.set_status(format!(
                "{} arm: previous goal still finishing",
                side.label()
            ));
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

// --------------------------- wire protocol ---------------------------

#[derive(Serialize)]
struct Snapshot {
    left_arm: ArmView,
    right_arm: ArmView,
    left_gripper: GripperView,
    right_gripper: GripperView,
    // Streaming deadman per side, shared by that side's arm and gripper.
    left_enabled: bool,
    right_enabled: bool,
    // Operator's self-collision governor controls (streamed to the hub).
    collision_enabled: bool,
    d_stop: f64,
    d_safe: f64,
    max_ee_velocity_m_s: f64,
    // Live nearest-pair proximity from the hub (null until the first report).
    proximity: Option<ProximityView>,
    status: String,
}

#[derive(Serialize)]
struct ProximityView {
    distance: f64,
    link_a: String,
    link_b: String,
    throttled: bool,
    stopped: bool,
}

#[derive(Serialize)]
struct ArmView {
    joints: [f64; ARM_DOF],
    feedback: Option<[f64; ARM_DOF]>,
    in_flight: bool,
    // Per-joint [min, max] (rad); the browser bounds its sliders with these.
    limits: [[f64; 2]; ARM_DOF],
}

#[derive(Serialize)]
struct GripperView {
    position: f64,
    // Measured opening (m) from the gripper_states stream.
    feedback: Option<f64>,
    min: f64,
    max: f64,
}

impl Snapshot {
    fn build(s: &UiState, now: Instant) -> Self {
        Self {
            left_arm: arm_view(&s.left_arm, Side::Left),
            right_arm: arm_view(&s.right_arm, Side::Right),
            left_gripper: gripper_view(&s.left_gripper),
            right_gripper: gripper_view(&s.right_gripper),
            left_enabled: s.left_enabled,
            right_enabled: s.right_enabled,
            collision_enabled: s.collision_enabled,
            d_stop: s.d_stop,
            d_safe: s.d_safe,
            max_ee_velocity_m_s: s.max_ee_velocity_m_s,
            proximity: live_proximity(s, now).map(|p| ProximityView {
                distance: p.distance,
                link_a: p.link_a.clone(),
                link_b: p.link_b.clone(),
                throttled: p.throttled,
                stopped: p.stopped,
            }),
            status: s.status.clone(),
        }
    }
}

/// The proximity readout if it is still fresh, else `None` (the hub stopped
/// reporting), so the UI falls back to n/a instead of latching a stale distance.
fn live_proximity(s: &UiState, now: Instant) -> Option<&Proximity> {
    s.proximity
        .as_ref()
        .filter(|p| now.duration_since(p.received_at) < PROXIMITY_STALE_AFTER)
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
        feedback: g.last_feedback,
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
        // Requested move duration (s); 0 = fastest safe.
        duration_s: f64,
    },
    // Toggle the streaming deadman for one side. While enabled, command_stream
    // emits that side's arm target on arm_joint_commands and gripper opening on
    // gripper_commands; while disabled both track the measured pose and emit
    // nothing.
    SetEnabled {
        side: SideWire,
        on: bool,
    },
    // Update an enabled arm's streamed target. Ignored while disabled, where the
    // target follows the measured pose so enabling never steps the arm.
    SetArmTarget {
        side: SideWire,
        joints: [f64; ARM_DOF],
    },
    // Update an enabled gripper's streamed opening. Ignored while disabled.
    SetGripperTarget {
        side: SideWire,
        position: f64,
    },
    // Set the hub's self-collision-avoidance toggle (streamed continuously).
    SetCollision {
        enabled: bool,
    },
    // Retune the hub's governor band and stream speed cap (streamed continuously).
    SetGovernorParams {
        d_stop: f64,
        d_safe: f64,
        max_ee_velocity_m_s: f64,
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

#[cfg(test)]
mod tests {
    use super::*;

    /// Tests have no main() to run init_limits, so resolve the v2 limits on
    /// first use; concurrent tests settle benignly through get_or_init.
    fn init_limits_for_tests() {
        LIMITS.get_or_init(|| JointLimits::resolve(HardwareVersion::V2));
    }

    #[test]
    fn clamp_pins_each_joint_into_its_range() {
        init_limits_for_tests();
        for side in [Side::Left, Side::Right] {
            let limits = joint_limits().arm(side);

            let mut high = [f64::INFINITY; ARM_DOF];
            clamp_to_limits(&mut high, side);
            for (v, &[_, hi]) in high.iter().zip(limits.iter()) {
                assert_eq!(*v, hi);
            }

            let mut low = [f64::NEG_INFINITY; ARM_DOF];
            clamp_to_limits(&mut low, side);
            for (v, &[lo, _]) in low.iter().zip(limits.iter()) {
                assert_eq!(*v, lo);
            }
        }
    }

    #[test]
    fn clamp_leaves_in_range_values_untouched() {
        init_limits_for_tests();
        for side in [Side::Left, Side::Right] {
            let limits = joint_limits().arm(side);
            let mut mid = [0.0; ARM_DOF];
            for (m, &[lo, hi]) in mid.iter_mut().zip(limits.iter()) {
                *m = (lo + hi) / 2.0;
            }
            let before = mid;
            clamp_to_limits(&mut mid, side);
            assert_eq!(mid, before);
        }
    }

    #[test]
    fn disconnect_disarms_sides_and_restores_governor_default_on() {
        // Launched with avoidance on; operator turned it off with both sides armed.
        let mut s = UiState::new(true, 0.005, 0.02, 0.25);
        s.collision_enabled = false;
        s.set_enabled(Side::Left, true);
        s.set_enabled(Side::Right, true);
        on_operator_disconnect(&mut s);
        assert!(
            !s.left_enabled && !s.right_enabled,
            "disconnect must drop the deadman for both sides"
        );
        assert!(
            s.collision_enabled,
            "disconnect must restore the launch governor default (on)"
        );
    }

    #[test]
    fn disconnect_restores_governor_default_off_when_launched_ungoverned() {
        // Launched deliberately ungoverned; operator turned avoidance on.
        let mut s = UiState::new(false, 0.005, 0.02, 0.25);
        s.collision_enabled = true;
        on_operator_disconnect(&mut s);
        assert!(
            !s.collision_enabled,
            "disconnect must restore the launch default (off), not force on"
        );
    }

    #[test]
    fn valid_governor_band_boundaries() {
        assert!(valid_governor_band(0.005, 0.02, 1.0));
        assert!(
            !valid_governor_band(0.02, 0.02, 1.0),
            "d_stop == d_safe is degenerate"
        );
        assert!(
            !valid_governor_band(0.03, 0.02, 1.0),
            "d_stop > d_safe is inverted"
        );
        assert!(!valid_governor_band(0.0, 0.02, 1.0), "non-positive d_stop");
        assert!(
            !valid_governor_band(0.005, 0.02, 0.0),
            "non-positive speed cap"
        );
        assert!(
            !valid_governor_band(f64::NAN, 0.02, 1.0),
            "non-finite d_stop"
        );
    }

    #[test]
    fn config_joint_limits_are_well_formed() {
        init_limits_for_tests();
        // Each range must be non-empty so clamp and the slider bounds are valid.
        for side in [Side::Left, Side::Right] {
            for &[lo, hi] in joint_limits().arm(side).iter() {
                assert!(lo < hi, "joint range [{lo}, {hi}] must be non-empty");
            }
        }
        let [lo, hi] = joint_limits().gripper;
        assert!(lo < hi, "gripper range [{lo}, {hi}] must be non-empty");
    }
}
