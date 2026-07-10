// HTTP+WS UI on 0.0.0.0:PEPPY_JC_PORT (default 8765). The WS exposes
// unauthenticated motion control, so only run on a trusted network; set
// PEPPY_JC_BIND_IP=127.0.0.1 to restrict to loopback.
//
// This is only the transport: every text frame is decoded to a [`Command`] and sent to
// the state owner, and every snapshot the owner publishes is forwarded to the browser.
// The owner (see [`crate::owner`]) is the sole reader/writer of `UiState`.

use std::env;
use std::net::{IpAddr, Ipv4Addr, SocketAddr};
use std::time::{Duration, Instant};

use axum::Router;
use axum::extract::State;
use axum::extract::ws::{Message, Utf8Bytes, WebSocket, WebSocketUpgrade};
use axum::response::{Html, IntoResponse};
use axum::routing::get;
use openarm_description::HardwareVersion;
use peppylib::runtime::CancellationToken;
use serde::{Deserialize, Serialize};
use tokio::net::TcpListener;
use tokio::sync::{mpsc, watch};
use tracing::{info, warn};

use crate::error::Result;
use crate::owner::UiMsg;
use crate::pose::{ArmModels, JogMode, Pose};
use crate::state::{ARM_DOF, ArmTarget, Disposition, GripperTarget, Proximity, Side, UiState};

const DEFAULT_PORT: u16 = 8765;
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

/// The gripper opening range `[closed, open]` (m); the owner clamps gripper commands
/// into it, the same single source the sliders bound against.
pub(crate) fn gripper_limits() -> [f64; 2] {
    joint_limits().gripper
}

#[derive(Clone)]
struct AppState {
    // Operator input to the owner: decoded commands and the disconnect signal.
    command_tx: mpsc::Sender<UiMsg>,
    // The owner's latest pre-serialized snapshot; forwarded verbatim to the browser.
    snapshot_rx: watch::Receiver<String>,
    token: CancellationToken,
}

pub async fn run(
    command_tx: mpsc::Sender<UiMsg>,
    snapshot_rx: watch::Receiver<String>,
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
        command_tx,
        snapshot_rx,
        token: token.clone(),
    };
    let app = Router::new()
        .route("/", get(index))
        .route("/ws", get(ws_upgrade))
        .with_state(app_state);

    let listener = TcpListener::bind(addr).await?;
    info!("commander UI at http://localhost:{port}");

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
    let mut snapshots = app.snapshot_rx.clone();
    // Send the latest snapshot immediately so a fresh connection paints at once, then
    // follow the owner's updates. `borrow_and_update` marks it seen, so `changed` next
    // waits for the following snapshot.
    let initial = snapshots.borrow_and_update().clone();
    if !initial.is_empty()
        && socket
            .send(Message::Text(Utf8Bytes::from(initial)))
            .await
            .is_err()
    {
        return;
    }
    loop {
        tokio::select! {
            _ = app.token.cancelled() => break,
            changed = snapshots.changed() => {
                if changed.is_err() {
                    break; // the owner is gone
                }
                let json = snapshots.borrow_and_update().clone();
                if !json.is_empty()
                    && socket.send(Message::Text(Utf8Bytes::from(json))).await.is_err()
                {
                    break;
                }
            }
            msg = socket.recv() => match msg {
                Some(Ok(Message::Text(text))) => match serde_json::from_str::<Command>(text.as_str()) {
                    Ok(cmd) => { let _ = app.command_tx.send(UiMsg::Command(cmd)).await; }
                    Err(e) => warn!(error = %e, payload = %text, "ws: bad command"),
                },
                Some(Ok(Message::Close(_))) | None => break,
                Some(Err(e)) => { warn!(error = %e, "ws: recv"); break; }
                _ => {}
            }
        }
    }
    // Releasing the panel drops the deadman and restores the governor default.
    let _ = app.command_tx.send(UiMsg::Disconnect).await;
}

// A governor band the UI may stream: all finite and positive, with d_stop below
// d_safe. The hub validates again before applying.
pub(crate) fn valid_governor_band(d_stop: f64, d_safe: f64, max_ee_velocity_m_s: f64) -> bool {
    [d_stop, d_safe, max_ee_velocity_m_s]
        .iter()
        .all(|v| v.is_finite() && *v > 0.0)
        && d_stop < d_safe
}

// Clamp each joint setpoint into its configured [min, max]. The single clamp
// path for every operator-driven arm command; the arm clamps again on its side.
pub(crate) fn clamp_to_limits(joints: &mut [f64; ARM_DOF], side: Side) {
    for (j, &[lo, hi]) in joints.iter_mut().zip(joint_limits().arm(side).iter()) {
        *j = j.clamp(lo, hi);
    }
}

// Clamp a requested move duration to a sane range (finite, 0..=30 s). 0 = fastest.
pub(crate) fn sane_duration(duration_s: f64) -> f64 {
    if duration_s.is_finite() {
        duration_s.clamp(0.0, 30.0)
    } else {
        0.0
    }
}

// A discrete move's duration: the operator's request, floored so the straight-line
// EE speed never exceeds the governor cap (time >= distance / cap). The hub floors
// again at its joint-velocity limit, so this only ever slows a move, never speeds it.
pub(crate) fn ee_speed_floored(user_s: f64, ee_distance_m: f64, max_ee_velocity_m_s: f64) -> f64 {
    (ee_distance_m / max_ee_velocity_m_s).max(user_s)
}

/// Serialize the browser snapshot from the owner's state; called on the owner's
/// snapshot tick, so it holds no lock and each connection forwards the same bytes.
pub(crate) fn build_snapshot_json(
    s: &UiState,
    now: Instant,
    models: &ArmModels,
) -> serde_json::Result<String> {
    serde_json::to_string(&Snapshot::build(s, now, models))
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
    // World-frame x/y/z reachable bounds [[min, max]; 3]; the browser bounds its
    // position sliders with these (per generation, from the arm's FK envelope).
    pos_bounds: [[f64; 2]; 3],
    // World-frame end-effector pose [x, y, z, roll, pitch, yaw] of the joint target
    // (FK), so moving a joint updates the panel's pose fields.
    pose: Pose,
    // Same for the measured joints, `null` until the first state arrives; the panel
    // shows it beside the target pose the way it does per-joint feedback.
    pose_feedback: Option<Pose>,
    // World-frame end-effector orientation as a quaternion [x, y, z, w] for the target
    // (FK of the joint target) and the measured pose. The arcball composes on these,
    // so orientation never round-trips through euler on the wire.
    orientation: [f64; 4],
    orientation_feedback: Option<[f64; 4]>,
    // Arm angle psi (elbow swivel, rad) of the target and the measured pose; `null` at
    // the straight-arm singularity (kept off by the elbow floor). Drives the elbow slider.
    arm_angle: Option<f64>,
    arm_angle_feedback: Option<f64>,
}

#[derive(Serialize)]
struct GripperView {
    position: f64,
    // Measured opening (m) from the gripper_states stream.
    feedback: Option<f64>,
    min: f64,
    max: f64,
    // A discrete move_gripper is in flight (drives the gripper card's badge).
    in_flight: bool,
}

impl Snapshot {
    fn build(s: &UiState, now: Instant, models: &ArmModels) -> Self {
        Self {
            left_arm: arm_view(&s.arms[Side::Left], Side::Left, models),
            right_arm: arm_view(&s.arms[Side::Right], Side::Right, models),
            left_gripper: gripper_view(&s.grippers[Side::Left]),
            right_gripper: gripper_view(&s.grippers[Side::Right]),
            left_enabled: s.enabled[Side::Left],
            right_enabled: s.enabled[Side::Right],
            collision_enabled: s.collision_enabled,
            d_stop: s.d_stop,
            d_safe: s.d_safe,
            max_ee_velocity_m_s: s.max_ee_velocity_m_s,
            proximity: live_proximity(s, now).map(|p| ProximityView {
                distance: p.distance,
                link_a: p.link_a.clone(),
                link_b: p.link_b.clone(),
                throttled: p.disposition == Disposition::Throttled,
                stopped: p.disposition == Disposition::Stopped,
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

fn arm_view(a: &ArmTarget, side: Side, models: &ArmModels) -> ArmView {
    ArmView {
        joints: a.joints,
        feedback: a.last_feedback,
        in_flight: a.in_flight,
        limits: *joint_limits().arm(side),
        pos_bounds: models.pos_bounds(side),
        pose: models.ee_pose_world(side, &a.joints),
        pose_feedback: a.last_feedback.map(|fb| models.ee_pose_world(side, &fb)),
        orientation: models.ee_quat_world(side, &a.joints),
        orientation_feedback: a.last_feedback.map(|fb| models.ee_quat_world(side, &fb)),
        arm_angle: models.arm_angle(side, &a.joints),
        arm_angle_feedback: a.last_feedback.and_then(|fb| models.arm_angle(side, &fb)),
    }
}

fn gripper_view(g: &GripperTarget) -> GripperView {
    let [min, max] = joint_limits().gripper;
    GripperView {
        position: g.position,
        feedback: g.last_feedback,
        min,
        max,
        in_flight: g.in_flight,
    }
}

#[derive(Deserialize)]
#[serde(tag = "cmd", rename_all = "snake_case")]
pub(crate) enum Command {
    FireArm {
        side: SideWire,
        joints: [f64; ARM_DOF],
        // Requested move duration (s); 0 = fastest safe.
        duration_s: f64,
    },
    // Toggle the streaming deadman for one side. While enabled, the command stream
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
    // Arm a jog: position (metres) + orientation quaternion [x, y, z, w] + arm angle
    // (elbow swivel, rad), plus which one the jog drives (`mode` = the touched control;
    // the rest is held). The command stream walks the joint target toward it and holds
    // at the boundary. Ignored while disabled, like set_arm_target.
    SetArmPose {
        side: SideWire,
        position: [f64; 3],
        orientation: [f64; 4],
        arm_angle: f64,
        mode: JogModeWire,
    },
    // Fire the hub's planned Cartesian move_arm to a composed world-frame pose
    // (Actions-mode Execute): a governed straight-line move, not a jog. Refused
    // while the side streams, like fire_arm.
    FireArmPose {
        side: SideWire,
        position: [f64; 3],
        orientation: [f64; 4],
        // Requested move duration (s); 0 = fastest safe.
        duration_s: f64,
    },
    // Update an enabled gripper's streamed opening. Ignored while disabled.
    SetGripperTarget {
        side: SideWire,
        position: f64,
    },
    // Fire the hub's discrete move_gripper (Actions-mode gripper Execute): a governed
    // open/close to `position` (m), not the streamed opening. Refused while the side
    // streams or a prior gripper move is in flight.
    FireGripper {
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
pub(crate) enum SideWire {
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

// Which component a jog drives, as sent by the panel: "position" from the x/y/z
// sliders, "orientation" from the arcball, or "arm_angle" from the elbow slider.
#[derive(Deserialize, Copy, Clone)]
#[serde(rename_all = "lowercase")]
pub(crate) enum JogModeWire {
    Position,
    Orientation,
    #[serde(rename = "arm_angle")]
    ArmAngle,
}

impl From<JogModeWire> for JogMode {
    fn from(m: JogModeWire) -> Self {
        match m {
            JogModeWire::Position => JogMode::Position,
            JogModeWire::Orientation => JogMode::Orientation,
            JogModeWire::ArmAngle => JogMode::ArmAngle,
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
