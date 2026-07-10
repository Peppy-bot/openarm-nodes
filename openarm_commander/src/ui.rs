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
use srs_model::nalgebra::{Quaternion, UnitQuaternion};
use tokio::net::TcpListener;
use tracing::{info, warn};

use crate::error::Result;
use crate::pose::{ArmModels, CartesianJog, Jog, JogMode, Pose, dist3};
use crate::state::{
    ARM_DOF, ArmTarget, Disposition, GripperTarget, Proximity, SharedState, Side, UiState,
};
use crate::{move_arm, move_arm_joints, move_gripper};

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
    // Per-side arm kinematics for the panel's Cartesian pose fields.
    models: ArmModels,
}

pub async fn run(
    runner: Arc<NodeRunner>,
    state: SharedState,
    token: CancellationToken,
    models: ArmModels,
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
        models,
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
                    Snapshot::build(&s, Instant::now(), &app.models)
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
        s.enabled[side] = false;
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
                if s.enabled[side] {
                    s.set_status(format!(
                        "{} arm: disable before a discrete move",
                        side.label()
                    ));
                    return;
                }
            }
            clamp_to_limits(&mut joints, side);
            let (measured, max_ee) = {
                let s = app.state.lock().unwrap_or_else(|p| p.into_inner());
                (s.arms[side].last_feedback, s.max_ee_velocity_m_s)
            };
            // Floor the requested duration so the end-effector never crosses the
            // workspace faster than the governor cap; the hub floors again at its
            // joint-velocity limit.
            let duration_s = match measured {
                Some(m) => {
                    let from = app.models.ee_pose_world(side, &m);
                    let to = app.models.ee_pose_world(side, &joints);
                    let dist = dist3([from[0], from[1], from[2]], [to[0], to[1], to[2]]);
                    ee_speed_floored(sane_duration(duration_s), dist, max_ee)
                }
                None => sane_duration(duration_s),
            };
            fire_arm(app, side, joints, duration_s);
        }
        Command::SetEnabled { side, on } => {
            let side: Side = side.into();
            let mut s = app.state.lock().unwrap_or_else(|p| p.into_inner());
            if on {
                // A discrete move owns the arm until its result lands; enabling now
                // would fight it and the hub would snap back when it completes.
                if s.arms[side].in_flight {
                    s.set_status(format!("{}: move in flight, not enabling", side.label()));
                    return;
                }
                // Enabling streams both the arm and the gripper for this side, so
                // seed each target first; refuse until measurements exist so the
                // first emitted command holds position instead of a stale default.
                let (Some(_), Some(gripper_measured)) =
                    (s.arms[side].last_feedback, s.grippers[side].last_feedback)
                else {
                    s.set_status(format!(
                        "{}: no measured pose yet, not enabling",
                        side.label()
                    ));
                    return;
                };
                // Keep the retained arm target: it is the setpoint the arm already
                // tracks under the same PD whether enabled or disabled, so commanding
                // it on enable holds position with no jump, and repeated enable/disable
                // never re-seeds the sagged measured (which ratcheted the arm down).
                // joint_states establishes the target from the first measured pose at
                // boot. The gripper does not sag, so seeding it from measured is safe.
                s.grippers[side].position = gripper_measured;
            }
            // A jog must not survive across a deadman edge in either direction: on
            // enable it would replay a stale desired pose, on disable it would resume
            // unexpectedly at the next enable. Its status latch resets with it.
            s.arms[side].jog = None;
            s.arms[side].jog_blocked = false;
            s.enabled[side] = on;
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
            if s.enabled[side] {
                // Arm a joint jog: the stream reconciles it to the setpoint on the next
                // tick (in one step, so the slider never lags the drag). This replaces
                // any live Cartesian jog, which would otherwise walk the target right
                // back off the operator's slider; its status latch resets with it.
                s.arms[side].jog = Some(Jog::Joints(joints));
                s.arms[side].jog_blocked = false;
            }
        }
        Command::SetArmPose {
            side,
            position,
            orientation,
            arm_angle,
            mode,
        } => {
            let side: Side = side.into();
            if !position
                .iter()
                .chain(orientation.iter())
                .chain(std::iter::once(&arm_angle))
                .all(|v| v.is_finite())
            {
                return;
            }
            // The wire carries orientation as a quaternion; store the desired pose as
            // euler for the jog, which re-derives a quaternion each step (so the euler
            // encoding never drives interpolation). arm_angle is the null-space (elbow)
            // target, used only in ArmAngle mode.
            let (roll, pitch, yaw) = quat_to_euler(orientation);
            let pose: Pose = [position[0], position[1], position[2], roll, pitch, yaw];
            // Arm the Cartesian jog: the command stream walks the joint target toward
            // this pose a capped step per tick (holding at the reach boundary), so a
            // slider drag can never command a teleport or a branch flip. Enabled-gated
            // like set_arm_target.
            let mut s = app.state.lock().unwrap_or_else(|p| p.into_inner());
            if s.enabled[side] {
                s.arms[side].jog = Some(Jog::Cartesian(CartesianJog {
                    mode: mode.into(),
                    desired: pose,
                    arm_angle,
                }));
            }
        }
        Command::FireArmPose {
            side,
            position,
            orientation,
            duration_s,
        } => {
            let side: Side = side.into();
            let (seed, measured, max_ee) = {
                let mut s = app.state.lock().unwrap_or_else(|p| p.into_inner());
                if s.enabled[side] {
                    s.set_status(format!(
                        "{} arm: disable before a discrete move",
                        side.label()
                    ));
                    return;
                }
                (
                    s.arms[side].joints,
                    s.arms[side].last_feedback,
                    s.max_ee_velocity_m_s,
                )
            };
            if !position
                .iter()
                .chain(orientation.iter())
                .all(|v| v.is_finite())
            {
                return;
            }
            let rotation = UnitQuaternion::from_quaternion(Quaternion::new(
                orientation[3],
                orientation[0],
                orientation[1],
                orientation[2],
            ));
            // Preview the pose as joints (seeded from the current target) so both the
            // joint sliders and the pose FK show where it is going, and reject an
            // unreachable pose up front rather than firing a goal the hub will refuse.
            let Some(mut target_joints) = app.models.solve_ik(side, position, rotation, &seed)
            else {
                let mut s = app.state.lock().unwrap_or_else(|p| p.into_inner());
                s.set_status(format!(
                    "{} arm: pose unreachable, not firing",
                    side.label()
                ));
                return;
            };
            clamp_to_limits(&mut target_joints, side);
            // Floor the duration to the governor's EE-speed cap over the straight-line
            // distance; the hub floors again at its joint-velocity limit.
            let duration_s = match measured {
                Some(m) => {
                    let from = app.models.ee_pose_world(side, &m);
                    let dist = dist3([from[0], from[1], from[2]], position);
                    ee_speed_floored(sane_duration(duration_s), dist, max_ee)
                }
                None => sane_duration(duration_s),
            };
            let q = [rotation.i, rotation.j, rotation.k, rotation.w];
            fire_arm_pose(app, side, position, q, target_joints, duration_s);
        }
        Command::SetGripperTarget { side, position } => {
            let side: Side = side.into();
            let [lo, hi] = joint_limits().gripper;
            let position = position.clamp(lo, hi);
            let mut s = app.state.lock().unwrap_or_else(|p| p.into_inner());
            if s.enabled[side] {
                s.grippers[side].position = position;
            }
        }
        Command::FireGripper { side, position } => {
            let side: Side = side.into();
            if !position.is_finite() {
                return;
            }
            let [lo, hi] = joint_limits().gripper;
            let position = position.clamp(lo, hi);
            {
                let mut s = app.state.lock().unwrap_or_else(|p| p.into_inner());
                // The side must be disabled for a discrete move, and only one gripper
                // goal may be in flight; refuse rather than preempt (moves are short).
                if s.enabled[side] {
                    s.set_status(format!(
                        "{} gripper: disable before a discrete move",
                        side.label()
                    ));
                    return;
                }
                if s.grippers[side].in_flight {
                    s.set_status(format!(
                        "{} gripper: previous move still finishing",
                        side.label()
                    ));
                    return;
                }
                s.grippers[side].in_flight = true;
                s.grippers[side].position = position;
                s.set_status(format!("{} gripper: firing move_gripper", side.label()));
            }
            move_gripper::spawn(
                app.runner.clone(),
                app.state.clone(),
                app.token.clone(),
                side,
                position,
            );
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

// Clamp a requested move duration to a sane range (finite, 0..=30 s). 0 = fastest.
fn sane_duration(duration_s: f64) -> f64 {
    if duration_s.is_finite() {
        duration_s.clamp(0.0, 30.0)
    } else {
        0.0
    }
}

// A discrete move's duration: the operator's request, floored so the straight-line
// EE speed never exceeds the governor cap (time >= distance / cap). The hub floors
// again at its joint-velocity limit, so this only ever slows a move, never speeds it.
fn ee_speed_floored(user_s: f64, ee_distance_m: f64, max_ee_velocity_m_s: f64) -> f64 {
    (ee_distance_m / max_ee_velocity_m_s).max(user_s)
}

// Intrinsic-XYZ roll/pitch/yaw of a `[x, y, z, w]` quaternion, via nalgebra so the
// convention matches the panel's FK readout and the jog's `from_euler_angles`.
fn quat_to_euler(q: [f64; 4]) -> (f64, f64, f64) {
    UnitQuaternion::from_quaternion(Quaternion::new(q[3], q[0], q[1], q[2])).euler_angles()
}

fn fire_arm(app: &AppState, side: Side, joints: [f64; ARM_DOF], duration_s: f64) {
    fire_discrete(
        app,
        side,
        "move_arm_joints",
        Some(joints),
        move |app, preempt| {
            move_arm_joints::spawn(
                app.runner.clone(),
                app.state.clone(),
                app.token.clone(),
                preempt,
                side,
                joints,
                duration_s,
            );
        },
    );
}

fn fire_arm_pose(
    app: &AppState,
    side: Side,
    position: [f64; 3],
    orientation: [f64; 4],
    target_joints: [f64; ARM_DOF],
    duration_s: f64,
) {
    fire_discrete(
        app,
        side,
        "move_arm",
        Some(target_joints),
        move |app, preempt| {
            move_arm::spawn(
                app.runner.clone(),
                app.state.clone(),
                app.token.clone(),
                preempt,
                side,
                position,
                orientation,
                duration_s,
            );
        },
    );
}

/// The shared gate every discrete move fires through. The preempt round-trip can
/// take ~1 s, so the whole flow runs off the WS loop; after the awaits the gate
/// re-checks everything they could have invalidated (an interleaved Enable, a new
/// in-flight goal) before claiming the slot. `target_joints`, when given, becomes
/// the retained joint target so the panel mirrors where the move is going.
fn fire_discrete(
    app: &AppState,
    side: Side,
    action: &'static str,
    target_joints: Option<[f64; ARM_DOF]>,
    launch: impl FnOnce(&AppState, tokio_util::sync::CancellationToken) + Send + 'static,
) {
    let app = app.clone();
    tokio::spawn(async move {
        // Preempt: an Execute while a goal is in flight cancels the old one (the
        // hub's single-flight gate would otherwise reject the new goal) and waits
        // for it to finalize before firing. The cancelled goal's feedback loop
        // exits promptly, so in_flight clears within the cancel round-trip.
        let preempt = {
            let s = app.state.lock().unwrap_or_else(|p| p.into_inner());
            if s.arms[side].in_flight {
                s.arms[side].preempt.clone()
            } else {
                None
            }
        };
        if let Some(tok) = preempt {
            tok.cancel();
            for _ in 0..50 {
                tokio::time::sleep(std::time::Duration::from_millis(20)).await;
                let clear =
                    !app.state.lock().unwrap_or_else(|p| p.into_inner()).arms[side].in_flight;
                if clear {
                    break;
                }
            }
            // Grace for the hub to release its busy gate after the result lands.
            tokio::time::sleep(std::time::Duration::from_millis(50)).await;
        }
        let goal_preempt = tokio_util::sync::CancellationToken::new();
        {
            let mut s = app.state.lock().unwrap_or_else(|p| p.into_inner());
            // Re-check everything the awaits above could have invalidated: an
            // Enable interleaved during the preempt wait means the side is
            // streaming again and a discrete move must not fire under it.
            if s.enabled[side] {
                s.set_status(format!(
                    "{} arm: enabled during preempt, move dropped",
                    side.label()
                ));
                return;
            }
            if s.arms[side].in_flight {
                s.set_status(format!(
                    "{} arm: previous goal still finishing",
                    side.label()
                ));
                return;
            }
            s.arms[side].in_flight = true;
            if let Some(joints) = target_joints {
                s.arms[side].joints = joints;
            }
            s.arms[side].preempt = Some(goal_preempt.clone());
            s.set_status(format!("{} arm: firing {action}", side.label()));
        }
        launch(&app, goal_preempt);
    });
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

// Which component a jog drives, as sent by the panel: "position" from the x/y/z
// sliders, "orientation" from the arcball, or "arm_angle" from the elbow slider.
#[derive(Deserialize, Copy, Clone)]
#[serde(rename_all = "lowercase")]
enum JogModeWire {
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
    fn disconnect_disarms_sides_and_restores_governor_default_on() {
        // Launched with avoidance on; operator turned it off with both sides armed.
        let mut s = UiState::new(true, 0.005, 0.02, 0.25);
        s.collision_enabled = false;
        s.enabled[Side::Left] = true;
        s.enabled[Side::Right] = true;
        on_operator_disconnect(&mut s);
        assert!(
            !s.enabled[Side::Left] && !s.enabled[Side::Right],
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
