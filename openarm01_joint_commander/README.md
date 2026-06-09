# openarm01_joint_commander

Rust node that drives the bimanual openarm01 in joint space from a browser. On
startup it binds an HTTP + WebSocket server (default `:8765`, override with
`PEPPY_JC_PORT`), serves a single-page UI with sliders for every joint and the
gripper, and fires `move_arm_joints` / `move_gripper` actions at
`openarm01_backbone`. Action feedback streams back over the WebSocket.

MVP scope: joint-space only. Cartesian operator input (and the upstream IK +
collision-detection pipeline) is post-MVP; the node goes through `backbone`'s
`move_arm_joints` action which arm executes directly. A future pose-estimation
implementation will land as a separate sibling node.

---

## Dependency

`openarm01_backbone:v1` must be in the stack (backbone forwards goals to
arm / gripper instances). Backbone in turn depends on `openarm01_robot_initializer:v1`,
`openarm01_arm:v1`, `openarm01_gripper:v1` via interface conformance — the
launcher binds concrete impls (real / mujoco / isaac) at startup.

---

## Build

```bash
peppy node add . -sb           # add + sync + build
```

Apptainer pulls `tuatini/peppy-rust-cargo-base:latest` and runs `cargo build --release`.

---

## Run

```bash
peppy node run openarm01_joint_commander:v1
# logs: "joint commander UI at http://localhost:8765"
```

Open the logged URL in any browser. Cancel the node (`peppy stack stop` or
ctrl-C on the runner) to shut down the server cleanly.

---

## UI

Four panels: left arm, right arm, left gripper, right gripper.

- **Arm**: 7 sliders (±π rad) with live target + feedback readouts. `Send` fires
  `move_arm_joints` with the current slider values. `Home` resets sliders to
  zero (does not send).
- **Gripper**: position slider (0–0.044 m). `Open`, `Close`, and `Send`
  shortcuts each fire `move_gripper`.
- A previous goal must finish before the same arm / gripper accepts a new one —
  the status line shows `previous goal still in flight` otherwise.

The WebSocket auto-reconnects every second when the server restarts, so you can
leave the browser tab open across rebuilds.

---

## Architecture

- `main.rs`: thin entrypoint; `NodeBuilder::new().run(...)` spawns `ui::run` in a
  background task so `node_health` registers during NodeBuilder finalisation
  before the daemon's health probe fires.
- `ui.rs`: axum router (`GET /` → embedded `static/index.html`, `GET /ws` →
  WebSocket). The WS loop ticks state snapshots out at 10 Hz and dispatches
  `fire_arm` / `fire_gripper` commands by spawning action tasks.
- `state.rs`: `Arc<std::sync::Mutex<UiState>>` shared with action tasks. The
  `in_flight` flag per arm / gripper is the single-writer gate that prevents
  double-firing.
- `actions/move_arm_joints.rs` + `actions/move_gripper.rs`: each command spawns
  one task that fires the consumed action at backbone, streams feedback into
  the shared state, then writes the result into the status line and clears
  `in_flight`. Cancel-aware: a global token cancel abandons the feedback wait
  and finalises with a status message.
