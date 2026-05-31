# openarm01_joint_commander

Rust TUI that drives the bimanual openarm01 in joint space. Reads stepping
input from the keyboard, fires `move_arm_joints` and `move_gripper` actions at
`openarm01_backbone`, and shows in-flight feedback in a ratatui interface.

MVP scope: joint-space only. Cartesian operator input (and the upstream IK +
collision-detection pipeline) is post-MVP; the node goes through `backbone`'s
`move_arm_joints` action which arm executes directly. A future pose-estimation
variant will land as a separate sibling node, not as a variant of this one.

---

## Dependency

`openarm01_backbone:v1` must be in the stack (backbone forwards goals to
arm / gripper instances). Backbone in turn depends on `openarm01_robot_initializer:v1`,
`openarm01_arm:v1`, `openarm01_gripper:v1` via interface conformance — the
launcher binds concrete impls (real / mujoco / isaac) at startup.

---

## Build

```bash
peppy node sync .              # regenerate peppygen
peppy node add . -sb           # add + sync + build
```

Apptainer pulls `tuatini/peppy-rust-cargo-base` and runs `cargo build --release`.

---

## Run

```bash
peppy node run openarm01_joint_commander:v1
```

Single instance. The TUI takes over the terminal; press `q` to quit cleanly.

---

## Keymap

| Key | Effect |
|---|---|
| `[` / `]` | Focus left / right arm |
| `{` / `}` | Focus left / right gripper |
| `1`..`7` | Select joint of the focused arm |
| `↑` / `↓` | Step the selected joint by the current step size |
| `+` / `-` | Halve / double the step size (clamped 0.01 - 0.5 rad) |
| `Enter` | Fire `move_arm_joints` for the focused arm |
| `h` | Reset focused arm target to home `[0, 0, 0, 0, 0, 0, 0]` |
| `o` | Open the focused gripper (0.044 m) |
| `c` | Close the focused gripper (0.0 m) |
| `q` / `Esc` | Quit |

A previous goal must finish before the same arm / gripper accepts a new one —
the status line shows `previous goal still in flight` otherwise.

---

## Architecture

- `main.rs`: thin entrypoint; `NodeBuilder::new().run(...)` then awaits `ui::run`.
- `ui.rs`: ratatui render loop + `crossterm::EventStream` input loop. Holds the
  terminal alive via a `Drop` guard so any exit path restores the TTY.
- `state.rs`: `Arc<tokio::sync::Mutex<UiState>>` shared with action tasks. The
  `in_flight` flag per arm / gripper is the single-writer gate that prevents
  double-firing.
- `actions/move_arm_joints.rs` + `actions/move_gripper.rs`: each `Enter` /
  `o` / `c` spawns one task that fires the consumed action at backbone,
  streams feedback into the shared state, then writes the result into the
  status line and clears `in_flight`. Cancel-aware: a global token cancel
  abandons the feedback wait and finalises with a status message.
