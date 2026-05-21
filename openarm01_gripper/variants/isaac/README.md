# openarm01_gripper — isaac variant

Rust peppy node that drives the gripper in an Isaac Sim by subscribing to raw
peppylib telemetry published by `openarm01_robot_initializer:isaac`'s
in-process bridge extension, and publishing raw `set_ctrl_gripper_<side>`
back to it for actuator control.

Same source as the mujoco variant — the gripper node is engine-agnostic over
peppylib. Only the dependency (`robot_initializer:isaac` instead of `:mujoco`)
differs.

The container talks only over the peppy daemon — no shared filesystem with
`robot_initializer`, no host-bind dependencies. Works cross-host out of the box.

---

## Dependency

This variant needs `openarm01_robot_initializer:0.1.0` running in its
`isaac` variant. That node owns the Isaac Sim process, applies ctrl writes
via `ArticulationView.set_joint_position_targets()`, and emits the raw
telemetry topics this gripper consumes.

If `robot_initializer:isaac` is not running, the gripper starts but does
not publish telemetry until the raw topics begin flowing.

---

## Build

```bash
# From this directory (variants/isaac/)
peppy node sync ../..
peppy node add . --variant isaac
peppy node build openarm01_gripper:0.1.0
```

The apptainer build pulls `tuatini/peppy-rust-cargo-base:latest` and runs
`cargo build --release` inside the container.

---

## Run

```bash
peppy node run openarm01_robot_initializer:0.1.0
peppy node run openarm01_gripper:0.1.0 gripper_id=0 -i left_gripper
peppy node run openarm01_gripper:0.1.0 gripper_id=1 -i right_gripper
peppy stack list
```

---

## Testing

Same standalone Rust harness as the mujoco variant:

```bash
cargo build --release --features test-tools --bin test_move_gripper
./target/release/test_move_gripper --side left --position 0.022 --feedback-hz 20
```

---

## Architecture notes

- **No engine-specific code.** `src/main.rs`, action handler, telemetry pipelines
  are identical to the mujoco variant. Engine differences live entirely in
  `robot_initializer:<variant>`'s bridge extension.
- **`position` semantics**: total aperture per V10 spec (0.0 closed, 0.044 fully
  open). Each finger is driven to `position / 2`.
- **First-run note**: USD prim paths in `robot_initializer:isaac`'s
  `config/sim_bridge.json5` may need verification against the actual stage
  layout — defaults are reasonable guesses but the scene wiring is asset-driven.
