# openarm01_gripper_isaac

Rust peppy node that drives the gripper in Isaac Sim. Subscribes to raw peppylib
telemetry published by `openarm01_robot_initializer_isaac`'s in-process bridge
extension, and publishes raw `set_ctrl_gripper_<side>` back to it for actuator
control.

Same source as the mujoco sibling — the gripper node is engine-agnostic over
peppylib. Only the dependency (`openarm01_robot_initializer_isaac` instead of
`_mujoco`) differs.

The container talks only over the peppy daemon — no shared filesystem with
`robot_initializer`, no host-bind dependencies. Works cross-host out of the box.

---

## Dependency

This node needs `openarm01_robot_initializer_isaac:v1` running. That node owns
the Isaac Sim process, applies ctrl writes via
`ArticulationView.set_joint_position_targets()`, and emits the raw telemetry
topics this gripper consumes.

If `openarm01_robot_initializer_isaac` is not running, the gripper starts but
does not publish telemetry until the raw topics begin flowing.

---

## Build

```bash
peppy node add openarm01_gripper_isaac -sb
```

The apptainer build pulls `tuatini/peppy-rust-cargo-base:latest` and runs
`cargo build --release` inside the container.

---

## Run

```bash
peppy node run openarm01_robot_initializer_isaac:v1
peppy node run openarm01_gripper_isaac:v1 gripper_id=0 -i left_gripper
peppy node run openarm01_gripper_isaac:v1 gripper_id=1 -i right_gripper
peppy stack list
```

---

## Testing

Same standalone Rust harness as the mujoco sibling:

```bash
cargo build --release --features test-tools --bin test_move_gripper
./target/release/test_move_gripper --side left --position 0.022 --feedback-hz 20
```

---

## Architecture notes

- **No engine-specific code.** `src/main.rs`, action handler, telemetry pipelines
  are identical to the mujoco sibling. Engine differences live entirely in
  `openarm01_robot_initializer_<engine>`'s bridge extension.
- **`position` semantics**: total aperture per V10 spec (0.0 closed, 0.044 fully
  open). Each finger is driven to `position / 2`.
- **First-run note**: USD prim paths in
  `openarm01_robot_initializer_isaac`'s `config/sim_bridge.json5` may need
  verification against the actual stage layout — defaults are reasonable
  guesses but the scene wiring is asset-driven.
