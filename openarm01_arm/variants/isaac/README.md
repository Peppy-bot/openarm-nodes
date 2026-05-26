# openarm01_arm — isaac variant

Rust peppy node that drives one side of the bimanual arm in an Isaac Sim by
subscribing to raw peppylib telemetry published by
`openarm01_robot_initializer:isaac`'s in-process bridge extension, and
publishing raw `set_ctrl_arm_<side>` back to it for actuator control.

Same source as the mujoco variant — the arm node is engine-agnostic over
peppylib. Only the dependency (`robot_initializer:isaac` instead of `:mujoco`)
differs.

The container talks only over the peppy daemon — no shared filesystem with
`robot_initializer`, no host-bind dependencies. Works cross-host out of the box.

---

## Dependency

This variant needs `openarm01_robot_initializer:0.1.0` running in its
`isaac` variant. That node owns the Isaac Sim process, applies ctrl writes
via `ArticulationView.set_joint_position_targets()`, and emits the raw
telemetry topics this arm consumes.

If `robot_initializer:isaac` is not running, the arm starts but does not
publish telemetry until the raw topics begin flowing.

---

## Build

```bash
# From this directory (variants/isaac/)
peppy node sync ../..
peppy node add . --variant isaac
peppy node build openarm01_arm:0.1.0
```

The apptainer build pulls `tuatini/peppy-rust-cargo-base:latest` and runs
`cargo build --release` inside the container.

---

## Run

```bash
peppy node run openarm01_robot_initializer:0.1.0
peppy node run openarm01_arm:0.1.0 arm_id=0 -i left_arm
peppy node run openarm01_arm:0.1.0 arm_id=1 -i right_arm
peppy stack list
```

---

## Testing

Same standalone Rust harness as the mujoco variant:

```bash
cargo build --release --features test-tools --bin test_move_arm_joints

./target/release/test_move_arm_joints --side both \
    --positions "0.5,0.3,-0.2,1.0,0.0,0.5,0.0" --feedback-hz 20
```

---

## Architecture notes

- **No engine-specific code.** `src/main.rs`, action handlers, telemetry pipelines
  are identical to the mujoco variant. Engine differences live entirely in
  `robot_initializer:<variant>`'s bridge extension.
- **`joint_positions` semantics**: 7-DOF, joint-order j1..j7 of one arm side,
  radians, hard-limited to `[-pi, pi]`. The cartesian `move_arm` action is a
  stub that rejects until backbone IK lands.
- **First-run note**: USD prim paths in `robot_initializer:isaac`'s
  `config/sim_bridge.json5` may need verification against the actual stage
  layout — particularly `/World/openarm/left_arm` / `/World/openarm/right_arm`
  for the `set_ctrl_arm_<side>` subscribers. Defaults match the openarm
  bimanual USD as of 2026-05-25.
