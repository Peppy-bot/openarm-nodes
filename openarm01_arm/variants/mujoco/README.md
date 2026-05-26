# openarm01_arm — mujoco variant

Rust peppy node that drives one side of the bimanual arm in a MuJoCo sim by
subscribing to raw peppylib telemetry published by
`openarm01_robot_initializer:mujoco`'s in-process bridge extension, and
publishing raw `set_ctrl_arm_<side>` back to it for actuator control.

The container talks only over the peppy daemon — no shared filesystem with
`robot_initializer`, no mmap, no host-bind dependencies. Same transport as
every other peppy node; works cross-host out of the box.

---

## Dependency

This variant needs `openarm01_robot_initializer:0.1.0` running in its
`mujoco` variant. That node owns the MuJoCo process and emits the raw
telemetry topics (`joint_states_<side>`, `tf_tree`, `imu_<side>`, …) that this
arm subscribes to, and subscribes to `set_ctrl_arm_<side>` to apply ctrl
writes inside its `mj_step` loop.

If `robot_initializer:mujoco` is not running, the arm starts but does not
publish telemetry until the raw topics begin flowing.

---

## Build

```bash
# From this directory (variants/mujoco/)
peppy node sync ../..              # regenerate peppygen
peppy node add . --variant mujoco
peppy node build openarm01_arm:0.1.0
```

The apptainer build pulls `tuatini/peppy-rust-cargo-base:latest` and runs
`cargo build --release` inside the container.

---

## Run

Two instances — left and right — pinned to deterministic IDs so the test
harness and backbone can address them:

```bash
peppy node run openarm01_robot_initializer:0.1.0
peppy node run openarm01_arm:0.1.0 arm_id=0 -i left_arm
peppy node run openarm01_arm:0.1.0 arm_id=1 -i right_arm
peppy stack list
```

---

## Testing

Standalone Rust harness fires `move_arm_joints` at the running instances
over the typed peppygen path. Gated behind the `test-tools` cargo feature
so the production SIF doesn't ship it.

```bash
cargo build --release --features test-tools --bin test_move_arm_joints

# Both arms to neutral pose
./target/release/test_move_arm_joints

# Left arm only, custom 7-DOF pose
./target/release/test_move_arm_joints --side left \
    --positions "0.5,0.3,-0.2,1.0,0.0,0.5,0.0"

# Right arm at faster feedback rate
./target/release/test_move_arm_joints --side right --feedback-hz 50
```

---

## Architecture notes

- **No mmap, no offsets, no shared memory.** All telemetry comes in via raw
  peppylib JSON from the daemon; all control goes out via raw peppylib.
- **Driver-only abstraction.** `src/main.rs` wires `sim_bridge_core::SimBridge`
  pipelines: each raw subscription is paired with one typed peppygen emit.
- **Joint-space semantics.** `move_arm_joints` carries 7 absolute joint angles
  (radians, joint-order j1..j7 of one arm side). The cartesian `move_arm`
  action is a stub that rejects — the IK pass that translates cartesian goals
  into joint goals lives in the backbone in a future MVP module.
- **No engine-specific code in src/.** The Rust source is byte-identical
  between mujoco and isaac variants; only `apptainer.def` (image tag, labels)
  and this README differ.
