# openarm01_gripper — mujoco variant

Rust peppy node that drives the gripper in a MuJoCo sim by subscribing to raw
peppylib telemetry published by `openarm01_robot_initializer:mujoco`'s
in-process bridge extension, and publishing raw `set_ctrl_gripper_<side>`
back to it for actuator control.

The container talks only over the peppy daemon — no shared filesystem with
`robot_initializer`, no mmap, no host-bind dependencies. Same transport as
every other peppy node; works cross-host out of the box.

---

## Dependency

This variant needs `openarm01_robot_initializer:0.1.0` running in its
`mujoco` variant. That node owns the MuJoCo process and emits the raw
telemetry topics (`gripper_state_<side>`, `ee_pose_<side>`, `contact_forces`,
…) that this gripper subscribes to, and subscribes to `set_ctrl_gripper_<side>`
to apply ctrl writes inside its `mj_step` loop.

If `robot_initializer:mujoco` is not running, the gripper starts but does not
publish telemetry until the raw topics begin flowing.

---

## Build

```bash
# From this directory (variants/mujoco/)
peppy node sync ../..              # regenerate peppygen
peppy node add . --variant mujoco
peppy node build openarm01_gripper:0.1.0
```

The apptainer build pulls `tuatini/peppy-rust-cargo-base:latest` and runs
`cargo build --release` inside the container.

---

## Run

Two instances — left and right — pinned to deterministic IDs so the test
harness can address them:

```bash
peppy node run openarm01_robot_initializer:0.1.0
peppy node run openarm01_gripper:0.1.0 gripper_id=0 -i left_gripper
peppy node run openarm01_gripper:0.1.0 gripper_id=1 -i right_gripper
peppy stack list
```

---

## Testing

Same standalone Rust harness as before — fires `move_gripper` at the running
instances over the typed peppygen path. Gated behind the `test-tools` cargo
feature so the production SIF doesn't ship it.

```bash
cargo build --release --features test-tools --bin test_move_gripper

./target/release/test_move_gripper                              # both grippers, fully open
./target/release/test_move_gripper --side left --position 0.0   # close left only
./target/release/test_move_gripper --side right --position 0.022 --feedback-hz 20
```

---

## Architecture notes

- **No mmap, no offsets, no shared memory.** All telemetry comes in via raw
  peppylib JSON from the daemon; all control goes out via raw peppylib.
- **Driver-only abstraction.** `src/main.rs` wires `sim_bridge_core::SimBridge`
  pipelines: each raw subscription is paired with one typed peppygen emit.
- **Two-finger semantics.** `position` in `move_gripper` is per-finger displacement
  (0 closed → ~0.044 fully open in the openarm MJCF). Each finger is driven to
  the same target via the same `set_ctrl_gripper_<side>` payload.
