# openarm01_gripper — mujoco variant

Rust peppy node that drives the gripper in a MuJoCo sim by directly reading and
writing the simulator's `MjData` arrays through a shared-memory bus published
by `openarm01_robot_initializer:mujoco`.

This is the **variant-as-driver** design: the gripper is the same peppy node
across every variant; only the driver layer (how it talks to the underlying
"hardware") changes. For mujoco, the driver is a `memmap2` reader/writer on
`$XDG_RUNTIME_DIR/peppy/sim/mjdata.bin`.

---

## Dependency

This variant requires `openarm01_robot_initializer:0.1.0` running in its
`mujoco` variant. That node owns the MuJoCo process and exposes the bus at
`$XDG_RUNTIME_DIR/peppy/sim/mjdata.{bin,meta.json}`. The bus path is bound
into the gripper container via `mount_paths: ["/run/user"]` in
`peppy.json5`.

If `robot_initializer:mujoco` is not running, the gripper will retry
opening the bus for ~30s then exit.

---

## Build

```bash
# From this directory (variants/mujoco/)
peppy node sync ../..              # regenerate peppygen (.peppy/libs/peppygen)
peppy node add . --variant mujoco
peppy node build openarm01_gripper:0.1.0
```

The apptainer build pulls `tuatini/peppy-rust-cargo-base:latest` and runs
`cargo build --release` inside the container, so no host-side cargo step is
needed for the node binary. (For the test harness see "Testing" below.)

---

## Run

Two instances — left and right — pinned to deterministic IDs so the test
harness can address them:

```bash
peppy node run openarm01_robot_initializer:0.1.0                # bring up MuJoCo first
peppy node run openarm01_gripper:0.1.0 gripper_id=0 -i left_gripper
peppy node run openarm01_gripper:0.1.0 gripper_id=1 -i right_gripper
peppy stack list                                                # confirm 2 gripper instances Ready
```

Expected log line from each gripper at startup:

```
INFO openarm01_gripper: bus open: nq=18 nv=18 nu=18 nbody=28
INFO openarm01_gripper: resolved side: qpos_addrs=[7, 8] ctrl_ids=[7, 8] ee_body=12 finger1_geoms=[39] finger2_geoms=[42]
```

`nq=18 nv=18 nu=18 nbody=28` must match what `robot_initializer:mujoco` logged
— same numbers prove both containers are reading the same `MjData`.

---

## Testing

A standalone Rust harness fires `move_gripper` at the running instances and
reads back feedback + result over the same zenoh transport the future
`joint_commander` will use. It is gated behind a `test-tools` cargo feature so
`peppy node build` does not ship it in the SIF.

```bash
cargo build --release --features test-tools --bin test_move_gripper

./target/release/test_move_gripper                              # both grippers, fully open
./target/release/test_move_gripper --side left --position 0.0   # close left only
./target/release/test_move_gripper --side right --position 0.022 --feedback-hz 20
```

`--position` is **per-finger** displacement in meters (0.0 closed → ~0.044
fully open). Each finger is independently driven to that value.

The harness reuses the gripper crate's existing peppygen path-dep, so no
Python venv, no peppylib install, no extra config. The compiled binary lives
at `target/release/test_move_gripper` and is excluded from the production SIF
via the `required-features = ["test-tools"]` declaration in `Cargo.toml`.

---

## Architecture notes (for review)

- **Bus location**: `$XDG_RUNTIME_DIR/peppy/sim` (i.e. `/run/user/$UID/peppy/sim`).
  Per-user isolation today; per-stack tmpfs is the long-term design once peppy's
  stack manager grows that feature. See the workaround comment in `apptainer.def`.
- **No peppygen transport between containers**: the gripper has its own
  peppygen generated inside its own image. The bus carries raw `MjData` bytes
  + a `mjdata.meta.json` shape descriptor.
- **Driver-only abstraction**: `src/drivers/mjdata_bus.rs` is the entire
  sim-specific surface. `src/actions/`, `src/services/`, `src/pipeline/`
  contain no MuJoCo-specific code — they call the driver via `bus.snapshot()`
  / `bus.write_ctrl()`. Swapping in a different driver (real hardware, Isaac)
  changes only that file.
- **Two-finger semantics**: `desired_position` is per-finger. Each finger's
  qpos ranges 0 (closed) → ~0.044 (fully open) in the openarm MJCF. Convergence
  uses worst-finger error; stall detection uses sum motion across a 500ms
  window so a hard contact (e.g., fingers pressed together at full close) is
  detected as `success=true msg="stalled at physical limit"` rather than a
  30s timeout.

---

## Known caveats (post-architecture-review follow-ups)

1. **Actuator one-sidedness in the MJCF** — commanding the fingers to a
   position partway between current state and "open" doesn't actively drive
   them when they're already past the commanded position. Pulling them closed
   works fine. Likely a `<position>` actuator without symmetric `forcerange`
   or an underdamped tendon model. Needs MJCF tuning at the asset level, not
   in this node.
2. **2mm tolerance is coarse** for fine positioning (a `0.021` request can
   stop at `0.019`). `POSITION_TOLERANCE_M` in `src/actions/move_gripper.rs`
   is the knob. Tightening to `5e-4` (0.5mm) gives finer stops at the cost of
   longer convergence.
3. **Stale ctrl between actions** — when no action is in flight, the gripper
   does not maintain its last commanded position. The `MjData.ctrl` mmap
   retains the last write, but if anything else touches it the fingers drift.
   A small idle-hold loop in `src/main.rs` would close this gap.
4. **Idle-time drift** — the right gripper's qpos can drift while the left is
   being commanded, which surfaces as instant-return goals (current position
   already in tolerance). Reset the sim between tests to get deterministic
   initial state.
