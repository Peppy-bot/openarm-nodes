# openarm01_robot_initializer

Manages the simulation process lifecycle for the OpenArm01 bimanual robot. Exposes the `is_ready` service that dependent nodes poll before initialising — ensuring the simulation world is fully loaded before any control logic runs.

Under peppy v0.10's interface-conformance model, the former `variants/{default,isaac,mujoco}` sub-folders are gone. Each implementation is now a separate top-level node that `conforms_to openarm01_robot_initializer:v1` (interface defined in [Peppy-bot/interfaces_hub](https://github.com/Peppy-bot/interfaces_hub/blob/main/openarm01/robot_initializer.json5)).

## Implementations

| Node directory | Language | Use case |
|---|---|---|
| `openarm01_robot_initializer/` | Rust | Real robot — no simulation, `is_ready` returns `true` instantly |
| `openarm01_robot_initializer_mujoco/` | Python | MuJoCo simulation — `is_ready: true` once the model is loaded |
| `openarm01_robot_initializer_isaac/` | Python | Isaac Sim simulation — `is_ready: true` once the stage warmup completes |

All three implementations expose exactly one service from the interface:

```yaml
is_ready:
  response: { ready: bool }
```

Consumers (e.g. `openarm01_backbone`) depend on `openarm01_robot_initializer:v1` and bind to one of the three implementations via the launcher's `bindings:` block. The consumer's binary is identical across deployments.

## Running

The interface (`openarm01_robot_initializer:v1`) is resolved from the configured `interfaces_hub` repo. If you haven't registered it locally yet:

```bash
peppy repo add /path/to/interfaces_hub          # or the git URL once published
peppy repo refresh
```

### Real robot

```bash
peppy node add openarm01_robot_initializer -sb
peppy node run openarm01_robot_initializer:v1
```

### MuJoCo

```bash
peppy node add openarm01_robot_initializer_mujoco -sb
peppy node run openarm01_robot_initializer_mujoco:v1
```

Headless by default. Open `http://<host>:8080` in any browser for the [mjviser](https://github.com/mujocolab/mjviser) view — no GPU or display required client-side.

To open the native MuJoCo viewer window on the same machine instead:

```bash
PEPPY_BRIDGE_HEADLESS=0 peppy node run openarm01_robot_initializer_mujoco:v1
```

### Isaac Sim

```bash
peppy node add openarm01_robot_initializer_isaac -sb
peppy node run openarm01_robot_initializer_isaac:v1
```

Headless with WebRTC streaming by default. Connect via the [Isaac Sim Livestream client](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/manual_livestream_clients.html). Requires NVIDIA GPU.

To open the Isaac Sim window on the same machine instead:

```bash
PEPPY_BRIDGE_HEADLESS=0 peppy node run openarm01_robot_initializer_isaac:v1
```

## Assets

Robot assets (USD, MJCF, meshes) are baked into the container images:
- `aaqibmahamood/openarm01-isaac-sim:5.1.0` (Isaac variant base)
- `aaqibmahamood/openarm01-mujoco-sim:3.8.1-4` (current MuJoCo variant base — see Status below for the rebump)

Contributors do not need to fetch assets — `peppy node build` pulls the pre-built images from Docker Hub.

To rebuild the base images (maintainers only, requires R2 credentials):

```bash
RCLONE_S3_ACCESS_KEY_ID=<key> RCLONE_S3_SECRET_ACCESS_KEY=<secret> \
  bash scripts/build_base_images.sh
```

## Project structure

```text
openarm01_nodes/
  openarm01_robot_initializer/             # Rust, real robot
    peppy.json5                            # conforms_to openarm01_robot_initializer:v1
    Cargo.toml + src/{main.rs, service.rs}
    apptainer.def
    scripts/, README.md (this file)

  openarm01_robot_initializer_mujoco/      # Python ext, MuJoCo
    peppy.json5                            # conforms_to openarm01_robot_initializer:v1
    apptainer.def
    robots/openarm/
      _launcher.py                         # SimLauncher — loads model, steps physics
      bridge_extension.py                  # plugin registry + step hook
      openarm/launch.py                    # entrypoint, wires peppylib + SimLauncher
      config/sim_bridge.json5              # publisher/subscriber declarations

  openarm01_robot_initializer_isaac/       # Python ext, Isaac Sim
    peppy.json5                            # conforms_to openarm01_robot_initializer:v1
    apptainer.def
    robots/openarm/
      _launcher.py                         # SimLauncher — loads USD stage, runs timeline
      bridge_extension.py
      openarm/launch.py                    # entrypoint, Isaac Sim must own main thread
      config/sim_bridge.json5
```

## Build / deployment status

This README reflects the v0.10 migration on branch `feat/robot-initializer-bridge-ext`. Two upstream dependencies must land before the full stack passes end-to-end:

1. **`openarm01_robot_initializer:v1` interface** — [`Peppy-bot/interfaces_hub#4`](https://github.com/Peppy-bot/interfaces_hub/pull/4). Until merged, CI on this repo cannot resolve `conforms_to`. Locally registered interfaces_hub checkouts resolve fine.
2. **`sim_ext_core` peppylib SenderTarget fix** — [`Peppy-bot/nodes_shared_code#4`](https://github.com/Peppy-bot/nodes_shared_code/pull/4). v0.10 peppylib tightened `as_target` / `from_target` from `str` to `peppylib.messaging.SenderTarget`. Until merged + the MuJoCo base image is rebuilt against the new revision, every raw JSON publish/subscribe between `_mujoco` and its sim peripherals (`gripper_mujoco`, `arm_mujoco`) fails silently with `'str' object is not an instance of 'SenderTarget'` warnings — the node still runs and `is_ready` still responds, but no engine-internal topics propagate.

### Verifying after both PRs land

1. Bump `From:` in `openarm01_robot_initializer_mujoco/apptainer.def` from `aaqibmahamood/openarm01-mujoco-sim:3.8.1-4` to whatever tag the rebuilt image gets (e.g. `:3.8.1-5`).
2. `peppy node add openarm01_robot_initializer_mujoco -sb` → SIF rebuild.
3. `peppy node run openarm01_robot_initializer_mujoco:v1`.
4. Expected log shape — boot:
   - `Loading model: /opt/robot_assets/openarm/mujoco/openarm_bimanual.xml`
   - `Registered publisher: joint_states → topic='joint_states'` (and friends)
   - `MujocoBridgeExtension ready — 12 plugin(s)`
   - `peppylib connected (instance_id=...)`
   - `Scene loaded — is_ready: true`
   - **No `Failed to emit` warnings, no `Subscribe error` warnings.** (The current build floods both — that's the bug the SenderTarget fix closes.)
5. Confirm `is_ready` responds: any consumer polling the service should see `{ ready: true }` after the scene loads.
6. Stop with `peppy node stop <instance-id>`.

### Isaac side

Isaac requires NVIDIA GPU which the current dev machine does not have. `_isaac` was structurally migrated and syncs cleanly under v0.10 but has not been runtime-tested. Same SenderTarget fix applies; same base-image rebump procedure (`aaqibmahamood/openarm01-isaac-sim:5.1.0` → whatever tag the rebuilt image gets).
