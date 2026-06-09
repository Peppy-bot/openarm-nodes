# openarm01_robot_initializer

Manages the simulation process lifecycle for the OpenArm01 bimanual robot. Exposes the `is_ready` service that dependent nodes poll before initialising — ensuring the simulation world is fully loaded before any control logic runs.

Under peppy v0.10's interface-conformance model, each implementation is a separate top-level node that `conforms_to openarm01_robot_initializer:v1`.

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
- `aaqibmahamood/openarm01-isaac-sim:5.1.0-7` (Isaac sim base)
- `aaqibmahamood/openarm01-mujoco-sim:3.8.1-7` (MuJoCo sim base)

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
      exts/                                # MuJoCo engine wrappers (sensors + actuator_ctrl)

  openarm01_robot_initializer_isaac/       # Python ext, Isaac Sim
    peppy.json5                            # conforms_to openarm01_robot_initializer:v1
    apptainer.def
    robots/openarm/
      _launcher.py                         # SimLauncher — loads USD stage, runs timeline
      bridge_extension.py
      openarm/launch.py                    # entrypoint, Isaac Sim must own main thread
      config/sim_bridge.json5
      exts/                                # Isaac Sim engine wrappers (sensors + actuator_ctrl)
```

