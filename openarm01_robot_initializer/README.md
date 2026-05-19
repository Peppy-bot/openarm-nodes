# openarm01_robot_initializer

Manages the simulation process lifecycle for the OpenArm01 bimanual robot. Exposes an `is_ready` service that dependent nodes poll before initialising — ensuring the simulation world is fully loaded before any control logic runs.

## Variants

| Variant | Language | Use case |
|---|---|---|
| `default` | Rust | Real robot — no simulation, always returns `ready: true` immediately |
| `mujoco` | Python | MuJoCo simulation |
| `isaac` | Python | Isaac Sim simulation |

## Interface

**Exposed service: `is_ready`**

```yaml
response: { ready: bool }
```

Returns `false` while the simulation is loading, `true` once the scene is fully initialised and the physics timeline is running.

## Running

### default (real robot)

```bash
peppy node add . --variant default -sb
peppy node run openarm01_robot_initializer:0.1.0
```

### MuJoCo

```bash
peppy node add . --variant mujoco -sb
peppy node run openarm01_robot_initializer:0.1.0
```

Runs headless by default. Open `http://<host>:8080` in any browser to view the simulation via [mjviser](https://github.com/mujocolab/mjviser) — no GPU or display required on the client.

To open the native MuJoCo viewer window on the same machine instead:

```bash
PEPPY_BRIDGE_HEADLESS=0 peppy node run openarm01_robot_initializer:0.1.0
```

### Isaac Sim

```bash
peppy node add . --variant isaac -sb
peppy node run openarm01_robot_initializer:0.1.0
```

Runs headless with WebRTC streaming by default. Connect via the [Isaac Sim Livestream client](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/manual_livestream_clients.html). Requires NVIDIA GPU.

To open the Isaac Sim window on the same machine instead:

```bash
PEPPY_BRIDGE_HEADLESS=0 peppy node run openarm01_robot_initializer:0.1.0
```

## Assets

Robot assets (USD, MJCF, meshes) are baked into the container images at `aaqibmahamood/openarm01-isaac-sim:5.1.0` and `aaqibmahamood/openarm01-mujoco-sim:3.8.1`. Contributors do not need to fetch assets — `peppy node build` pulls the pre-built images from Docker Hub.

To rebuild the base images (maintainers only, requires R2 credentials):

```bash
RCLONE_S3_ACCESS_KEY_ID=<key> RCLONE_S3_SECRET_ACCESS_KEY=<secret> \
  bash scripts/build_base_images.sh
```

## Project structure

```text
openarm01_robot_initializer/
  peppy.json5                          # root manifest + is_ready service interface
  scripts/
    download_assets.sh                 # fetches assets from R2 into /tmp staging dirs
  variants/
    default/                           # Rust no-op variant
      src/main.rs
      src/service.rs
      Cargo.toml
      peppy.json5
      apptainer.def
    mujoco/                            # Python MuJoCo variant
      robots/openarm/
        _launcher.py                   # SimLauncher — loads model, steps physics
        openarm/launch.py              # entrypoint, wires peppylib + SimLauncher
      peppy.json5
      apptainer.def
    isaac/                             # Python Isaac Sim variant
      robots/openarm/
        _launcher.py                   # SimLauncher — loads USD stage, runs timeline
        openarm/launch.py              # entrypoint, Isaac Sim must own main thread
      peppy.json5
      apptainer.def
```
