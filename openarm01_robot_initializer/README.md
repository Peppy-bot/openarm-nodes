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

```
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
peppy node run openarm01_robot_initializer:0.1.0 node_root=<abs_path_to_variants/mujoco>
```

Runs headless by default. To open the viewer GUI:

```bash
PEPPY_BRIDGE_HEADLESS=0 peppy node run openarm01_robot_initializer:0.1.0 node_root=<abs_path_to_variants/mujoco>
```

### Isaac Sim

```bash
peppy node add . --variant isaac -sb
peppy node run openarm01_robot_initializer:0.1.0 \
  node_root=<abs_path_to_variants/isaac> \
  home_dir=<home_dir>
```

Runs headless by default. Requires NVIDIA GPU. To open the GUI:

```bash
PEPPY_BRIDGE_HEADLESS=0 peppy node run openarm01_robot_initializer:0.1.0 \
  node_root=<abs_path_to_variants/isaac> \
  home_dir=<home_dir>
```

## Assets

Robot assets (USD, MJCF, meshes) are not committed to the repository. Fetch them before building:

```bash
RCLONE_S3_ACCESS_KEY_ID=<key> RCLONE_S3_SECRET_ACCESS_KEY=<secret> \
  bash scripts/download_assets.sh
```

Assets are baked into the SIF image at build time. Re-run `download_assets.sh` and rebuild whenever assets change.

## Project structure

```
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
