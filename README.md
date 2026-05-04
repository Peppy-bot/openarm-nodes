PeppyOS nodes for OpenArm01

## Prerequisites

- `peppy` CLI installed and daemon running
- Apptainer installed
- `rclone` installed (`sudo apt-get install rclone`) — required to download assets before building

## Running the backbone (MuJoCo)

Robot assets (MJCF + meshes) are downloaded from R2 and baked into the SIF at build time — no manual asset placement needed.

Register all dependency nodes first (`peppy node sync` requires them in the stack).
For each dependency, run `peppy node sync` then `peppy node add .` from its directory:

```bash
# From openarm01_nodes/
cd collisions_detection  && peppy node sync && peppy node add . && cd ..
cd inverse_kinematics    && peppy node sync && peppy node add . && cd ..
cd openarm01_arm         && peppy node sync && peppy node add . && cd ..
cd openarm01_gripper     && peppy node sync && peppy node add . && cd ..
```

Then register, sync, build, and run backbone:

```bash
cd openarm01_backbone
peppy node sync
peppy node add . --variant mujoco

# Step 1 — download assets from R2 into a host staging dir
RCLONE_S3_ACCESS_KEY_ID=<key_id> RCLONE_S3_SECRET_ACCESS_KEY=<secret> \
  bash scripts/download_assets.sh

# Step 2 — build SIF; %setup copies assets from staging dir into the container
peppy node build openarm01_backbone:0.1.0

# GUI
peppy node run openarm01_backbone:0.1.0 \
  node_root=/abs/path/to/openarm01_backbone/variants/mujoco \
  nodes_shared_code=/abs/path/to/nodes_shared_code

# Headless
PEPPY_BRIDGE_HEADLESS=1 peppy node run openarm01_backbone:0.1.0 \
  node_root=/abs/path/to/openarm01_backbone/variants/mujoco \
  nodes_shared_code=/abs/path/to/nodes_shared_code
```

> **Building without assets:** skipping `download_assets.sh` emits a warning but still produces a valid SIF. In that case set `PEPPY_ROBOT_ASSETS_DIR` at runtime to point to a local assets directory.

### Runtime parameters

| Parameter          | Description                                              |
|--------------------|----------------------------------------------------------|
| `node_root`        | Absolute path to `variants/mujoco/` — bind-mounted at `/opt/mujoco_backbone` |
| `nodes_shared_code`| Absolute path to `nodes_shared_code/` repo root          |

### Environment variables

| Variable                  | Default                               | Description                                          |
|---------------------------|---------------------------------------|------------------------------------------------------|
| `PEPPY_BRIDGE_HEADLESS`   | `0`                                   | Set to `1` to run without a viewer                   |
| `PEPPY_BRIDGE_PRESET`     | `mujoco_openarm`                      | Preset config name under `config/presets/`           |
| `PEPPY_ROBOT_ASSETS_DIR`  | `/opt/robot_assets/openarm/mujoco`    | Path to the directory containing the MJCF and meshes |

## Running the backbone (Isaac Sim)

Assets are baked into the SIF at build time via the same R2 download mechanism.

```bash
cd openarm01_backbone
peppy node sync
peppy node add . --variant isaac

# Step 1 — download assets from R2 into a host staging dir
RCLONE_S3_ACCESS_KEY_ID=<key_id> RCLONE_S3_SECRET_ACCESS_KEY=<secret> \
  bash scripts/download_assets.sh

# Step 2 — build SIF
peppy node build openarm01_backbone:0.1.0

peppy node run openarm01_backbone:0.1.0 \
  node_root=/abs/path/to/openarm01_backbone/variants/isaac \
  nodes_shared_code=/abs/path/to/nodes_shared_code \
  home_dir=/home/<user>
```

### Environment variables

| Variable                  | Default                               | Description                                        |
|---------------------------|---------------------------------------|----------------------------------------------------|
| `PEPPY_BRIDGE_HEADLESS`   | `0`                                   | Set to `1` to run without a viewer                 |
| `PEPPY_BRIDGE_PRESET`     | `isaac_openarm`                       | Preset config name under `config/presets/`         |
| `PEPPY_ROBOT_ASSETS_DIR`  | `/opt/robot_assets/openarm/isaac`     | Path to the directory containing the USD and config|