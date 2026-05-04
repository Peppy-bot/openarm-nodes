PeppyOS nodes for OpenArm01

## Prerequisites

- `peppy` CLI installed and daemon running
- Apptainer installed

## Running the backbone (MuJoCo)

Place the MJCF model and meshes in the assets directory before running:

```text
openarm01_backbone/variants/mujoco/robots/openarm/openarm/assets/
  openarm_bimanual.xml
  meshes/
```

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

### Runtime parameters

| Parameter          | Description                                              |
|--------------------|----------------------------------------------------------|
| `node_root`        | Absolute path to `variants/mujoco/` — bind-mounted at `/opt/mujoco_backbone` |
| `nodes_shared_code`| Absolute path to `nodes_shared_code/` repo root          |

### Environment variables

| Variable               | Default          | Description                          |
|------------------------|------------------|--------------------------------------|
| `PEPPY_BRIDGE_HEADLESS`| `0`              | Set to `1` to run without a viewer   |
| `PEPPY_BRIDGE_PRESET`  | `mujoco_openarm` | Preset config name under `config/presets/` |

## Isaac Sim variant

> **TODO:** The Isaac variant (`variants/isaac/`) has the extension scaffold in place. Update this section once the Isaac variant is validated.
