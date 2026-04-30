PeppyOS nodes for OpenArm01

## Prerequisites

- `peppy` CLI installed and daemon running
- Apptainer installed

## Running the backbone (MuJoCo)

Place the MJCF model and meshes in the assets directory before running:

```
openarm01_backbone/variants/mujoco/robots/openarm/openarm/assets/
  openarm_bimanual.xml
  meshes/
```

Register, build, and run:

```bash
# From openarm01_nodes/
peppy node add openarm01_backbone --variant mujoco
peppy node build openarm01_backbone --variant mujoco

# Run (headless recommended for containers)
PEPPY_BRIDGE_HEADLESS=1 peppy node run openarm01_backbone:0.1.0 \
  node_root=<abs-path-to>/openarm01_backbone/variants/mujoco \
  nodes_shared_code=<abs-path-to>/nodes_shared_code
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
