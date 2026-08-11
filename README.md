# OpenArm Nodes

[Peppy](https://github.com/Peppy-bot/peppy) nodes for the OpenArm bimanual robot.

The repository supports OpenArm v1.0 and v2.0 hardware together with simulation workflows using MuJoCo and NVIDIA Isaac Sim. The OpenArm system provides two 7-DOF arms and two grippers. Peppy is used to build, launch, connect and manage the individual robot components.

The Isaac Sim integration currently targets **NVIDIA Isaac Sim 6.0.1** and adds runtime control for arm joints, robot root position, complete USD environments, NVIDIA Isaac asset-root environments, local and remote USD objects, object scaling, object movement/removal, optional static and dynamic physics and live environment replacement without rebuilding Isaac Sim.

The simulation node remains:

```text
openarm_sim_isaac:v1
```

The `v1` tag is the Peppy node version. It is independent of the Isaac Sim version now.

---

## Repository overview

| Component | What it does |
| --- | --- |
| [`openarm_robot_initializer`](./openarm_robot_initializer) | Initializes the OpenArm robot environment |
| [`openarm_arm`](./openarm_arm) | Drives one OpenArm arm side with 7 joints |
| [`openarm_gripper`](./openarm_gripper) | Drives one v1.0 gripper |
| [`openarm_gripper_v2`](./openarm_gripper_v2) | Drives one v2.0 gripper |
| [`openarm_backbone`](./openarm_backbone) | Routes goals to the correct side |
| [`openarm_commander`](./openarm_commander) | Browser-oriented OpenArm control |
| [`openarm_sim_isaac`](./openarm_sim_isaac) | Isaac Sim 6.0.1 OpenArm simulation runtime |

The real and simulated components expose the same Peppy-level interfaces so higher-level components do not need to know which simulation or hardware backend is being used.

---

# 1. Prerequisites

Recommended host configuration:

- Ubuntu 24.04 LTS
- NVIDIA GPU for Isaac Sim
- NVIDIA driver compatible with Isaac Sim 6.0.1
- Docker
- NVIDIA Container Toolkit
- Git
- Peppy v0.23.1 or newer

MuJoCo can run without an NVIDIA GPU. Isaac Sim requires supported NVIDIA GPU and correctly configured container GPU access.

---

# 2. Install Peppy

Install Peppy v0.23.1:

```sh
curl -fsSL https://peppy.bot/install.sh | PEPPY_VERSION="v0.23.1" sh
```

Verify:

```sh
peppy --version
```

---

# 3. Clone the repositories

A typical workspace is:

```text
ws/
├── contracts-hub/
├── launchers-hub/
└── openarm-nodes/
```

Create the workspace and clone the repositories:

```sh
mkdir -p ~/ws
cd ~/ws

git clone https://github.com/Peppy-bot/contracts-hub.git
git clone https://github.com/Peppy-bot/launchers-hub.git
git clone https://github.com/Peppy-bot/openarm-nodes.git
```

Enter the OpenArm repository:

```sh
cd ~/ws/openarm-nodes
```

During development, switch to the required feature branch if needed:

```sh
git checkout feature/isaacsim-6.0.1-clean
```

---

# 4. Start Peppy

Start the Peppy service:

```sh
peppy service serve &
```

Register the repositories:

```sh
peppy repo add ~/ws/contracts-hub
peppy repo add ~/ws/openarm-nodes
peppy repo refresh
```

Check registration:

```sh
peppy repo list
```

---

# 5. Isaac Sim container architecture

The Isaac integration uses custom base image derived from NVIDIA Isaac Sim 6.0.1.

Current image:

```text
peppybot/openarm-isaac-sim:6.0.1-1
```

Base image:

```text
nvcr.io/nvidia/isaac-sim:6.0.1
```

The corresponding Apptainer definition is:

```text
openarm_sim_isaac/apptainer.def
```

Version relationship:

```text
Peppy node version     v1
Isaac Sim version      6.0.1
Container revision     1
```

These version numbers are intentionally independent.

---

# 6. Build the custom Isaac base image

Normal users do not need to rebuild Docker base image. This step is intended for maintainers when upgrading Isaac Sim, changing baked OpenArm assets or changing system-level dependencies.

The build script is:

```text
openarm_robot_initializer/scripts/build_base_images.sh
```

Build only the Isaac image:

```sh
cd ~/ws/openarm-nodes
./openarm_robot_initializer/scripts/build_base_images.sh --isaac-only
```

Normal Python changes too Isaac node do not require rebuilding this Docker image.

---

# 7. Build the Isaac Peppy node

```sh
cd ~/ws/openarm-nodes/openarm_sim_isaac
peppy node sync .
peppy node add . -sb --force --idle-timeout 18000
```

The generated SIF is normally available at:

```text
~/.peppy/built_nodes/openarm_sim_isaac_v1.sif
```

After changing Isaac-side Python code, repeat:

```sh
cd ~/ws/openarm-nodes/openarm_sim_isaac
peppy node sync .
peppy node add . -sb --force --idle-timeout 18000
```

---

# 8. Build the MuJoCo stack

```sh
peppy node add ~/ws/openarm-nodes/openarm_robot_initializer_mujoco -sb --idle-timeout 18000
peppy node add ~/ws/openarm-nodes/openarm_arm_mujoco -sb --idle-timeout 1800
peppy node add ~/ws/openarm-nodes/openarm_gripper_mujoco -sb --idle-timeout 1800
peppy node add ~/ws/openarm-nodes/openarm_backbone -sb --idle-timeout 1800
peppy node add ~/ws/openarm-nodes/openarm_commander -sb --idle-timeout 1800
```

---

# 9. Build the real robot stack

```sh
peppy node add ~/ws/openarm-nodes/openarm_robot_initializer -sb --idle-timeout 1800
peppy node add ~/ws/openarm-nodes/openarm_arm -sb --idle-timeout 1800
peppy node add ~/ws/openarm-nodes/openarm_gripper -sb --idle-timeout 1800
```

For OpenArm v2.0 hardware, use `openarm_gripper_v2` instead of `openarm_gripper`.

---

# 10. Verify built nodes

```sh
peppy stack list
```

Built nodes should report `Stage: Ready`.

---

# 11. Run Isaac Sim standalone

When running the Isaac simulator by itself with Peppy v0.23.1+, explicitly mark optional arm and gripper pairings as vacant:

```sh
peppy node run \
  -i isaac601_test \
  --idle-timeout 1800 \
  --max-timeout 7200 \
  --vacant-link 'left_arm=standalone simulation' \
  --vacant-link 'right_arm=standalone simulation' \
  --vacant-link 'left_gripper=standalone simulation' \
  --vacant-link 'right_gripper=standalone simulation' \
  openarm_sim_isaac:v1 \
  hardware_version=v2 \
  state_rate_hz=50 \
  headless=true
```

Leave this terminal running while using the runtime commander.

---

# 12. Runtime commander

The repository-level runtime client is:

```text
commander.py
```

Use it from the repository root:

```sh
cd ~/ws/openarm-nodes
python3 commander.py --help
```

ruuntime commander communicates with running Isaac node using JSON over TCP.

Default server:

```text
host: 127.0.0.1
port: 5556
```

Override with:

```text
PEPPY_ISAAC_COMMAND_HOST
PEPPY_ISAAC_COMMAND_PORT
```

---

# 13. Runtime command architecture

```text
commander.py
     │
     │ JSON / TCP
     ▼
runtime_commander_server.py
     │
     │ queued onto Isaac simulation thread
     ▼
SimLauncher.execute_runtime_command()
     │
     ├── arm control
     ├── robot root movement
     ├── local USD scene loading
     ├── NVIDIA Isaac scene loading
     ├── local USD object spawning
     ├── NVIDIA Isaac object spawning
     ├── object movement
     ├── object removal
     └── optional runtime physics
```

---

# 14. Control an OpenArm arm

Left arm:

```sh
python3 commander.py arm left \
  0.0 0.2 0.0 -0.8 0.0 0.6 0.0
```

Right arm:

```sh
python3 commander.py arm right \
  0.0 -0.2 0.0 -0.8 0.0 0.6 0.0
```

Each command supplies seven joint targets.

---

# 15. Release runtime arm control

```sh
python3 commander.py release left
python3 commander.py release right
```

---

# 16. List detected joints

```sh
python3 commander.py joints
```

---

# 17. Move the OpenArm robot root

```sh
python3 commander.py robot 0.0 0.0 0.0
```

Example:

```sh
python3 commander.py robot 1.0 0.0 0.0
```

Coordinates are expressed in the Isaac world frame. `Z` is the vertical/up axis and distances are normally metres.

---

# 18. Runtime scene model

Runtime environments are loaded under:

```text
/World/RuntimeScene
```

Runtime objects are loaded under:

```text
/World/RuntimeObjects/<name>
```

This keeps dynamically loaded content separate from the OpenArm articulation and allows environments and props to be replaced without rebuilding the simulator.

---

# 19. Built-in OpenArm test scenes

Available lightweight test scenes:

```text
tabletop
shelf_reach
```

Load them with:

```sh
python3 commander.py scene tabletop
python3 commander.py scene shelf_reach
```

The CLI accepts a uniform scale parameter:

```sh
python3 commander.py scene tabletop --scale 0.7
python3 commander.py scene shelf_reach --scale 0.7
```

Use scaling with launcher versions that implement built-in scene scaling.

---

# 20. Load an arbitrary local USD scene

```sh
python3 commander.py scene-usd \
  /absolute/path/to/environment.usd \
  --scale 1.0
```

Example:

```sh
python3 commander.py scene-usd \
  /data/workcell.usd \
  --scale 0.7
```

The USD must be visible from inside Isaac/Apptainer runtime.

Loading another runtime scene replaces the previous `/World/RuntimeScene`.

---

# 21. Load NVIDIA Isaac environments directly

The runtime can resolve `Isaac/...` paths against IsaacSim configured asset root.

Load smaller warehouse:

```sh
python3 commander.py scene-isaac \
  Isaac/Environments/Simple_Warehouse/warehouse.usd \
  --scale 1.0
```

Load full warehouse:

```sh
python3 commander.py scene-isaac \
  Isaac/Environments/Simple_Warehouse/full_warehouse.usd \
  --scale 1.0
```

No manual USD download is required when selected asset exists ine configured Isaac asset root.

---

# 22. Replace an environment live

```sh
python3 commander.py clear-scene

python3 commander.py scene-isaac \
  Isaac/Environments/Simple_Warehouse/warehouse.usd \
  --scale 1.0
```

Switch again:

```sh
python3 commander.py clear-scene

python3 commander.py scene-isaac \
  Isaac/Environments/Simple_Warehouse/full_warehouse.usd \
  --scale 1.0
```

These changes do not require Isaac rebuild.

---

# 23. Clear the current runtime scene

```sh
python3 commander.py clear-scene
```

This removes `/World/RuntimeScene` without intentionally removing the OpenArm robot.

---

# 24. Spawn a local USD object

```sh
python3 commander.py spawn \
  object_name \
  /absolute/path/to/object.usd \
  0.7 0.0 1.0 \
  --scale 0.5
```

Arguments are:

```text
name
USD path
X
Y
Z
```

The resulting prim is created beneath:

```text
/World/RuntimeObjects/<name>
```

---

# 25. Spawn NVIDIA Isaac assets

YCB power drill:

```sh
python3 commander.py spawn-isaac drill \
  Isaac/Props/YCB/Axis_Aligned/035_power_drill.usd \
  0.7 0.0 1.0 \
  --scale 0.7 \
  --physics none
```

Cracker box:

```sh
python3 commander.py spawn-isaac cracker \
  Isaac/Props/YCB/Axis_Aligned/003_cracker_box.usd \
  0.7 -0.15 1.0 \
  --scale 0.7 \
  --physics none
```

The object name becomes the runtime prim name, for example:

```text
/World/RuntimeObjects/drill
```

---

# 26. Object scaling

Runtime-spawned objects support uniform scaling.

```sh
python3 commander.py spawn-isaac drill \
  Isaac/Props/YCB/Axis_Aligned/035_power_drill.usd \
  0.7 0.0 1.0 \
  --scale 0.5 \
  --physics none
```

Typical values:

```text
1.0   original size
0.7   70% size
0.5   half size
1.5   150% size
```

Scaling is useful for adapting generic assets to the OpenArm reachable workspace.

---

# 27. Runtime physics

Spawned objects support three physics modes:

```text
none
static
dynamic
```

## No additional physics

```sh
--physics none
```

The imported USD is referenced without recursively adding extra collision or rigid-body APIs.

Recommended for:

- complex assets
- articulated assets
- conveyors
- complete environments
- assets that already contain authored physics

Example:

```sh
python3 commander.py spawn-isaac drill \
  Isaac/Props/YCB/Axis_Aligned/035_power_drill.usd \
  0.7 0.0 1.0 \
  --scale 0.7 \
  --physics none
```

## Static physics

```sh
--physics static
```

Static mode adds collision behaviour without making the object movalbe rigid body.

Typical uses:

- tables
- shelves
- workbenches
- walls
- fixed fixtures

Example:

```sh
python3 commander.py spawn table \
  /data/table.usd \
  0.8 0.0 0.5 \
  --scale 0.7 \
  --physics static
```

## Dynamic physics

```sh
--physics dynamic
```

Dynamic mode adds runtime collision behaviour, rigid-body behaviour, and mass.

```sh
python3 commander.py spawn-isaac drill \
  Isaac/Props/YCB/Axis_Aligned/035_power_drill.usd \
  0.7 0.0 1.0 \
  --scale 0.7 \
  --physics dynamic \
  --mass 0.5
```

Mass is in kilograms. For mesh-based dynamic objects, the runtime attempts to use a convex collision approximation.

---

# 28. Physics performance warning

The runtime physics helper traverses imported USD hierarchy and may apply collision APIs to geometry prims. This can be expensive for large assets.

Avoid automatic `static` or `dynamic` physics on large or complex assets such as conveyors, full environments, large architectural models, or articulated machinery unless necessary.

Start with:

```sh
--physics none
```

If the asset loads correctly so add physics selectively.

---

# 29. Move a runtime object

```sh
python3 commander.py move drill 0.75 0.1 0.9
```

The first argument is the name used when the object was spawned.

---

# 30. Remove a runtime object

```sh
python3 commander.py remove drill
```

Other examples:

```sh
python3 commander.py remove cracker
python3 commander.py remove conveyor
```

Only named runtime object is removed.

---

# 31. Example live workflow

Start Isaac Sim with then from another terminal:

```sh
cd ~/ws/openarm-nodes
```

Load a warehouse:

```sh
python3 commander.py scene-isaac \
  Isaac/Environments/Simple_Warehouse/warehouse.usd \
  --scale 1.0
```

Spawn a drill:

```sh
python3 commander.py spawn-isaac drill \
  Isaac/Props/YCB/Axis_Aligned/035_power_drill.usd \
  0.7 0.0 1.0 \
  --scale 0.7 \
  --physics none
```

Move the drill:

```sh
python3 commander.py move drill 0.65 -0.15 0.9
```

Move the left arm:

```sh
python3 commander.py arm left \
  0.0 0.2 0.0 -0.8 0.0 0.6 0.0
```

Remove the drill:

```sh
python3 commander.py remove drill
```

Replace the environment:

```sh
python3 commander.py clear-scene

python3 commander.py scene-isaac \
  Isaac/Environments/Simple_Warehouse/full_warehouse.usd \
  --scale 1.0
```

All of this can happen without rebuilding or restarting Isaac Sim.

---

# 32. Isaac browser viewer

Isaac Sim can be viewed through WebRTC. A browser viewer can be run separately using NVIDIA's Isaac Sim web viewer. Use Chrome on linux dist

Typical ports:

```text
8210/TCP     browser UI
49100/TCP    WebRTC signalling
47998/UDP    media stream
```

Do not hard-code local machine IP addresses into the repository.

A typical viewer launch is:

```sh
ISAACSIM_HOST=<ISAAC_HOST_IP> \
ISAACSIM_SIGNAL_PORT=49100 \
ISAACSIM_STREAM_PORT=47998 \
docker compose \
  -f tools/docker/docker-compose.yml \
  up -d --no-deps web-viewer
```

Then open:

```text
http://<ISAAC_HOST_IP>:8210
```

If the viewer stays at `WAITING FOR STREAM`, check that another native WebRTC client is not already connected.

---

# 33. MuJoCo viewer

A typical MuJoCo launcher is:

```sh
peppy stack launch \
  ~/ws/launchers-hub/openarm/openarm_v2_teleop_mujoco.json5
```

Follow ports reported by stack for MuJoCo browser viewer and OpenArm commander UI.

---

# 34. Real robot workflow

The real robot uses the same overall Peppy architecture. Build hardware nodes and launch corresponding OpenArm hardware stack from `launchers-hub`.

Higher-level components are intended to remain independent of whether the backend is:

```text
real robot
MuJoCo
Isaac Sim
```

---

# 35. Runtime USD hierarchy

A typical stage layout is:

```text
/World
├── RuntimeScene
├── RuntimeObjects
│   ├── drill
│   ├── cracker
│   └── ...
└── ...

/openarm
└── OpenArm articulation
```

This separation allows scenes and objects to be removed or replaced without intentionally removing robot.

---

# 36. Local USD visibility

A local USD path passed to `scene-usd` or `spawn` must be visible from inside Isaac/Apptainer runtime.

Using a host path does not automatically guarantee that the path is available inside container. Configure an appropriate Apptainer bind or runtime mount when required.

---

# 37. Updating code

Changes to the local `commander.py` CLI do not by themselves require rebuilding SIF.

Changes to files that execute inside Isaac node, such as:

```text
openarm_sim_isaac/robots/openarm/_launcher.py
openarm_sim_isaac/robots/openarm/runtime_commander_server.py
openarm_sim_isaac/robots/openarm/launch.py
```

require rebuilding Peppy node:

```sh
cd ~/ws/openarm-nodes/openarm_sim_isaac
peppy node sync .
peppy node add . -sb --force --idle-timeout 18000
```

Restart Isaac node afterward.

---

# 38. Troubleshooting

## `repo-node 'X:v1' not found in nodes.json5`

```sh
peppy repo add ~/ws/openarm-nodes
peppy repo refresh
peppy repo list
```

## Optional links are not paired

Use:

```sh
--vacant-link 'left_arm=standalone simulation'
--vacant-link 'right_arm=standalone simulation'
--vacant-link 'left_gripper=standalone simulation'
--vacant-link 'right_gripper=standalone simulation'
```

## Isaac node build times out

```sh
peppy node add . -sb --force --idle-timeout 18000
```

## Runtime command prints JSON but nothing changes

Check that:

1. Isaac Sim is running.
2. `runtime_commander_server.py` started successfully.
3. TCP port `5556` is reachable.
4. `_launcher.py` supports the requested command.
5. The SIF was rebuilt after launcher-side changes.
6. `commander.py` is being run from the correct repository.

## `scene-isaac` does nothing

Verify that runtime includes `load_isaac_scene` and rebuild the SIF after adding it.

Test with:

```sh
python3 commander.py scene-isaac \
  Isaac/Environments/Simple_Warehouse/warehouse.usd \
  --scale 1.0
```

## `spawn-isaac` does nothing

Verify that the runtime includes `spawn_isaac_asset` and rebuild the node.

Test with:

```sh
python3 commander.py spawn-isaac drill \
  Isaac/Props/YCB/Axis_Aligned/035_power_drill.usd \
  0.7 0.0 1.0 \
  --scale 0.7 \
  --physics none
```

## Isaac freezes while spawning a complex object

Retry with:

```sh
--physics none
```

Example:

```sh
python3 commander.py spawn-isaac conveyor \
  Isaac/Props/Conveyors/ConveyorBelt_A08.usd \
  1.0 0.0 0.2 \
  --scale 0.6 \
  --physics none
```

Use automatic `static` or `dynamic` physics mainly for smaller manipulation objects.

## NVIDIA asset does not appear

The path must exist in the configured Isaac asset root. Start with a known asset and `--physics none`. Remote assets may take time to resolve.

## Local USD cannot be found

Use an absolute path and make sure that location is visible inside Apptainer.

## Browser viewer stays at `WAITING FOR STREAM`

Check:

- Isaac Sim is running
- the correct host IP is configured
- TCP signalling is reachable
- UDP streaming is reachable
- another WebRTC client is not already consuming the stream

## Black Isaac stream

If necessary:

```sh
rm -rf ~/.cache/isaac-sim
```

Then restart Isaac.

## Peppy build completes but changes do not appear

Run both:

```sh
peppy node sync .
peppy node add . -sb --force --idle-timeout 18000
```

Then restart the node.

---

# 40. Current Isaac Sim runtime capabilities

```text
Start Isaac Sim
      │
      ▼
Load an environment
      │
      ├── local USD
      │
      └── NVIDIA Isaac asset root
      │
      ▼
Spawn objects
      │
      ├── local USD
      │
      └── NVIDIA Isaac asset root
      │
      ▼
Scale / position objects
      │
      ▼
Optionally apply physics
      │
      ▼
Control OpenArm
      │
      ▼
Move/remove objects
      │
      ▼
Replace environment
```
This allows OpenArm manipulation scenarios to be assembled interactively without baking every environment and prop into the repository or rebuilding the simulator for every scene change.

