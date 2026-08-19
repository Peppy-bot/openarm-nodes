# openarm Nodes

[Peppy](https://github.com/Peppy-bot/peppy) nodes for the OpenArm bimanual robot (v1.0 and v2.0). The full stack lets you drive two 7-DOF arms and two grippers from your browser, against the real robot, Isaac Sim, or MuJoCo. The nodes and the UI stay the same; only the launcher changes.

| Component | What it does |
|---|---|
| [`openarm_robot_initializer`](./openarm_robot_initializer) | aggregates per-limb readiness into `is_ready` |
| [`openarm_arm`](./openarm_arm) | drives one arm side (7 joints) |
| [`openarm_gripper`](./openarm_gripper) | drives one gripper side (v1.0 prismatic or v2.0 pinch, by `hardware_version`) |
| [`openarm_arm_sim`](./openarm_arm_sim) | relays one arm side between the backbone and a sim engine |
| [`openarm_gripper_sim`](./openarm_gripper_sim) | relays one gripper side between the backbone and a sim engine |
| [`openarm_sim_mujoco`](./openarm_sim_mujoco) | MuJoCo engine: the physics behind the relays |
| [`openarm_sim_isaac`](./openarm_sim_isaac) | Isaac Sim engine: the physics behind the relays |
| [`openarm_backbone`](./openarm_backbone) | routes goals to the correct side |
| [`openarm_commander`](./openarm_commander) | browser control panel |
| [`openarm_ker`](./openarm_ker) | streams joint setpoints from a physical leader arm |

Sim support splits into engine-agnostic relays plus one node per engine: `openarm_arm_sim` and `openarm_gripper_sim` face the backbone exactly like the real nodes and lead the matching limb slot on the engine node (`openarm_sim_mujoco` or `openarm_sim_isaac`), which owns the physics and models v1.0 or v2.0 hardware via its `hardware_version` parameter. The launcher decides which nodes fill each slot, so the backbone and the UI never know which engine is underneath.

This guide takes you from a fresh machine to a moving arm. MuJoCo is the quickest way to see everything working.

## 1. Prerequisites

- Ubuntu 22.04 or 24.04
- [Peppy](https://peppy.bot) 0.16 or newer, installed with `curl -fsSL https://peppy.bot/install.sh | sh`
- Docker, running
- For Isaac only: an NVIDIA GPU with the [Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) configured

Clone this repo together with [contracts-hub](https://github.com/Peppy-bot/contracts-hub) (the contracts) and [launchers-hub](https://github.com/Peppy-bot/launchers-hub) (the stack launchers) so the paths below line up:

```text
ws/
├── contracts-hub/
├── launchers-hub/
└── openarm-nodes/
```

## 2. Start the daemon and register the repos

The daemon builds, runs, and connects every node. Registering the repos is what lets it resolve nodes and contracts by name. The launcher depends on this, so don't skip it on a fresh machine.

```sh
peppy service serve &

peppy repo add /path/to/ws/contracts-hub
peppy repo add /path/to/ws/openarm-nodes
peppy repo add /path/to/ws/launchers-hub
peppy repo refresh
```

`peppy repo refresh` walks the registered repos and ends with a summary like `Repository refresh complete. N node(s), M contract(s) found.` You can double-check what got registered with `peppy repo list`.

## 3. Build the nodes

Each `peppy node add <path> -sb` registers the node in the stack, generates its API code from the manifest and contracts, and builds its container. The first sim engine build also pulls its base image (about 1 GB for MuJoCo and 7.5 GB for Isaac), so it gets a much larger idle timeout than the rest; without it the daemon kills the build mid-download.

MuJoCo stack:

```sh
peppy node add /path/to/ws/openarm-nodes/openarm_sim_mujoco -sb --idle-timeout 18000
peppy node add /path/to/ws/openarm-nodes/openarm_arm_sim -sb --idle-timeout 1800
peppy node add /path/to/ws/openarm-nodes/openarm_gripper_sim -sb --idle-timeout 1800
peppy node add /path/to/ws/openarm-nodes/openarm_robot_initializer -sb --idle-timeout 1800
peppy node add /path/to/ws/openarm-nodes/openarm_backbone -sb --idle-timeout 1800
peppy node add /path/to/ws/openarm-nodes/openarm_commander -sb --idle-timeout 1800
```

For Isaac, swap the engine node; the relays, initializer, backbone, and commander are engine-agnostic and don't need rebuilding:

```sh
peppy node add /path/to/ws/openarm-nodes/openarm_sim_isaac -sb --idle-timeout 18000
```

The `sim_cameras` launcher option, on either engine, additionally needs the camera relay nodes, which live in the separate [nodes-hub](https://github.com/Peppy-bot/nodes-hub) repo because nothing in them is OpenArm-specific:

```sh
peppy node add /path/to/ws/nodes-hub/sim_rgb_camera -sb --idle-timeout 1800
peppy node add /path/to/ws/nodes-hub/sim_rgbd_camera -sb --idle-timeout 1800
```

Real robot:

```sh
peppy node add /path/to/ws/openarm-nodes/openarm_robot_initializer -sb --idle-timeout 1800
peppy node add /path/to/ws/openarm-nodes/openarm_arm -sb --idle-timeout 1800
peppy node add /path/to/ws/openarm-nodes/openarm_gripper -sb --idle-timeout 1800
```

Both hardware generations run the same nodes; the launcher's `hardware_version` argument selects which one each arm and gripper drives.

Upgrading a v2.0 rig from `openarm_gripper_v2`: stop and remove any gripper instance running that node before launching. Both nodes drive the same motor id on the same bus, but the old one holds a different instance lock, so nothing would stop the two from commanding one gripper at once.

After changing a node's code, rebuild it by re-running the same command with `--force` added.

Now verify everything built:

```sh
peppy stack list
```

Every node you added should show `Stage: Ready`. If one is stuck at an earlier stage, jump to Troubleshooting.

## 4. Launch the stack

The `--with=` selection names the engine, so pick the one matching the nodes built above:

```sh
# MuJoCo
peppy stack launch openarm_v2 --with=mujoco
# Isaac
peppy stack launch openarm_v2 --with=isaac_sim
```

The launcher starts the instances in dependency order (sim first, then arms and grippers, then backbone, then the UI) and wires them together. Once it prints `Launch complete`:

- open **http://localhost:8765** for the control panel, one slider per joint
- MuJoCo: open **http://localhost:8080** for the browser viewer
- Isaac: connect with the [livestream client](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/manual_livestream_clients.html)

Move a slider, press **Send**, and watch the arm follow in the viewer. The launchers themselves are documented in [launchers-hub/openarm](https://github.com/Peppy-bot/launchers-hub/tree/main/openarm). Check the stack's health any time:

```sh
peppy stack list
```

Every instance should be `running` and `healthy`. To stop everything, Ctrl-C the launch terminal, or stop instances individually:

```sh
peppy node stop commander_inst
```

## Troubleshooting

**`repo-node 'X:v1' not found in nodes.json5` when launching**
The repo that provides X was never registered with the daemon. Run the `peppy repo add` lines from step 2 followed by `peppy repo refresh`, then launch again.

**The sim engine build dies partway through**
The base image download outlived the daemon's idle timeout. Re-run the add with `--idle-timeout 18000`. The Isaac image is large and the first build genuinely takes a while; later builds reuse the cached image and finish quickly.

**A node won't reach `Stage: Ready`**
Rebuild it and read the build log peppy prints on failure:
```sh
peppy node add /path/to/ws/openarm-nodes/<node> -sb --force --idle-timeout 1800
```

**The stack launches but the arms don't respond**
The sim keeps loading after `Launch complete`, and Isaac can take a minute. Watch its log until the world is up:

```sh
peppy node info openarm_sim_mujoco:v1   # or openarm_sim_isaac:v1
```

**A move finishes with "reached (target clamped to joint limits)"**
Not an error. The requested angle was beyond that joint's physical range, so the arm went as far as the model allows and reported success there.

**The Isaac stream is a black screen**
Stop the stack, clear the shader cache with `rm -rf ~/.cache/isaac-sim`, and launch again.

**Port 8765 or 8080 is already in use**
An older instance is still running. Find it with `peppy stack list` and stop it with `peppy node stop <instance_id>`.

## Adding an item to this repository

This repository publishes what `peppy_repository.json5` says it publishes, and nothing else. An item
that is not listed there is invisible to peppy, so after adding, moving, or renaming a node, run:

```sh
peppy repo index .
```

Commit the updated `peppy_repository.json5` alongside your change. CI runs `peppy repo index --check`
on every pull request and fails if the index has drifted from the repository, naming the file and the
identity involved.

Generation refuses, naming both files, if your change claims a `name:tag` another one already
publishes. Rename yours: within one repository, a `name:tag` is claimed by exactly one file.
