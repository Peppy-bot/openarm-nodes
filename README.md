# openarm Nodes

[Peppy](https://github.com/Peppy-bot/peppy) nodes for the OpenArm bimanual robot (v1.0 and v2.0). The full stack lets you drive two 7-DOF arms and two grippers from your browser, against the real robot, Isaac Sim, or MuJoCo. The nodes and the UI stay the same; only the launcher changes.

| Component | What it does |
|---|---|
| [`openarm_robot_initializer`](./openarm_robot_initializer) | loads the sim world and reports `is_ready` |
| [`openarm_arm`](./openarm_arm) | drives one arm side (7 joints) |
| [`openarm_gripper`](./openarm_gripper) | drives one gripper side (v1.0 prismatic) |
| [`openarm_gripper_v2`](./openarm_gripper_v2) | drives one gripper side (v2.0 pinch) |
| [`openarm_backbone`](./openarm_backbone) | routes goals to the correct side |
| [`openarm_commander`](./openarm_commander) | browser control panel |

Each sim-capable component comes in three flavours: the real-hardware node plus `_isaac` and `_mujoco` siblings (for example [`openarm_arm_isaac`](./openarm_arm_isaac) and [`openarm_arm_mujoco`](./openarm_arm_mujoco)), all conforming to the same interface. The two real gripper nodes share the same sim siblings, which pick the modeled gripper via a `hardware_version` parameter (`"v1"` or `"v2"`). The launcher decides which flavour fills each slot, so backbone and the UI never know which engine is underneath.

This guide takes you from a fresh machine to a moving arm. MuJoCo is the quickest way to see everything working.

## 1. Prerequisites

- Ubuntu 22.04 or 24.04
- [Peppy](https://peppy.bot) 0.10 or newer, installed with `curl -fsSL https://peppy.bot/install.sh | sh`
- Docker, running
- For Isaac only: an NVIDIA GPU with the [Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) configured

Clone this repo together with [interfaces_hub](https://github.com/Peppy-bot/interfaces_hub) (the interface contracts) and [launchers_hub](https://github.com/Peppy-bot/launchers_hub) (the stack launchers) so the paths below line up:

```text
ws/
├── interfaces_hub/
├── launchers_hub/
└── openarm_nodes/
```

## 2. Start the daemon and register the repos

The daemon builds, runs, and connects every node. Registering the repos is what lets it resolve nodes and interfaces by name. The launcher depends on this, so don't skip it on a fresh machine.

```sh
peppy service serve &

peppy repo add /path/to/ws/interfaces_hub
peppy repo add /path/to/ws/openarm_nodes
peppy repo refresh
```

`peppy repo refresh` walks the registered repos and ends with a summary like `Repository refresh complete. N node(s), M interface(s) found.` You can double-check what got registered with `peppy repo list`.

## 3. Build the nodes

Each `peppy node add <path> -sb` registers the node in the stack, generates its interface code, and builds its container. The first robot_initializer build also pulls the sim base image (about 1 GB for MuJoCo and 7.5 GB for Isaac), so it gets a much larger idle timeout than the rest; without it the daemon kills the build mid-download.

MuJoCo stack:

```sh
peppy node add /path/to/ws/openarm_nodes/openarm_robot_initializer_mujoco -sb --idle-timeout 18000
peppy node add /path/to/ws/openarm_nodes/openarm_arm_mujoco -sb --idle-timeout 1800
peppy node add /path/to/ws/openarm_nodes/openarm_gripper_mujoco -sb --idle-timeout 1800
peppy node add /path/to/ws/openarm_nodes/openarm_backbone -sb --idle-timeout 1800
peppy node add /path/to/ws/openarm_nodes/openarm_commander -sb --idle-timeout 1800
```

For Isaac, swap the three sim-specific nodes. Backbone and joint_commander are engine-agnostic and don't need rebuilding:

```sh
peppy node add /path/to/ws/openarm_nodes/openarm_robot_initializer_isaac -sb --idle-timeout 18000
peppy node add /path/to/ws/openarm_nodes/openarm_arm_isaac -sb --idle-timeout 1800
peppy node add /path/to/ws/openarm_nodes/openarm_gripper_isaac -sb --idle-timeout 1800
```

Real robot:

```sh
peppy node add /path/to/ws/openarm_nodes/openarm_robot_initializer -sb --idle-timeout 1800
peppy node add /path/to/ws/openarm_nodes/openarm_arm -sb --idle-timeout 1800
peppy node add /path/to/ws/openarm_nodes/openarm_gripper -sb --idle-timeout 1800
```

On v2.0 hardware, build `openarm_gripper_v2` in place of `openarm_gripper`.

After changing a node's code, rebuild it by re-running the same command with `--force` added.

Now verify everything built:

```sh
peppy stack list
```

Every node you added should show `Stage: Ready`. If one is stuck at an earlier stage, jump to Troubleshooting.

## 4. Launch the stack

```sh
peppy stack launch /path/to/ws/launchers_hub/openarm/openarm_teleop_mujoco.json5
```

The launcher starts all eight instances in dependency order (sim first, then arms and grippers, then backbone, then the UI) and wires them together. Once it prints `Launch complete`:

- open **http://localhost:8765** for the control panel, one slider per joint
- MuJoCo: open **http://localhost:8080** for the browser viewer
- Isaac: connect with the [livestream client](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/manual_livestream_clients.html)

Move a slider, press **Send**, and watch the arm follow in the viewer. The launchers themselves are documented in [launchers_hub/openarm](https://github.com/Peppy-bot/launchers_hub/tree/main/openarm). Check the stack's health any time:

```sh
peppy stack list
```

You should see 8 instances, all `running` and `healthy`. To stop everything, Ctrl-C the launch terminal, or stop instances individually:

```sh
peppy node stop commander
```

## Troubleshooting

**`repo-node 'X:v1' not found in nodes.json5` when launching**
The repo that provides X was never registered with the daemon. Run the `peppy repo add` lines from step 2 followed by `peppy repo refresh`, then launch again.

**The robot_initializer build dies partway through**
The base image download outlived the daemon's idle timeout. Re-run the add with `--idle-timeout 18000`. The Isaac image is large and the first build genuinely takes a while; later builds reuse the cached image and finish quickly.

**A node won't reach `Stage: Ready`**
Rebuild it and read the build log peppy prints on failure:
```sh
peppy node add /path/to/ws/openarm_nodes/<node> -sb --force --idle-timeout 1800
```

**The stack launches but the arms don't respond**
The sim keeps loading after `Launch complete`, and Isaac can take a minute. Watch its log until the world is up:

```sh
peppy node info openarm_robot_initializer_mujoco:v1
```

**A move finishes with "reached (target clamped to joint limits)"**
Not an error. The requested angle was beyond that joint's physical range, so the arm went as far as the model allows and reported success there.

**The Isaac stream is a black screen**
Stop the stack, clear the shader cache with `rm -rf ~/.cache/isaac-sim`, and launch again.

**Port 8765 or 8080 is already in use**
An older instance is still running. Find it with `peppy stack list` and stop it with `peppy node stop <instance_id>`.
