# openarm_robot_initializer

This node owns the world the OpenArm lives in. It loads the simulation (or does nothing, on the real robot) and exposes an `is_ready` service that the rest of the stack polls, so nothing tries to command an arm until this node says the world is actually up.

Three siblings conform to the same `openarm_robot_initializer:v1` contract, and the launcher picks one:

| Node | Use case |
|---|---|
| `openarm_robot_initializer` | real robot: nothing to load, `is_ready` is true immediately |
| `openarm_robot_initializer_mujoco` | MuJoCo: ready once the model is loaded |
| `openarm_robot_initializer_isaac` | Isaac Sim: ready once the stage finishes warming up |

Robot assets (USD, MJCF, meshes) are baked into the sim base images, so there is nothing to download or mount yourself.

## Build

The first build pulls the sim base image (about 1 GB for MuJoCo, 7.5 GB for Isaac), so give it a generous idle timeout or the daemon will kill the build mid-download:

```sh
peppy node add /path/to/ws/openarm_nodes/openarm_robot_initializer_mujoco -sb --idle-timeout 18000
```

Swap in `openarm_robot_initializer_isaac` or `openarm_robot_initializer` (real) as needed. Rebuild after code changes by re-running with `--force`. When the build finishes, `peppy stack list` shows the node at `Stage: Ready`.

## Run

Every declared slot must be bound when an instance starts, and the sim variants and the arm/gripper bridge nodes consume from each other (the sim reads their passthrough streams on `arm_cmd` / `gripper_cmd`, the bridges read the sim's state streams), so the stack can only start through a launcher, which plans and binds all instances together. The same goes for the real variant, whose four `hardware_ready` slots the launcher binds to the driver instances. The launchers in [launchers_hub/openarm](https://github.com/Peppy-bot/launchers_hub/tree/main/openarm) do exactly that; the [top-level README](../README.md) walks through the whole sequence:

```sh
peppy stack launch /path/to/ws/launchers_hub/openarm/openarm_teleop_mujoco.json5
```

MuJoCo runs headless and renders to your browser at **http://localhost:8080**. Isaac runs headless and streams over WebRTC; connect with the [Isaac Sim livestream client](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/manual_livestream_clients.html). If you want a native window on the same machine instead, launch with `PEPPY_BRIDGE_HEADLESS=0` set in the environment.

Watch it come up with:

```sh
peppy node info openarm_robot_initializer_mujoco:v1
```

## Troubleshooting

**The first build times out**
That's the base image pull outliving the daemon's idle timeout. Re-run with `--idle-timeout 18000`; later builds reuse the cached image and finish quickly.

**`is_ready` never becomes true**
The world failed to load. Read the log; the load error is usually near the top:
```sh
peppy node info openarm_robot_initializer_mujoco:v1
```

**The Isaac stream is a black screen**
Stop the node, clear the shader cache with `rm -rf ~/.cache/isaac-sim`, and start it again.
