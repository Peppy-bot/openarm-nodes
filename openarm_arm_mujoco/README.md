# openarm_arm_mujoco

Drives one side of the OpenArm (7 joints, either hardware generation) inside MuJoCo. It conforms to `openarm_arm_sim_passthrough:v1`, relabeling the backbone's governed setpoint stream onto the sim-internal passthrough topic, so backbone and the UI work with it unchanged.

It attaches to the MuJoCo world that `openarm_robot_initializer_mujoco` owns, so that node has to be running first. Targets beyond a joint's physical range are clamped to the model's limits, so the arm always goes as far as it can and the result says so.

## Build

```sh
peppy node add /path/to/ws/openarm_nodes/openarm_arm_mujoco -sb --idle-timeout 1800
```

Re-run with `--force` after code changes. The node shows up at `Stage: Ready` in `peppy stack list` once built.

## Run

One instance drives one side; `arm_id` picks the side (0 = left, 1 = right). Every declared slot must be bound when an instance starts, and this node and the sim consume from each other (the arm reads `arm_states` from the sim, the sim reads the arm's `arm_sim_passthrough`), so the pair can only start through a launcher, which plans and binds all instances together. The [top-level README](../README.md) has the complete sequence:

```sh
peppy stack launch /path/to/ws/launchers_hub/openarm/openarm_teleop_mujoco.json5
```

## Troubleshooting

**Goals time out with "no telemetry from robot_initializer"**
The sim isn't running or hasn't finished loading. Check it with `peppy node info openarm_robot_initializer_mujoco:v1`.

**A move finishes with "reached (target clamped to joint limits)"**
This is informational: the requested angle was beyond the joint's range, so the arm stopped at the limit and reported success there.

**A move fails with "stalled before reaching target"**
The arm physically can't get there, usually because it's colliding with the body or the other arm. Try a different pose.
