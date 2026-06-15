# openarm01_arm_isaac

Drives one side of the OpenArm V10 (7 joints) inside Isaac Sim. It conforms to `openarm01_arm:v1`, the same interface the real arm driver implements, so backbone and the UI work with it unchanged.

It attaches to the Isaac world that `openarm01_robot_initializer_isaac` owns, so that node has to be running first. Targets beyond a joint's physical range are clamped to the model's limits, so the arm always goes as far as it can and the result says so.

## Build

```sh
peppy node add /path/to/ws/openarm01_nodes/openarm01_arm_isaac -sb --idle-timeout 1800
```

Re-run with `--force` after code changes. The node shows up at `Stage: Ready` in `peppy stack list` once built.

## Run

One instance drives one side. `arm_id` picks the side (0 = left, 1 = right), `-i` names the instance, and `--bind` points the node at the sim instance it should attach to:

```sh
peppy node run openarm01_robot_initializer_isaac:v1 -i sim
peppy node run openarm01_arm_isaac:v1 arm_id=0 -i left_arm_inst --bind sim@sim
peppy node run openarm01_arm_isaac:v1 arm_id=1 -i right_arm_inst --bind sim@sim
```

For the full stack, with the browser UI driving both arms and grippers, use the launcher instead; the [top-level README](../README.md) has the complete sequence:

```sh
peppy stack launch /path/to/ws/launchers_hub/openarm01/openarm01_teleop_isaac.json5
```

## Troubleshooting

**Goals time out with "no telemetry from robot_initializer"**
Isaac isn't up yet; it keeps loading for a while after starting. Watch it with `peppy node info openarm01_robot_initializer_isaac:v1`.

**A move finishes with "reached (target clamped to joint limits)"**
This is informational: the requested angle was beyond the joint's range, so the arm stopped at the limit and reported success there.

**A move fails with "stalled before reaching target"**
The arm physically can't get there, usually because it's colliding with the body or the other arm. Try a different pose.
