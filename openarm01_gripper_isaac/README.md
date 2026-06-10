# openarm01_gripper_isaac

Drives one side of the OpenArm V10 gripper inside Isaac Sim. It conforms to `openarm01_gripper:v1`, the same interface the real gripper driver implements, so backbone and the UI work with it unchanged.

It attaches to the Isaac world that `openarm01_robot_initializer_isaac` owns, so that node has to be running first. `move_gripper` takes the total aperture in meters (0.0 closed, 0.044 fully open); each finger moves to half of it.

## Build

```sh
peppy node add /path/to/ws/openarm01_nodes/openarm01_gripper_isaac -sb --idle-timeout 1800
```

Re-run with `--force` after code changes. The node shows up at `Stage: Ready` in `peppy stack list` once built.

## Run

One instance drives one side. `gripper_id` picks the side (0 = left, 1 = right), `-i` names the instance, and `--bind` points the node at the sim instance it should attach to:

```sh
peppy node run openarm01_robot_initializer_isaac:v1 -i sim
peppy node run openarm01_gripper_isaac:v1 gripper_id=0 -i left_grip_inst --bind sim@sim
peppy node run openarm01_gripper_isaac:v1 gripper_id=1 -i right_grip_inst --bind sim@sim
```

For the full stack, with the browser UI driving both arms and grippers, use the launcher instead; the [top-level README](../README.md) has the complete sequence:

```sh
peppy stack launch /path/to/ws/launchers_hub/openarm01/openarm01_teleop_isaac.json5
```

## Troubleshooting

**Goals time out with "no usable telemetry from robot_initializer"**
Isaac isn't up yet; it keeps loading for a while after starting. Watch it with `peppy node info openarm01_robot_initializer_isaac:v1`.

**A move finishes with "stalled at physical limit"**
That's success: the fingers closed onto something (or hit fully open) before reaching the exact target, which is exactly how gripping is supposed to work.

**A goal is rejected with "position out of range"**
`move_gripper` accepts 0.0 to 0.044 meters of total aperture.
