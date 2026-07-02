# openarm_gripper_mujoco

Drives one side of the OpenArm gripper (either hardware generation, selected by `hardware_version`) inside MuJoCo. It conforms to `openarm_gripper:v1`, the same interface the real gripper drivers implement, so backbone and the UI work with it unchanged.

It attaches to the MuJoCo world that `openarm_robot_initializer_mujoco` owns, so that node has to be running first. `move_gripper` takes the jaw opening in meters (0.0 closed, fully open at the generation's jaw width: 0.044 on v1, 0.0697 on v2); the sim maps it onto each finger joint's own travel.

## Build

```sh
peppy node add /path/to/ws/openarm_nodes/openarm_gripper_mujoco -sb --idle-timeout 1800
```

Re-run with `--force` after code changes. The node shows up at `Stage: Ready` in `peppy stack list` once built.

## Run

One instance drives one side. `gripper_id` picks the side (0 = left, 1 = right), `-i` names the instance, and `--bind` points the node at the sim instance it should attach to:

```sh
peppy node run openarm_robot_initializer_mujoco:v1 -i sim
peppy node run openarm_gripper_mujoco:v1 gripper_id=0 hardware_version=v1 control_rate_hz=100 stream_timeout_s=0.5 -i left_grip_inst --bind sim@sim
peppy node run openarm_gripper_mujoco:v1 gripper_id=1 hardware_version=v1 control_rate_hz=100 stream_timeout_s=0.5 -i right_grip_inst --bind sim@sim
```

For the full stack, with the browser UI driving both arms and grippers, use the launcher instead; the [top-level README](../README.md) has the complete sequence:

```sh
peppy stack launch /path/to/ws/launchers_hub/openarm/openarm_teleop_mujoco.json5
```

## Troubleshooting

**Goals time out with "no usable telemetry from robot_initializer"**
The sim isn't running or hasn't finished loading. Check it with `peppy node info openarm_robot_initializer_mujoco:v1`.

**A move finishes with "stalled at physical limit"**
That's success: the fingers closed onto something (or hit fully open) before reaching the exact target, which is exactly how gripping is supposed to work.

**A goal is rejected with "position out of range"**
`move_gripper` accepts 0.0 to 0.044 meters of total aperture.
