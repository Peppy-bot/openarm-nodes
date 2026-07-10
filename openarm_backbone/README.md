# openarm_backbone

Routes operator commands to the right place. The commander fires `move_arm_joints` and `move_gripper` goals at this node; backbone reads the goal's `arm_id` or `gripper_id` (0 = left, 1 = right), forwards it to the matching arm or gripper instance, and streams the feedback and result back to the caller.

It is engine-agnostic. The launcher binds one robot_initializer, two arms, and two grippers into its five slots, and those can be real, Isaac, or MuJoCo implementations. At startup it waits on the robot_initializer's `is_ready` before accepting any goals, so nothing moves until the world is actually loaded.

## Build

```sh
peppy node add /path/to/ws/openarm_nodes/openarm_backbone -sb --idle-timeout 1800
```

Re-run with `--force` after code changes. The node shows up at `Stage: Ready` in `peppy stack list` once built.

## Run

Backbone needs all five of its slots bound to do anything useful, so run it through a launcher rather than by hand. The [top-level README](../README.md) has the complete build-and-launch sequence:

```sh
peppy stack launch /path/to/ws/launchers_hub/openarm/openarm_teleop_mujoco.json5
```

After launch, watch it route goals live:

```sh
peppy node info openarm_backbone:v1
```

## Actions

```
move_arm_joints   goal: { arm_id, feedback_frequency, joint_positions[7] }
move_gripper      goal: { gripper_id, feedback_frequency, position }
```

`position` is the gripper's total aperture in meters (0.0 closed, 0.044 fully open).

## Troubleshooting

**Backbone never gets past startup**
It's waiting on the robot_initializer's `is_ready`. Check the sim actually loaded with `peppy node info openarm_robot_initializer_<engine>:v1`.

**Goals are rejected immediately**
One of its slots isn't bound, usually because a launcher binding points at an instance that isn't running. Compare the launcher's `bindings` block against `peppy stack list`.

**A goal is accepted but never completes**
The downstream arm or gripper is unhealthy. Check its log with `peppy node info openarm_arm_<engine>:v1`.
