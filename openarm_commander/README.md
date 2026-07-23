# openarm_commander

The browser control panel for the OpenArm (either hardware generation). It serves a page on port 8765 with a slider per joint and per gripper; pressing **Send** fires the matching `move_arm_joints` or `move_gripper` goal at `openarm_backbone`, and feedback streams back into the page while the arm moves.

Slider ranges come from the generation's description (`hardware_version` parameter): arm limits parse from the bundled URDF with the elbow held off its singularity floor, and the gripper spans the generation's jaw width. The UI won't let you ask for an angle the arm can't physically reach.

## Build

```sh
peppy node add /path/to/ws/openarm-nodes/openarm_commander -sb --idle-timeout 1800
```

Re-run with `--force` after code changes. The node shows up at `Stage: Ready` in `peppy stack list` once built.

## Run

It needs a running backbone, so the usual way is through a launcher; the [top-level README](../README.md) has the complete sequence:

```sh
peppy stack launch /path/to/ws/launchers-hub/openarm/openarm_v2_teleop_mujoco.json5
```

You can also run it alone against an already-running stack. Every declared slot must be linked at start: the `backbone` slot to the backbone instance, and each per-side state slot to that side's arm or gripper instance (sim followers here, the drivers on real hardware). The required parameters have no defaults, so they are supplied too:

```sh
peppy node run openarm_commander:v1 \
    command_rate_hz=100 hardware_version=v2 max_ee_velocity_m_s=0.5 \
    collision_governor_enabled=true d_stop=0.005 d_safe=0.02 \
    --link backbone@backbone_inst \
    --link left_arm_states@left_arm_inst \
    --link right_arm_states@right_arm_inst \
    --link left_gripper_states@left_grip_inst \
    --link right_gripper_states@right_grip_inst
```

Then open **http://localhost:8765**. Each arm panel has 7 sliders: **Send** fires the goal and **Home** resets the sliders to zero. The gripper slider runs from closed (0.0) to the generation's full jaw width (0.044 m on v1, 0.0697 m on v2), with **Open** and **Close** shortcuts. The page reconnects automatically if the node restarts, and the port can be changed with `PEPPY_JC_PORT`.

## Troubleshooting

**The page loads but Send does nothing**
Backbone isn't up or isn't healthy. Check `peppy stack list`, then `peppy node info openarm_backbone:v1`.

**The status line says "previous goal still in flight"**
Each arm and gripper takes one goal at a time, so wait for the badge to flip back to idle before sending again.

**Port 8765 is already in use**
A previous instance is still running. Find it with `peppy stack list` and stop it with `peppy node stop <instance_id>`, or set `PEPPY_JC_PORT` to a different port.
