# openarm_gripper_isaac

Drives one side of the OpenArm gripper (either hardware generation, selected by `hardware_version`) inside Isaac Sim. It implements `openarm_gripper_sim_passthrough:v1`, exposing the resolved sim-internal command stream while consuming the shared gripper-state contract.

It attaches to the Isaac world that `openarm_robot_initializer_isaac` owns, so that node has to be running first. `move_gripper` takes the jaw opening in meters (0.0 closed, fully open at the generation's jaw width: 0.044 on v1, 0.0697 on v2); the sim maps it onto each finger joint's own travel.

## Build

```sh
peppy node add /path/to/ws/openarm-nodes/openarm_gripper_isaac -sb --idle-timeout 1800
```

Re-run with `--force` after code changes. The node shows up at `Stage: Ready` in `peppy stack list` once built.

## Run

One instance drives one side; `gripper_id` picks the side (0 = left, 1 = right). The `engine_states` slot must be bound when an instance starts (the `backbone` pairing is optional; the follower idles until the backbone pairs it), and this node and the sim consume from each other (the gripper reads `gripper_states` from the sim, the sim reads the gripper's `gripper_sim_passthrough`), so the pair can only start through a launcher, which plans and binds all instances together. The [top-level README](../README.md) has the complete sequence:

```sh
peppy stack launch /path/to/ws/launchers-hub/openarm/openarm_v2_teleop_isaac.json5
```

## Troubleshooting

**Goals time out with "no usable telemetry from robot_initializer"**
Isaac isn't up yet; it keeps loading for a while after starting. Watch it with `peppy node info openarm_robot_initializer_isaac:v1`.

**A move finishes with "stalled at physical limit"**
That's success: the fingers closed onto something (or hit fully open) before reaching the exact target, which is exactly how gripping is supposed to work.

**A goal is rejected with "position out of range"**
`move_gripper` accepts 0.0 up to the instance's jaw width: 0.044 meters on v1, 0.0697 meters on v2.
