# Robot description

`openarm_v10.urdf` is the OpenArm V1.0 description the arm and pose-tester nodes
load at runtime to build the `srs_model` kinematics/dynamics in-process (gravity
and Coriolis in the arm, IK in the tester). Point each node's `urdf_path` here and
select the side with `base_link` (`openarm_left_link0` / `openarm_right_link0`).

It is a flat, self-contained URDF (no xacro) that includes the
`world -> openarm_body -> {left,right}_link0` mount tree, so gravity resolves in
the world frame. Structurally identical to enactic's
`openarm_description` V1.0 example, plus the parallel-gripper prismatic fingers
so the distal-payload path is exercised in production, not just in tests.

## Gripper mass

The `hand_tcp` links carry the parallel-gripper **hand-body inertial** (0.127 kg,
COM at z ≈ 0.102 m from link7), taken from enactic `openarm_description`
`assets/end_effector/parallel_link/config/inertials.yaml`, alongside the two 36 g
fingers. `srs_model` lumps everything past the wrist into the distal payload
(`Payload::from_distal` walks every descendant of the wrist), so the full ~0.2 kg
gripper is in the gravity/Coriolis feedforward.

This matches the **parallel-link (2-finger) gripper this robot uses**, not the
heavier single-body `openarm_hand` (0.35 kg) the openarm_teleop launch scripts
target, which is a different end effector. The arm holds the wrist with a position
setpoint plus this feedforward, so the gripper-body mass is compensated and the
wrist holds without sag.
