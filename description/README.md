# Robot description

`openarm_v10.urdf` is the OpenArm V1.0 description the `srs_model` compensation
instances load at runtime. Point each instance's `urdf_path` here and select the
side with `base_link` (`openarm_left_link0` / `openarm_right_link0`).

It is a flat, self-contained URDF (no xacro) that includes the
`world -> openarm_body -> {left,right}_link0` mount tree, so gravity resolves in
the world frame. Structurally identical to enactic's
`openarm_description` V1.0 example, plus the parallel-gripper prismatic fingers
so the distal-payload path is exercised in production, not just in tests.

## Gripper mass is incomplete (known)

The enactic V1.0 description models the gripper as **two 36 g fingers only**; the
hand/`base_link` body has visual and collision geometry but **no inertial**, so
its mass is zero in every URDF (this one, the test fixture, and the upstream
example). `srs_model` lumps the fingers (~72 g) into the wrist, which is already
more than openarm_teleop compensates (its serial `body_link0 -> hand` chain
excludes the branched fingers), but neither models the real gripper-body mass
(motor, housing). In float mode (kp = kd = 0) any unmodeled distal mass shows up
as wrist sag. If that sag matters, add a measured `<inertial>` to the hand body
here; do not guess the value.
