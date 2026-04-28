from __future__ import annotations

import logging

logger = logging.getLogger(__name__)

_JOINT_TRANSMISSION_TYPE = 0  # mjTRN_JOINT


class MujocoGripperCommand:
    """Applies gripper position targets to MuJoCo finger actuators.

    finger_joints: ordered list of joint names matching the desired_position array
    from the move_gripper action goal (index 0 → positions[0], etc.).
    Joints not found in the model or without a driving actuator are skipped with a warning.
    """

    def __init__(self, model, data, finger_joints: list[str]) -> None:
        self._model = model
        self._data = data
        self._finger_joints = finger_joints
        self._ctrl_indices: list[int] = []
        self._ready: bool = False

    def setup(self) -> bool:
        try:
            import mujoco  # pylint: disable=E0401

            resolved: list[tuple[str, int]] = []
            for name in self._finger_joints:
                jid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, name)
                if jid < 0:
                    logger.warning(f"MujocoGripperCommand: joint '{name}' not found.")
                    continue

                ctrl_adr = -1
                for act_i in range(self._model.nu):
                    if (
                        self._model.actuator_trntype[act_i] == _JOINT_TRANSMISSION_TYPE
                        and self._model.actuator_trnid[act_i, 0] == jid
                    ):
                        ctrl_adr = act_i
                        break

                if ctrl_adr < 0:
                    logger.warning(
                        f"MujocoGripperCommand: no actuator drives '{name}'."
                    )
                    continue

                resolved.append((name, ctrl_adr))

            if len(resolved) != len(self._finger_joints):
                logger.error(
                    f"MujocoGripperCommand: {len(self._finger_joints) - len(resolved)}"
                    f" joint(s) failed to resolve — refusing to start with partial config."
                )
                self._ready = False
                return False

            self._ctrl_indices = [ctrl for _, ctrl in resolved]
            self._ready = True
            logger.info(
                f"MujocoGripperCommand ready — fingers={[n for n, _ in resolved]}"
            )
        except Exception as exc:
            logger.error(f"Failed to setup MujocoGripperCommand: {exc}")
            return False

        return True

    def teardown(self) -> None:
        self._ctrl_indices = []
        self._ready = False

    def apply(self, positions: list[float]) -> bool:
        if not self._ready:
            return False

        if len(positions) != len(self._ctrl_indices):
            logger.warning(
                f"GripperCommand: expected {len(self._ctrl_indices)} positions,"
                f" got {len(positions)} — dropped."
            )
            return False

        try:
            for ctrl_idx, pos in zip(self._ctrl_indices, positions):
                self._data.ctrl[ctrl_idx] = pos
            return True
        except Exception as exc:
            logger.warning(f"Could not apply gripper command: {exc}")
            return False

    @property
    def is_ready(self) -> bool:
        return self._ready
