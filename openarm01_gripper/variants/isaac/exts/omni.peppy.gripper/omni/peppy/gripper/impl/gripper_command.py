from __future__ import annotations

import logging
from typing import Optional

logger = logging.getLogger(__name__)


class IsaacGripperCommand:
    """Applies gripper position targets to an Isaac Sim articulation.

    finger_joints: ordered list of DOF names matching the desired_position array
    from the move_gripper action goal. Names not found in the articulation are
    skipped with a warning at setup time.
    """

    def __init__(self, prim_path: str, finger_joints: list[str]) -> None:
        self._prim_path = prim_path
        self._finger_joints = finger_joints
        self._articulation = None
        self._finger_indices: list[int] = []
        self._ready: bool = False

    def setup(self) -> bool:
        if self._articulation is not None and self._ready:
            return True
        try:
            from isaacsim.core.prims import Articulation  # pylint: disable=E0401

            self._articulation = Articulation(prim_paths_expr=self._prim_path)
            self._articulation.initialize()

            dof_names = list(self._articulation.dof_names)
            self._finger_indices = []
            for name in self._finger_joints:
                if name in dof_names:
                    self._finger_indices.append(dof_names.index(name))
                else:
                    logger.warning(
                        f"IsaacGripperCommand: DOF '{name}' not found"
                        f" in articulation '{self._prim_path}'."
                        f" Available: {dof_names}"
                    )

            self._ready = True
            logger.info(
                f"IsaacGripperCommand ready — prim='{self._prim_path}'"
                f" fingers={[self._finger_joints[i] for i in range(len(self._finger_indices))]}"
            )
        except Exception as exc:
            logger.error(
                f"Failed to setup IsaacGripperCommand at '{self._prim_path}': {exc}"
            )
            self._articulation = None
            self._ready = False
            return False

        return True

    def teardown(self) -> None:
        self._articulation = None
        self._finger_indices = []
        self._ready = False

    def apply(self, positions: list[float]) -> bool:
        if not self._ready or self._articulation is None:
            return False

        if len(positions) != len(self._finger_indices):
            logger.warning(
                f"GripperCommand: expected {len(self._finger_indices)} positions,"
                f" got {len(positions)} — dropped."
            )
            return False

        try:
            import numpy as np  # pylint: disable=E0401

            all_positions = self._articulation.get_joint_positions()[0].copy()
            for list_idx, dof_idx in enumerate(self._finger_indices):
                all_positions[dof_idx] = positions[list_idx]
            self._articulation.set_joint_positions(
                np.array([all_positions])
            )
            return True
        except Exception as exc:
            logger.warning(f"Could not apply gripper command: {exc}")
            return False

    @property
    def is_ready(self) -> bool:
        return self._ready
