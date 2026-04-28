from __future__ import annotations

import logging
from typing import Optional

logger = logging.getLogger(__name__)


class IsaacGripperCommand:
    """Applies gripper position targets to an Isaac Sim articulation.

    finger_joints: ordered list of DOF names matching the desired_position array
    from the move_gripper action goal. Setup fails hard if any name cannot be
    resolved to a DOF in the articulation — no partial configuration is accepted.
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
        seen: set[str] = set()
        for name in self._finger_joints:
            if name in seen:
                logger.error(
                    f"IsaacGripperCommand: duplicate finger joint '{name}' in configuration."
                )
                return False
            seen.add(name)
        try:
            from isaacsim.core.prims import Articulation  # pylint: disable=E0401

            self._articulation = Articulation(prim_paths_expr=self._prim_path)
            self._articulation.initialize()

            dof_names = list(self._articulation.dof_names)
            resolved_names: list[str] = []
            resolved_indices: list[int] = []
            for name in self._finger_joints:
                if name in dof_names:
                    resolved_names.append(name)
                    resolved_indices.append(dof_names.index(name))
                else:
                    logger.warning(
                        f"IsaacGripperCommand: DOF '{name}' not found"
                        f" in articulation '{self._prim_path}'."
                        f" Available: {dof_names}"
                    )

            if len(resolved_indices) != len(self._finger_joints):
                logger.error(
                    f"IsaacGripperCommand: {len(self._finger_joints) - len(resolved_indices)}"
                    f" joint(s) failed to resolve — refusing to start with partial config."
                )
                self._articulation = None
                self._ready = False
                return False

            self._finger_indices = resolved_indices
            self._ready = True
            logger.info(
                f"IsaacGripperCommand ready — prim='{self._prim_path}'"
                f" fingers={resolved_names}"
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
