from __future__ import annotations

import logging
from typing import Optional

logger = logging.getLogger(__name__)


class IsaacArmActuator:
    """Applies joint position commands to the arm joints via Isaac Sim ArticulationView."""

    def __init__(self, prim_path: str, arm_id: int) -> None:
        self._prim_path = prim_path
        self._arm_id = arm_id
        self._articulation = None
        self._joint_indices: list[int] = []
        self._ready: bool = False

    def setup(self) -> bool:
        if self._ready:
            return True
        try:
            from isaacsim.core.prims import Articulation  # pylint: disable=E0401

            self._articulation = Articulation(prim_paths_expr=self._prim_path)
            self._articulation.initialize()

            # TODO: resolve arm joint indices by name based on arm_id (0=left, 1=right)
            # Joint names should come from config or env, e.g.:
            #   left  → ["openarm_left_joint1", ..., "openarm_left_joint7"]
            #   right → ["openarm_right_joint1", ..., "openarm_right_joint7"]
            dof_names = list(self._articulation.dof_names)
            side = "left" if self._arm_id == 0 else "right"
            self._joint_indices = [
                i for i, name in enumerate(dof_names) if f"openarm_{side}_joint" in name
            ]
            self._ready = True
        except Exception as exc:
            logger.error(f"Failed to setup IsaacArmActuator at '{self._prim_path}': {exc}")
            self._articulation = None
            self._ready = False
            return False

        logger.info(
            f"IsaacArmActuator ready — prim='{self._prim_path}'"
            f" arm_id={self._arm_id} joints={self._joint_indices}"
        )
        return True

    def set_joint_positions(self, positions: list[float]) -> None:
        if not self._ready or self._articulation is None:
            return
        if len(positions) != len(self._joint_indices):
            logger.warning(
                f"IsaacArmActuator: expected {len(self._joint_indices)} positions,"
                f" got {len(positions)}"
            )
            return
        try:
            import numpy as np  # pylint: disable=E0401

            full = self._articulation.get_joint_positions()[0].copy()
            for local_idx, joint_idx in enumerate(self._joint_indices):
                full[joint_idx] = positions[local_idx]
            self._articulation.set_joint_positions(np.array([full]))
        except Exception as exc:
            logger.warning(f"IsaacArmActuator: failed to set joint positions: {exc}")

    def teardown(self) -> None:
        self._articulation = None
        self._joint_indices = []
        self._ready = False

    @property
    def is_ready(self) -> bool:
        return self._ready
