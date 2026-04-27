from __future__ import annotations

import logging
from typing import Optional

logger = logging.getLogger(__name__)


class MujocoImuSensor:
    """Reads IMU data from a MuJoCo body."""

    def __init__(self, model, data, body_name: str) -> None:
        self._model = model
        self._data = data
        self._body_name = body_name
        self._body_id: Optional[int] = None
        self._ready: bool = False

    def setup(self) -> bool:
        """Resolve body ID from the MuJoCo model."""
        try:
            import mujoco  # pylint: disable=E0401

            body_id = mujoco.mj_name2id(
                self._model, mujoco.mjtObj.mjOBJ_BODY, self._body_name
            )
            if body_id < 0:
                logger.error(f"IMU body '{self._body_name}' not found in model.")
                return False
            self._body_id = body_id
            self._ready = True
        except Exception as exc:
            logger.error(f"Failed to setup MujocoImuSensor: {exc}")
            return False

        logger.info(
            f"MujocoImuSensor ready — body='{self._body_name}' id={self._body_id}"
        )
        return True

    def teardown(self) -> None:
        """Reset sensor state."""
        self._ready = False
        self._body_id = None

    def get_imu_data(self) -> Optional[dict]:
        """Read orientation (wxyz), angular velocity, and linear acceleration."""
        if not self._ready:
            return None

        try:
            import numpy as np  # pylint: disable=E0401

            xquat = self._data.xquat[self._body_id]
            # xmat is row-major 3x3 rotation matrix (world←body); transpose gives body←world
            xmat = self._data.xmat[self._body_id].reshape(3, 3)
            cvel = self._data.cvel[self._body_id]
            # cacc requires mj_rnePostConstraint to be populated — backbone must call it
            # after mj_step if accurate linear acceleration is needed.
            cacc = self._data.cacc[self._body_id]

            angular_velocity_body = xmat.T @ cvel[:3]
            linear_acceleration_body = xmat.T @ cacc[3:]

            return {
                "orientation": [float(xquat[i]) for i in range(4)],
                "angular_velocity": [float(v) for v in angular_velocity_body],
                "linear_acceleration": [float(v) for v in linear_acceleration_body],
            }
        except Exception as exc:
            logger.warning(f"Could not read IMU data: {exc}")
            return None

    @property
    def is_ready(self) -> bool:
        """True when body ID has been resolved."""
        return self._ready
