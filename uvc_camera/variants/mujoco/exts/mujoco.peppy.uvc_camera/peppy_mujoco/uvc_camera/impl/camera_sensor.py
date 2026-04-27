from __future__ import annotations

import logging
from typing import Optional

logger = logging.getLogger(__name__)

_DEFAULT_WIDTH: int = 640
_DEFAULT_HEIGHT: int = 480


class MujocoRgbCameraSensor:
    """Off-screen RGB camera sensor backed by mujoco.Renderer."""

    def __init__(
        self,
        model,
        data,
        camera_name: str,
        width: int = _DEFAULT_WIDTH,
        height: int = _DEFAULT_HEIGHT,
    ) -> None:
        self._model = model
        self._data = data
        self._camera_name = camera_name
        self._width = width
        self._height = height
        self._renderer = None
        self._ready: bool = False

    def setup(self) -> bool:
        if self._ready:
            return True
        try:
            import mujoco  # pylint: disable=E0401

            # Renderer(model, height, width) — note: height before width.
            self._renderer = mujoco.Renderer(self._model, self._height, self._width)
            self._ready = True
        except Exception as exc:
            logger.error(
                f"Failed to setup MujocoRgbCameraSensor '{self._camera_name}': {exc}"
            )
            self._renderer = None
            self._ready = False
            return False
        logger.info(
            f"MujocoRgbCameraSensor ready — camera='{self._camera_name}'"
            f" resolution=({self._width}x{self._height})"
        )
        return True

    def teardown(self) -> None:
        self._renderer = None
        self._ready = False

    def get_frame(self) -> Optional[bytes]:
        if not self._ready or self._renderer is None:
            return None
        try:
            self._renderer.update_scene(self._data, camera=self._camera_name)
            pixels = self._renderer.render()  # (height, width, 3) uint8 RGB
            return bytes(pixels.tobytes())
        except Exception as exc:
            logger.warning(
                f"Could not render frame from camera '{self._camera_name}': {exc}"
            )
            return None

    @property
    def width(self) -> int:
        return self._width

    @property
    def height(self) -> int:
        return self._height

    @property
    def is_ready(self) -> bool:
        return self._ready
