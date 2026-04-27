from __future__ import annotations

import logging
from typing import Optional

logger = logging.getLogger(__name__)

_DEFAULT_WIDTH: int = 640
_DEFAULT_HEIGHT: int = 480
_DEFAULT_FX: float = 617.0
_DEFAULT_FY: float = 617.0
_DEFAULT_PIXEL_SIZE_MM: float = 0.003
_DEFAULT_FOCUS_DISTANCE: float = 400.0
_DEFAULT_F_STOP: float = 1.8
_DEFAULT_CLIPPING_NEAR: float = 1.0
_DEFAULT_CLIPPING_FAR: float = 1_000_000.0


class IsaacRgbCameraSensor:
    def __init__(
        self,
        prim_path: str,
        width: int = _DEFAULT_WIDTH,
        height: int = _DEFAULT_HEIGHT,
        fx: float = _DEFAULT_FX,
        fy: float = _DEFAULT_FY,
        pixel_size_mm: float = _DEFAULT_PIXEL_SIZE_MM,
        focus_distance: float = _DEFAULT_FOCUS_DISTANCE,
        f_stop: float = _DEFAULT_F_STOP,
        clipping_near: float = _DEFAULT_CLIPPING_NEAR,
        clipping_far: float = _DEFAULT_CLIPPING_FAR,
    ) -> None:
        self._prim_path = prim_path
        self._width = width
        self._height = height
        self._fx = fx
        self._fy = fy
        self._pixel_size_mm = pixel_size_mm
        self._focus_distance = focus_distance
        self._f_stop = f_stop
        self._clipping_near = clipping_near
        self._clipping_far = clipping_far
        self._camera = None
        self._ready: bool = False

    def setup(self) -> bool:
        if self._camera is not None and self._ready:
            return True
        try:
            from isaacsim.sensors.camera import Camera  # pylint: disable=E0401

            self._camera = Camera(
                prim_path=self._prim_path,
                resolution=(self._width, self._height),
            )
            self._camera.initialize()
            self._configure_pinhole()
            self._ready = True
        except Exception as exc:
            logger.error(f"Failed to setup IsaacRgbCameraSensor at '{self._prim_path}': {exc}")
            self._camera = None
            self._ready = False
            return False
        logger.info(
            f"IsaacRgbCameraSensor ready — prim='{self._prim_path}'"
            f" resolution=({self._width}x{self._height})"
        )
        return True

    def _configure_pinhole(self) -> None:
        h_aperture_mm = self._pixel_size_mm * self._width
        v_aperture_mm = self._pixel_size_mm * self._height
        focal_length_mm = (self._fx + self._fy) * self._pixel_size_mm / 2.0
        self._camera.set_focal_length(focal_length_mm / 1000.0)
        self._camera.set_focus_distance(self._focus_distance)
        self._camera.set_lens_aperture(self._f_stop * 1000.0)
        self._camera.set_horizontal_aperture(h_aperture_mm / 1000.0)
        self._camera.set_vertical_aperture(v_aperture_mm / 1000.0)
        self._camera.set_clipping_range(self._clipping_near, self._clipping_far)

    def teardown(self) -> None:
        self._camera = None
        self._ready = False

    def get_frame(self) -> Optional[bytes]:
        if not self._ready or self._camera is None:
            return None
        try:
            rgba = self._camera.get_rgba()
            if rgba is None:
                return None
            return bytes(rgba[:, :, :3].tobytes())
        except Exception as exc:
            logger.warning(f"Could not read camera frame: {exc}")
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
