#!/usr/bin/env python3
"""Shared SimLauncher for openarm MuJoCo launch scripts."""

import logging
import os
import sys
import time
from pathlib import Path

logger = logging.getLogger(__name__)

_HEADLESS_ENV = "PEPPY_BRIDGE_HEADLESS"
_EXT_ROOT_ENV = "PEPPY_MUJOCO_EXT_ROOT"
_UVC_EXT_ROOT_ENV = "PEPPY_UVC_CAMERA_EXT_ROOT"

_NODES_ROOT = Path(__file__).resolve().parents[5]

_DEFAULT_EXT_ROOT = (
    Path(__file__).resolve().parents[3] / "exts" / "mujoco.peppy.backbone"
)
_DEFAULT_UVC_EXT_ROOT = (
    _NODES_ROOT / "uvc_camera" / "variants" / "mujoco" / "exts" / "mujoco.peppy.uvc_camera"
)


class SimLauncher:
    """Manages the MuJoCo lifecycle for an openarm robot."""

    def __init__(self, xml_path: Path) -> None:
        self._xml_path = xml_path
        self._headless = os.environ.get(_HEADLESS_ENV, "0").strip() == "1"

    def run(self) -> None:
        """Load model, init all extensions, and drive the simulation loop."""
        import mujoco  # pylint: disable=E0401

        if not self._xml_path.exists():
            raise FileNotFoundError(
                f"MJCF not found at {self._xml_path} — "
                "download from https://github.com/enactic/openarm"
                " and place it in assets/"
            )

        logger.info(f"Loading model: {self._xml_path}")
        model = mujoco.MjModel.from_xml_path(str(self._xml_path))
        data = mujoco.MjData(model)

        backbone_ext_root = Path(os.environ.get(_EXT_ROOT_ENV, str(_DEFAULT_EXT_ROOT)))
        uvc_ext_root = Path(os.environ.get(_UVC_EXT_ROOT_ENV, str(_DEFAULT_UVC_EXT_ROOT)))

        sys.path.insert(0, str(backbone_ext_root))
        sys.path.insert(0, str(uvc_ext_root))

        from peppy_mujoco.backbone.extension import MujocoBackboneExtension  # pylint: disable=E0401
        from peppy_mujoco.uvc_camera.extension import MujocoUvcCameraExtension  # pylint: disable=E0401

        backbone = MujocoBackboneExtension(model, data)
        uvc = MujocoUvcCameraExtension(model, data)

        backbone.startup()
        uvc.startup()

        logger.info("Simulation running — Press Ctrl-C to stop.")

        extensions = [backbone, uvc]

        if self._headless:
            self._run_headless(model, backbone, extensions)
        else:
            self._run_windowed(model, data, backbone, extensions)

    def _run_headless(self, model, backbone, extensions) -> None:
        """Drive simulation loop at fixed timestep without a viewer."""
        dt = model.opt.timestep
        try:
            while True:
                t0 = time.perf_counter()
                self._step(backbone, extensions)
                remaining = dt - (time.perf_counter() - t0)
                if remaining > 0:
                    time.sleep(remaining)
        except KeyboardInterrupt:
            logger.info("Shutting down.")
        finally:
            for ext in extensions:
                ext.shutdown()

    def _run_windowed(self, model, data, backbone, extensions) -> None:
        """Drive simulation loop at fixed timestep with the passive MuJoCo viewer."""
        import mujoco.viewer  # pylint: disable=E0401

        dt = model.opt.timestep
        try:
            with mujoco.viewer.launch_passive(model, data) as viewer:
                while viewer.is_running():
                    t0 = time.perf_counter()
                    self._step(backbone, extensions)
                    viewer.sync()
                    remaining = dt - (time.perf_counter() - t0)
                    if remaining > 0:
                        time.sleep(remaining)
        except KeyboardInterrupt:
            logger.info("Shutting down.")
        finally:
            for ext in extensions:
                ext.shutdown()

    @staticmethod
    def _step(backbone, extensions) -> None:
        """Advance physics via backbone, then drive all other extensions."""
        backbone.step()
        if not backbone.is_paused:
            for ext in extensions[1:]:
                ext.step()
