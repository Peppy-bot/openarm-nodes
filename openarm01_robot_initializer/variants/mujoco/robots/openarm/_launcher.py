#!/usr/bin/env python3
"""MuJoCo SimLauncher for openarm01_robot_initializer."""

# pylint: disable=R0903
from __future__ import annotations

import logging
import os
import threading
import time
from pathlib import Path

logger = logging.getLogger(__name__)

_HEADLESS_ENV = "PEPPY_BRIDGE_HEADLESS"


class SimLauncher:
    def __init__(self, xml_path: Path, ready: threading.Event) -> None:
        self._xml_path = xml_path
        self._ready = ready
        self._headless = os.environ.get(_HEADLESS_ENV, "1").strip() == "1"

    def run(self) -> None:
        import mujoco

        if not self._xml_path.exists():
            logger.error(
                "MJCF not found at %s — assets should be baked into the container image",
                self._xml_path,
            )
            raise FileNotFoundError(self._xml_path)

        logger.info(f"Loading model: {self._xml_path}")
        model = mujoco.MjModel.from_xml_path(str(self._xml_path))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)

        self._ready.set()
        logger.info("Scene loaded — is_ready: true")

        if self._headless:
            self._run_streamed(model, data)
        else:
            self._run_windowed(model, data)

    def _run_streamed(self, model, data) -> None:
        import viser
        import mjviser

        try:
            server = viser.ViserServer(host="0.0.0.0", port=8080)
            viewer = mjviser.Viewer(model, data, server=server)
            logger.info(
                "MuJoCo viewer available — open http://<host>:8080 in a browser"
            )
            viewer.run()
        except KeyboardInterrupt:
            logger.info("Shutting down.")
        finally:
            self._ready.clear()

    def _run_windowed(self, model, data) -> None:
        import mujoco
        import mujoco.viewer

        dt = model.opt.timestep
        try:
            with mujoco.viewer.launch_passive(model, data) as viewer:
                while viewer.is_running():
                    step_start = time.monotonic()
                    mujoco.mj_step(model, data)
                    viewer.sync()
                    elapsed = time.monotonic() - step_start
                    remaining = dt - elapsed
                    if remaining > 0:
                        time.sleep(remaining)
        except KeyboardInterrupt:
            logger.info("Shutting down.")
        finally:
            self._ready.clear()
