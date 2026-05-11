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
_ASSETS_ENV = "PEPPY_ROBOT_ASSETS_DIR"
_DEFAULT_ASSETS = Path(__file__).resolve().parents[1] / "openarm" / "assets"


class SimLauncher:
    def __init__(self, xml_path: Path, ready: threading.Event) -> None:
        self._xml_path = xml_path
        self._ready = ready
        self._headless = os.environ.get(_HEADLESS_ENV, "1").strip() == "1"

    def run(self) -> None:
        import mujoco  # pylint: disable=E0401

        if not self._xml_path.exists():
            raise FileNotFoundError(
                f"MJCF not found at {self._xml_path} — "
                "run scripts/download_assets.sh to fetch assets"
            )

        logger.info(f"Loading model: {self._xml_path}")
        model = mujoco.MjModel.from_xml_path(str(self._xml_path))
        data = mujoco.MjData(model)

        self._ready.set()
        logger.info("Scene loaded — is_ready: true")

        if self._headless:
            self._run_headless(model)
        else:
            self._run_windowed(model, data)

    def _run_headless(self, model) -> None:
        dt = model.opt.timestep
        try:
            while True:
                time.sleep(dt)
        except KeyboardInterrupt:
            logger.info("Shutting down.")

    def _run_windowed(self, model, data) -> None:
        import mujoco.viewer  # pylint: disable=E0401

        dt = model.opt.timestep
        try:
            with mujoco.viewer.launch_passive(model, data) as viewer:
                while viewer.is_running():
                    time.sleep(dt)
        except KeyboardInterrupt:
            logger.info("Shutting down.")
