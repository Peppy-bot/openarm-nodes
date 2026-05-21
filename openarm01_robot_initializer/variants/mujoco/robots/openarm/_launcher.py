#!/usr/bin/env python3
"""MuJoCo SimLauncher for openarm01_robot_initializer."""

# pylint: disable=R0903
from __future__ import annotations

import logging
import os
import threading
import time
from pathlib import Path

from bridge_extension import MujocoBridgeExtension

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

        extension = MujocoBridgeExtension(model, data)
        try:
            extension.startup()
            self._ready.set()
            logger.info("Scene loaded — is_ready: true")

            if self._headless:
                self._run_streamed(model, data, extension)
            else:
                self._run_windowed(model, data, extension)
        finally:
            extension.shutdown()
            self._ready.clear()

    def _run_streamed(self, model, data, extension: MujocoBridgeExtension) -> None:
        import viser
        import mjviser

        # Hand mjviser the bridge extension's step() — it owns mj_step plus the
        # plugin loop, so this single callback is the entire per-tick work.
        def _step_fn(_m, _d) -> None:
            extension.step()

        try:
            server = viser.ViserServer(host="0.0.0.0", port=8080)
            viewer = mjviser.Viewer(model, data, server=server, step_fn=_step_fn)

            # viser sends batched position updates as delta messages only —
            # new/refreshing clients receive initial zero positions unless we
            # explicitly push current state on each connection.
            @server.on_client_connect
            def _(client) -> None:
                viewer._refresh_scene_from_gui()

            logger.info(
                "MuJoCo viewer available — open http://<host>:8080 in a browser"
            )
            viewer.run()
        except KeyboardInterrupt:
            logger.info("Shutting down.")

    def _run_windowed(self, model, data, extension: MujocoBridgeExtension) -> None:
        import mujoco
        import mujoco.viewer

        dt = model.opt.timestep
        try:
            with mujoco.viewer.launch_passive(model, data) as viewer:
                while viewer.is_running():
                    step_start = time.monotonic()
                    extension.step()
                    viewer.sync()
                    elapsed = time.monotonic() - step_start
                    remaining = dt - elapsed
                    if remaining > 0:
                        time.sleep(remaining)
        except KeyboardInterrupt:
            logger.info("Shutting down.")
