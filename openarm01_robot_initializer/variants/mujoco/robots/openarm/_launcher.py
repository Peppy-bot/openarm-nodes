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
        except Exception:
            # Otherwise asyncio.run_in_executor captures the traceback in a
            # Future that may never be awaited — the process exits silently.
            logger.exception("SimLauncher.run failed")
            raise
        finally:
            extension.shutdown()
            self._ready.clear()

    def _run_streamed(self, model, data, extension: MujocoBridgeExtension) -> None:
        import mujoco as _mujoco
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
                viewer._refresh_scene_from_gui()  # pylint: disable=W0212

            # viewer.run() installs a SIGINT handler which only works on the
            # main thread; we run inside run_in_executor, so drive the internals
            # by hand instead.
            viewer._setup_gui()  # pylint: disable=W0212
            _mujoco.mj_forward(model, data)
            viewer._render()  # pylint: disable=W0212

            logger.info(
                "MuJoCo viewer available — open http://<host>:8080 in a browser"
            )
            _render_period = 1.0 / 60.0
            _dt = model.opt.timestep
            _last_render = 0.0
            _last_phys_wall = time.monotonic()
            while True:
                now = time.monotonic()
                # Step physics at real time, decoupled from render rate.
                n = int((now - _last_phys_wall) / _dt)
                if n > 0:
                    # Cap prevents spiral-of-death after stalls. Trade-off: on
                    # stall recovery the sim falls behind real time
                    # permanently rather than catching up.
                    n = min(n, 200)
                    for _ in range(n):
                        viewer._tick()  # pylint: disable=W0212
                    _last_phys_wall += n * _dt
                if now - _last_render >= _render_period:
                    viewer._render()  # pylint: disable=W0212
                    _last_render = now
                time.sleep(0.001)
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
