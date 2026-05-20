#!/usr/bin/env python3
# pylint: disable=R0903,C0413
from __future__ import annotations

import logging
import os
import threading
import time
from pathlib import Path

logger = logging.getLogger(__name__)

_HEADLESS_ENV = "PEPPY_BRIDGE_HEADLESS"
_BUS_DIR_ENV = "PEPPY_MJDATA_BUS_DIR"
_DEFAULT_BUS_DIR = f"/run/user/{os.getuid()}/peppy/sim"


class SimLauncher:
    def __init__(self, xml_path: Path, ready: threading.Event) -> None:
        self._xml_path = xml_path
        self._ready = ready
        self._headless = os.environ.get(_HEADLESS_ENV, "1").strip() == "1"
        self._bus_dir = Path(os.environ.get(_BUS_DIR_ENV, _DEFAULT_BUS_DIR))

    def run(self) -> None:
        import mujoco  # pylint: disable=C0415
        from mjdata_bus import MjDataBus  # pylint: disable=C0415

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

        bus = MjDataBus(self._bus_dir, model)
        try:
            bus.open()
            bus.copy_state_from(data, 0)

            self._ready.set()
            logger.info("Scene loaded — is_ready: true")

            if self._headless:
                self._run_streamed(model, data, bus)
            else:
                self._run_windowed(model, data, bus)
        finally:
            bus.close()
            self._ready.clear()

    def _run_streamed(self, model, data, bus: MjDataBus) -> None:
        import mujoco as _mujoco  # pylint: disable=C0415
        import viser  # pylint: disable=C0415
        import mjviser  # pylint: disable=C0415

        step_counter = [0]

        def _step_fn(m, d) -> None:
            bus.copy_ctrl_to(d)
            _mujoco.mj_step(m, d)
            step_counter[0] += 1
            bus.copy_state_from(d, step_counter[0])

        try:
            server = viser.ViserServer(host="0.0.0.0", port=8080)
            viewer = mjviser.Viewer(model, data, server=server, step_fn=_step_fn)

            # viser sends batched position updates as delta messages only —
            # new/refreshing clients receive initial zero positions unless we
            # explicitly push current state on each connection.
            @server.on_client_connect
            def _(client) -> None:  # noqa: ARG001
                viewer._refresh_scene_from_gui()  # pylint: disable=W0212

            # viewer.run() installs signal handlers which only work on the main
            # thread; we run inside run_in_executor so call the internals directly.
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
                    n = min(n, 200)  # cap prevents spiral-of-death after stalls
                    for _ in range(n):
                        viewer._tick()  # pylint: disable=W0212
                    _last_phys_wall += n * _dt
                if now - _last_render >= _render_period:
                    viewer._render()  # pylint: disable=W0212
                    _last_render = now
                time.sleep(0.001)
        except KeyboardInterrupt:
            logger.info("Shutting down.")

    def _run_windowed(self, model, data, bus: MjDataBus) -> None:
        import mujoco  # pylint: disable=C0415
        import mujoco.viewer  # pylint: disable=C0415

        _dt = model.opt.timestep
        step_count = 0
        _last_phys_wall = time.monotonic()
        try:
            with mujoco.viewer.launch_passive(model, data) as viewer:
                while viewer.is_running():
                    now = time.monotonic()
                    n = int((now - _last_phys_wall) / _dt)
                    if n > 0:
                        n = min(n, 200)
                        for _ in range(n):
                            bus.copy_ctrl_to(data)
                            mujoco.mj_step(model, data)
                            step_count += 1
                            bus.copy_state_from(data, step_count)
                        _last_phys_wall += n * _dt
                    viewer.sync()
                    time.sleep(0.001)
        except KeyboardInterrupt:
            logger.info("Shutting down.")
