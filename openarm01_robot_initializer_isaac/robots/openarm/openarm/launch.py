#!/usr/bin/env python3
"""Isaac Sim launch script for openarm01_robot_initializer."""

# pylint: disable=C0413
from __future__ import annotations

import asyncio
import logging
import os
import sys
import threading
from pathlib import Path

from peppygen.exposed_services.openarm01_robot_initializer.v1 import is_ready
from peppylib.runtime import NodeBuilder

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s", force=True
)
logger = logging.getLogger(__name__)

_ASSETS_DIR = Path(
    os.environ.get("PEPPY_ROBOT_ASSETS_DIR", str(Path(__file__).parent / "assets"))
)
_USD_PATH = _ASSETS_DIR / "openarm_bimanual.usd"
_ROBOTS_DIR = Path(__file__).resolve().parents[1]

# Must be set before SimulationApp initialises.
os.environ["PEPPY_BRIDGE_NODE_NAME"] = "sim"

from isaacsim import SimulationApp

_headless = os.environ.get("PEPPY_BRIDGE_HEADLESS", "1") == "1"
_launch_config = {
    "headless": _headless,
    "renderer": os.environ.get("PEPPY_ISAAC_RENDERER", "RayTracedLighting"),
}
if _headless:
    simulation_app = SimulationApp(
        _launch_config,
        experience="/isaac-sim/apps/isaacsim.exp.full.streaming.kit",
    )
else:
    simulation_app = SimulationApp(_launch_config)

sys.path.insert(0, str(_ROBOTS_DIR))
from _launcher import SimLauncher

_ready = threading.Event()
_stop = threading.Event()


async def setup(_params, node_runner) -> list:
    async def _is_ready_loop() -> None:
        while True:
            await is_ready.handle_next_request(
                node_runner,
                lambda _req: is_ready.Response(ready=_ready.is_set()),
            )

    async def _shutdown_hook() -> None:
        # Drive the Isaac main-thread sim loop to exit inside the runtime
        # grace window; SimLauncher's finally bounds peppylib_io.stop().
        _stop.set()

    node_runner.on_shutdown(_shutdown_hook)

    return [asyncio.create_task(_is_ready_loop())]


def _run_node_builder() -> None:
    # NodeBuilder.run returns when peppylib's shutdown service cancels its
    # tasks. Flip _stop so the main-thread sim loop can exit; without this
    # the asyncio side tears down cleanly but Isaac keeps spinning forever.
    try:
        NodeBuilder().run(setup)
    finally:
        _stop.set()


def main() -> None:
    threading.Thread(target=_run_node_builder, daemon=True).start()
    SimLauncher(simulation_app, _USD_PATH, _ready, _stop).run()


if __name__ == "__main__":
    main()
