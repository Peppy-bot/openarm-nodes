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

from peppygen.exposed_services import is_ready
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
_NODE_ROOT = Path(__file__).resolve().parents[3]

# Must be set before SimulationApp initialises.
os.environ["PEPPY_BRIDGE_NODE_NAME"] = "sim"

from isaacsim import SimulationApp

simulation_app = SimulationApp(
    {
        "headless": os.environ.get("PEPPY_BRIDGE_HEADLESS", "1") == "1",
        "renderer": os.environ.get("PEPPY_ISAAC_RENDERER", "RayTracedLighting"),
    }
)

sys.path.insert(0, str(_ROBOTS_DIR))
from _launcher import SimLauncher

_ready = threading.Event()


async def setup(_params, node_runner) -> list:
    async def _is_ready_loop() -> None:
        while True:
            await is_ready.handle_next_request(
                node_runner,
                lambda _req: is_ready.Response(ready=_ready.is_set()),
            )

    return [asyncio.create_task(_is_ready_loop())]


def main() -> None:
    os.chdir(str(_NODE_ROOT))
    threading.Thread(
        target=lambda: NodeBuilder().run(setup),
        daemon=True,
    ).start()
    SimLauncher(simulation_app, _USD_PATH, _ready).run()


if __name__ == "__main__":
    main()
