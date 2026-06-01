#!/usr/bin/env python3
"""MuJoCo launch script for openarm01_robot_initializer."""

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
_XML_PATH = _ASSETS_DIR / "openarm_bimanual.xml"
_MUJOCO_DIR = Path(__file__).resolve().parents[1]

os.environ["PEPPY_BRIDGE_NODE_NAME"] = "sim"

sys.path.insert(0, str(_MUJOCO_DIR))
from _launcher import SimLauncher

_ready = threading.Event()
_stop = threading.Event()


async def _run_sim(_params, node_runner) -> list:
    async def _is_ready_loop() -> None:
        while True:
            await is_ready.handle_next_request(
                node_runner,
                lambda _req: is_ready.Response(ready=_ready.is_set()),
            )

    async def _run_sim_task() -> None:
        loop = asyncio.get_running_loop()
        try:
            await loop.run_in_executor(
                None, SimLauncher(_XML_PATH, _ready, _stop).run
            )
        finally:
            # Signal the sim thread to exit. Without this, asyncio task
            # cancellation cancels the awaitable but leaves the executor
            # thread spinning forever, blocking process exit and starving
            # peppylib's framework `shutdown` service.
            _stop.set()

    return [
        asyncio.create_task(_run_sim_task()),
        asyncio.create_task(_is_ready_loop()),
    ]


def main() -> None:
    NodeBuilder().run(_run_sim)


if __name__ == "__main__":
    main()
