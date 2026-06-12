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
    loop = asyncio.get_running_loop()
    token = node_runner.cancellation_token()

    sim_future = loop.run_in_executor(None, SimLauncher(_XML_PATH, _ready, _stop).run)

    async def _stop_sim() -> None:
        # The sim thread cannot observe asyncio cancellation; _stop is its
        # only stop path. Awaiting the executor future makes the thread's
        # teardown (viser stop, bridge/messenger shutdown) an awaited
        # obligation inside the shutdown grace window. Bounded below the 3s
        # window; a truly stuck sim thread is covered by the force-kill.
        _stop.set()
        await asyncio.wait({sim_future}, timeout=2.5)

    node_runner.on_shutdown(_stop_sim)

    async def _watch_sim() -> None:
        # Surface SimLauncher.run failures: an uncaught task exception
        # cancels the node via the runtime's loop exception handler.
        await sim_future

    async def _is_ready_loop() -> None:
        # handle_next_request parks until the next request arrives and is not
        # token-aware, so race it against the token: otherwise the loop stays
        # blocked on this await at shutdown and only ends when the runtime
        # force-cancels the task (best effort, racing process exit) instead of
        # returning cooperatively within the grace window.
        cancelled = asyncio.ensure_future(token.cancelled())
        try:
            while not token.is_cancelled():
                next_req = asyncio.ensure_future(
                    is_ready.handle_next_request(
                        node_runner,
                        lambda _req: is_ready.Response(ready=_ready.is_set()),
                    )
                )
                await asyncio.wait(
                    [cancelled, next_req], return_when=asyncio.FIRST_COMPLETED
                )
                if not next_req.done():
                    next_req.cancel()
                    break
        finally:
            cancelled.cancel()

    return [
        asyncio.create_task(_watch_sim()),
        asyncio.create_task(_is_ready_loop()),
    ]


def main() -> None:
    NodeBuilder().run(_run_sim)


if __name__ == "__main__":
    main()
