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
# Set by the sim main thread once the bounded part of its teardown (bridge
# extension shutdown, timeline stop) is done; only sim_app.close() remains.
_torn_down = threading.Event()


async def setup(_params, node_runner) -> list:
    loop = asyncio.get_running_loop()
    token = node_runner.cancellation_token()

    async def _stop_sim() -> None:
        # Isaac teardown must run on the sim main thread, which cannot observe
        # asyncio cancellation; _stop is its only stop path. Awaiting
        # _torn_down makes the bounded teardown (bridge/messenger shutdown,
        # timeline stop) an awaited obligation inside the shutdown grace
        # window. sim_app.close() stays outside the wait: renderer teardown
        # can exceed any realistic grace window and is covered by the
        # force-kill.
        _stop.set()
        await loop.run_in_executor(None, _torn_down.wait, 2.5)

    node_runner.on_shutdown(_stop_sim)

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

    return [asyncio.create_task(_is_ready_loop())]


def _run_node_builder() -> None:
    # The on_shutdown hook flips _stop on every runtime stop path; this
    # finally is the backstop for exits that bypass the hook (NodeBuilder.run
    # failing before setup registers it), so the main-thread sim loop is
    # always released and Isaac never keeps spinning forever.
    try:
        NodeBuilder().run(setup)
    finally:
        _stop.set()


def main() -> None:
    threading.Thread(target=_run_node_builder, daemon=True).start()
    SimLauncher(simulation_app, _USD_PATH, _ready, _stop, _torn_down).run()


if __name__ == "__main__":
    main()
