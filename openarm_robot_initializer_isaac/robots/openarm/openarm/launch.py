#!/usr/bin/env python3
"""Isaac Sim launch script for openarm_robot_initializer."""

# pylint: disable=C0413
from __future__ import annotations

import asyncio
import logging
import os
import sys
import threading
from dataclasses import dataclass
from pathlib import Path

from peppygen.exposed_services.robot_ready import is_ready
from peppylib.runtime import NodeBuilder

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s", force=True
)
logger = logging.getLogger(__name__)

_ASSETS_DIR = Path(
    os.environ.get("PEPPY_ROBOT_ASSETS_DIR", str(Path(__file__).parent / "assets"))
)
def _version(hardware_version: str) -> str:
    version = hardware_version.lower()
    if version not in ("v1", "v2"):
        raise ValueError(f"hardware_version must be v1 or v2, got {hardware_version!r}")
    return version


def _scene_path(hardware_version: str) -> Path:
    # The v1 and v2 scenes are separate USD files (openarm_bimanual_v1.usd /
    # _v2.usd) in the base image's assets dir. A missing scene fails loudly at
    # load rather than silently simulating the other geometry.
    return _ASSETS_DIR / f"openarm_bimanual_{_version(hardware_version)}.usd"


_ROBOTS_DIR = Path(__file__).resolve().parents[1]

_ready = threading.Event()
_stop = threading.Event()


@dataclass
class _SimHandoff:
    """The node loop resolves these and hands them to the main-thread sim loop.

    SimulationApp must be constructed on the main thread before any omni.*
    import, but its headless flag comes from the node parameters, which are only
    available once the node runner calls setup(). The node thread stashes the
    resolved IO + params here and flips `ready` so main() can proceed.
    """

    io: object
    state_rate_hz: int
    headless: bool
    hardware_version: str


_handoff: dict[str, _SimHandoff] = {}
_handoff_ready = threading.Event()


async def setup(params, node_runner) -> list:
    # Typed peppygen pub/sub lives on this loop; declare publishers and start the
    # command-consume tasks before the sim thread starts reading from them.
    sys.path.insert(0, str(_ROBOTS_DIR))
    from sim_topics import SimTopicIO

    loop = asyncio.get_running_loop()
    io = SimTopicIO(node_runner, loop)
    await io.start()

    _handoff["value"] = _SimHandoff(
        io=io,
        state_rate_hz=params.state_rate_hz,
        headless=params.headless,
        hardware_version=params.hardware_version,
    )
    _handoff_ready.set()

    async def _is_ready_loop() -> None:
        while True:
            await is_ready.handle_next_request(
                node_runner,
                lambda _req: is_ready.Response(ready=_ready.is_set()),
            )

    async def _shutdown_hook() -> None:
        # Drive the Isaac main-thread sim loop to exit inside the runtime grace
        # window; SimLauncher's finally bounds extension.shutdown(), then cancel
        # the consume tasks.
        _stop.set()
        await io.stop()

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
    if not _handoff_ready.wait(timeout=30):
        raise RuntimeError("node parameters not resolved within 30s")
    handoff = _handoff["value"]

    # SimulationApp must initialise before any omni.* import. Its headless flag
    # comes from the resolved node parameters; the renderer is a container-level
    # knob.
    from isaacsim import SimulationApp

    launch_config = {
        "headless": handoff.headless,
        "renderer": os.environ.get("PEPPY_ISAAC_RENDERER", "RayTracedLighting"),
    }
    if handoff.headless:
        simulation_app = SimulationApp(
            launch_config,
            experience="/isaac-sim/apps/isaacsim.exp.full.streaming.kit",
        )
    else:
        simulation_app = SimulationApp(launch_config)

    sys.path.insert(0, str(_ROBOTS_DIR))
    from _launcher import SimLauncher

    SimLauncher(
        simulation_app,
        _scene_path(handoff.hardware_version),
        _ready,
        _stop,
        handoff.io,
        handoff.state_rate_hz,
    ).run()


if __name__ == "__main__":
    main()
