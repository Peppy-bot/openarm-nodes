#!/usr/bin/env python3
"""Isaac Sim launch script for openarm01_robot_initializer."""

# pylint: disable=C0413
from __future__ import annotations

import asyncio
import json
import logging
import os
import sys
import threading
from pathlib import Path

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s", force=True
)
logger = logging.getLogger(__name__)

_ASSETS_DIR = Path(
    os.environ.get("PEPPY_ROBOT_ASSETS_DIR", str(Path(__file__).parent / "assets"))
)
_USD_PATH = _ASSETS_DIR / "openarm_bimanual.usd"
_ROBOTS_DIR = Path(__file__).resolve().parents[1]
_NODE_ROOT = Path("/opt/isaac_robot_initializer")

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


def _start_peppylib() -> None:
    # chdir so peppylib finds peppy.json5 and the codegen fingerprint.
    os.chdir(str(_NODE_ROOT))

    async def _run_sim(_params, node_runner) -> list:
        from peppygen.exposed_services import is_ready

        async def _is_ready_loop() -> None:
            while True:
                await is_ready.handle_next_request(
                    node_runner,
                    lambda _req: is_ready.Response(ready=_ready.is_set()),
                )

        return [asyncio.create_task(_is_ready_loop())]

    try:
        from peppylib.runtime import NodeBuilder, StandaloneConfig
    except ImportError:
        logger.error("peppylib unavailable — is_ready service cannot be served")
        os._exit(1)

    builder = NodeBuilder()
    if not os.environ.get("PEPPY_RUNTIME_CONFIG"):
        _state_file = Path.home() / ".peppy" / "daemon_state.json"
        _port = 7448
        try:
            _state = json.loads(_state_file.read_text())
            _port = int(_state.get("messaging_port", _port))
        except Exception as e:
            logger.warning(f"Unable to read daemon state from {_state_file}: {e}")
        builder = builder.standalone(
            StandaloneConfig()
            .with_messaging("localhost", _port)
            .with_instance_id("sim")
            .with_node_name("sim")
            .with_parameters({})
        )

    builder.run(_run_sim)


# peppylib runs in a background thread — Isaac Sim must own the main thread.
threading.Thread(target=_start_peppylib, daemon=True).start()
SimLauncher(simulation_app, _USD_PATH, _ready).run()
