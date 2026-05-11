#!/usr/bin/env python3
"""MuJoCo launch script for openarm01_robot_initializer."""

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

_ASSETS_DIR = Path(os.environ.get("PEPPY_ROBOT_ASSETS_DIR", str(Path(__file__).parent / "assets")))
_XML_PATH = _ASSETS_DIR / "openarm_bimanual.xml"
_MUJOCO_DIR = Path(__file__).resolve().parents[1]

os.environ["PEPPY_BRIDGE_NODE_NAME"] = "sim"

sys.path.insert(0, str(_MUJOCO_DIR))
from _launcher import SimLauncher  # pylint: disable=E0401

_ready = threading.Event()


async def _run_sim(_params, node_runner) -> list:
    from peppygen.exposed_services import is_ready  # pylint: disable=E0401

    async def _is_ready_loop() -> None:
        while True:
            await is_ready.handle_next_request(
                node_runner,
                lambda _req: is_ready.Response(ready=_ready.is_set()),
            )

    loop = asyncio.get_running_loop()
    return [
        loop.run_in_executor(None, SimLauncher(_XML_PATH, _ready).run),
        asyncio.create_task(_is_ready_loop()),
    ]


try:
    from peppylib.runtime import NodeBuilder, StandaloneConfig  # pylint: disable=E0401
except ImportError:
    SimLauncher(_XML_PATH, _ready).run()
else:
    builder = NodeBuilder()
    if not os.environ.get("PEPPY_RUNTIME_CONFIG"):
        _state_file = Path.home() / ".peppy" / "daemon_state.json"
        _PORT = 7448  # pylint: disable=C0103
        try:
            _state = json.loads(_state_file.read_text())
            _PORT = int(_state.get("messaging_port", _PORT))  # pylint: disable=C0103
        except Exception as e:
            logger.warning(f"Unable to read daemon state from {_state_file}: {e}")
        builder = builder.standalone(
            StandaloneConfig()
            .with_messaging("localhost", _PORT)
            .with_instance_id("sim")
            .with_node_name("sim")
            .with_parameters({})
        )

    builder.run(_run_sim)
