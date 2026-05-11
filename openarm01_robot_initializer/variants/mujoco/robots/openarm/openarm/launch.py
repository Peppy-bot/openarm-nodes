#!/usr/bin/env python3
"""MuJoCo launch script for openarm01_robot_initializer."""

# pylint: disable=C0413,E1120
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
_XML_PATH = _ASSETS_DIR / "openarm_bimanual.xml"
_MUJOCO_DIR = Path(__file__).resolve().parents[1]

os.environ["PEPPY_BRIDGE_NODE_NAME"] = "sim"

sys.path.insert(0, str(_MUJOCO_DIR))
from _launcher import SimLauncher

_ready = threading.Event()


async def _run_sim(_params, node_runner) -> list:
    from peppygen.exposed_services import is_ready

    async def _is_ready_loop() -> None:
        while True:
            await is_ready.handle_next_request(
                node_runner,
                lambda _req: is_ready.Response(ready=_ready.is_set()),
            )

    async def _run_sim_task() -> None:
        loop = asyncio.get_running_loop()
        try:
            await loop.run_in_executor(None, SimLauncher(_XML_PATH, _ready).run)
        except Exception:
            os._exit(1)

    return [
        asyncio.create_task(_run_sim_task()),
        asyncio.create_task(_is_ready_loop()),
    ]


try:
    from peppylib.runtime import NodeBuilder, StandaloneConfig
except ImportError as exc:
    raise RuntimeError(
        "peppylib unavailable — is_ready service cannot be served"
    ) from exc

builder = NodeBuilder()
if not os.environ.get("PEPPY_RUNTIME_CONFIG"):
    _state_file = Path.home() / ".peppy" / "daemon_state.json"
    _port = 7448  # pylint: disable=C0103
    try:
        _state = json.loads(_state_file.read_text())
        _port = int(_state.get("messaging_port", _port))  # pylint: disable=C0103
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
