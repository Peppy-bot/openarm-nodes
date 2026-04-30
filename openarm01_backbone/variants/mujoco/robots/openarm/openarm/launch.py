#!/usr/bin/env python3
"""MuJoCo launch script for the openarm backbone configuration."""

# pylint: disable=C0413
import asyncio
import json
import logging
import os
import sys
from pathlib import Path

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s", force=True
)
logger = logging.getLogger(__name__)

_XML_PATH = Path(__file__).parent / "assets" / "openarm_bimanual.xml"
_MUJOCO_DIR = Path(__file__).resolve().parents[1]

os.environ["PEPPY_BRIDGE_NODE_NAME"] = "sim"

sys.path.insert(0, str(_MUJOCO_DIR))
from _launcher import SimLauncher


async def _run_sim(_params, _node_runner) -> list:
    loop = asyncio.get_running_loop()
    return [loop.run_in_executor(None, SimLauncher(_XML_PATH).run)]


try:
    from peppylib.runtime import NodeBuilder, StandaloneConfig  # pylint: disable=E0401

    builder = NodeBuilder()
    if not os.environ.get("PEPPY_RUNTIME_CONFIG"):
        # Local dev — build standalone config from daemon_state.json.
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

except ImportError:
    # Fallback: run without peppy node lifecycle (no peppylib installed).
    SimLauncher(_XML_PATH).run()
