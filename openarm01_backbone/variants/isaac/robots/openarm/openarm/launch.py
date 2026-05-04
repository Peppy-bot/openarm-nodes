#!/usr/bin/env python3
"""Isaac Sim launch script for the openarm backbone configuration."""

# pylint: disable=C0413
# pylint: disable=E0401
import logging
import os
import sys
from pathlib import Path

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s", force=True
)

_ASSETS_DIR = Path(os.environ.get("PEPPY_ROBOT_ASSETS_DIR", str(Path(__file__).parent / "assets")))
_USD_PATH = _ASSETS_DIR / "openarm_bimanual.usd"
_EXT_ROOT = Path(__file__).resolve().parents[3] / "exts"
_ROBOTS_DIR = Path(__file__).resolve().parents[2]

# Bridge config — must be set before SimulationApp initialises.
os.environ["PEPPY_BRIDGE_NODE_NAME"] = "sim"

from isaacsim import SimulationApp

simulation_app = SimulationApp(
    {
        "headless": os.environ.get("PEPPY_BRIDGE_HEADLESS", "0") == "1",
        "renderer": "RayTracedLighting",
        "extra_args": ["--ext-folder", str(_EXT_ROOT)],
    }
)

sys.path.insert(0, str(_ROBOTS_DIR))
from _launcher import SimLauncher

SimLauncher(simulation_app, _USD_PATH).run()  # pylint: disable=E1121
