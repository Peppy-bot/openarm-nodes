#!/usr/bin/env python3
"""Shared SimLauncher for openarm Isaac Sim launch scripts."""

# pylint: disable=E0401
import logging
import sys
from pathlib import Path

logger = logging.getLogger(__name__)

_WARMUP_STEPS = 10
_EXTENSION_NAME = "omni.peppy.backbone"


class SimLauncher:
    """Manages the Isaac Sim lifecycle for an openarm robot."""

    def __init__(self, sim_app, usd_path: Path) -> None:
        self._sim_app = sim_app
        self._usd_path = usd_path
        self._timeline = None
        self._world = None

    def run(self) -> None:
        """Enable the extension, load the stage, and drive the simulation loop."""
        try:
            self._enable_extension()
            self._load_stage()
            self._warmup()
            self._start_timeline()
        except FileNotFoundError as exc:
            logger.error(str(exc))
            self._sim_app.close()
            sys.exit(1)
        logger.info("Simulation running — Press Ctrl-C to stop.")
        logger.info("Publishing sensor topics: Isaac Sim → openarm01_backbone")
        self._run_loop()

    def _enable_extension(self) -> None:
        import omni.kit.app

        manager = omni.kit.app.get_app().get_extension_manager()
        manager.set_extension_enabled_immediate(_EXTENSION_NAME, True)

    def _load_stage(self) -> None:
        import omni.usd

        if not self._usd_path.exists():
            raise FileNotFoundError(
                f"USD not found at {self._usd_path} — download from "
                "https://github.com/enactic/openarm_isaac_lab and place it in assets/"
            )
        logger.info(f"Loading stage: {self._usd_path}")
        omni.usd.get_context().open_stage(str(self._usd_path))

    def _warmup(self) -> None:
        from isaacsim.core.api import World  # pylint: disable=E0401

        self._world = World()
        for _ in range(_WARMUP_STEPS):
            self._sim_app.update()

    def _start_timeline(self) -> None:
        import omni.timeline

        self._timeline = omni.timeline.get_timeline_interface()
        self._timeline.play()

    def _run_loop(self) -> None:
        try:
            while self._sim_app.is_running():
                self._sim_app.update()
        except KeyboardInterrupt:
            logger.info("Shutting down.")
        finally:
            if self._timeline is not None:
                self._timeline.stop()
            self._sim_app.close()
