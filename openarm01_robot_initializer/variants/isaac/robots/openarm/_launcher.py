#!/usr/bin/env python3
"""Isaac Sim SimLauncher for openarm01_robot_initializer."""

# pylint: disable=R0903
from __future__ import annotations

import logging
import os
import threading
from pathlib import Path

logger = logging.getLogger(__name__)

_WARMUP_STEPS = 10


class SimLauncher:
    def __init__(self, sim_app, usd_path: Path, ready: threading.Event) -> None:
        self._sim_app = sim_app
        self._usd_path = usd_path
        self._ready = ready
        self._timeline = None

    def run(self) -> None:
        try:
            self._load_stage()
            self._warmup()
            self._start_timeline()
        except FileNotFoundError as exc:
            logger.error(str(exc))
            self._sim_app.close()
            os._exit(1)  # pylint: disable=W0212

        self._ready.set()
        logger.info("Scene loaded — is_ready: true")

        self._run_loop()

    def _load_stage(self) -> None:
        import omni.usd

        if not self._usd_path.exists():
            raise FileNotFoundError(
                f"USD not found at {self._usd_path} — "
                "run scripts/download_assets.sh to fetch assets"
            )
        logger.info(f"Loading stage: {self._usd_path}")
        omni.usd.get_context().open_stage(str(self._usd_path))

    def _warmup(self) -> None:
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
