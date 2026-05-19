#!/usr/bin/env python3
"""Isaac Sim SimLauncher for openarm01_robot_initializer."""

# pylint: disable=R0903
from __future__ import annotations

import logging
import os
import threading
from pathlib import Path

logger = logging.getLogger(__name__)

_WARMUP_STEPS = 100
_EXTS_DIR_ENV = "PEPPY_ISAAC_EXTS_DIR"
_DEFAULT_EXTS_DIR = "/var/run/peppy/sim/exts"


class SimLauncher:
    def __init__(self, sim_app, usd_path: Path, ready: threading.Event) -> None:
        self._sim_app = sim_app
        self._usd_path = usd_path
        self._ready = ready
        self._timeline = None
        self._world = None
        self._exts_dir = Path(os.environ.get(_EXTS_DIR_ENV, _DEFAULT_EXTS_DIR))

    def run(self) -> None:
        try:
            self._register_ext_path()
            self._load_stage()
            self._setup_lighting()
            self._warmup()
            self._start_timeline()
            self._ready.set()
            logger.info("Scene loaded — is_ready: true")
            self._run_loop()
        except FileNotFoundError as exc:
            logger.error(str(exc))
            self._sim_app.close()

    def _register_ext_path(self) -> None:
        # Make the per-stack extension dir discoverable by Isaac's native
        # ExtensionManager. Component-variant containers (gripper, arm, ...)
        # drop their `omni.peppy.<node>` driver extensions here; Isaac then
        # autoloads them when extension.toml declares autoload=true. We do NOT
        # enable specific extensions by name — that knowledge belongs to each
        # component, not robot_initializer.
        import omni.kit.app  # pylint: disable=C0415,E0401

        self._exts_dir.mkdir(parents=True, exist_ok=True)
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_manager.add_path(str(self._exts_dir))
        logger.info(f"Isaac extension path added: {self._exts_dir}")

    def _load_stage(self) -> None:
        import omni.usd

        if not self._usd_path.exists():
            raise FileNotFoundError(
                f"USD not found at {self._usd_path}"
                " — assets should be baked into the container image"
            )
        logger.info(f"Loading stage: {self._usd_path}")
        omni.usd.get_context().open_stage(str(self._usd_path))

    def _setup_lighting(self) -> None:
        import omni.usd
        from pxr import Sdf, UsdLux

        stage = omni.usd.get_context().get_stage()
        light = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/defaultDomeLight"))
        light.CreateIntensityAttr(1000)
        logger.info("Default dome light added to stage")

    def _warmup(self) -> None:
        from omni.isaac.core import World  # pylint: disable=E0401

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
            self._ready.clear()
            if self._timeline is not None:
                self._timeline.stop()
            self._sim_app.close()
            logger.info("Isaac Sim closed.")
