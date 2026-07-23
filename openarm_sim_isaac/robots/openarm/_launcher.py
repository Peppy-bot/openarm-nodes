#!/usr/bin/env python3
"""Isaac Sim SimLauncher for openarm_robot_initializer."""

# pylint: disable=R0903
from __future__ import annotations

import logging
import threading
from pathlib import Path
from typing import Optional

from bridge_extension import IsaacBridgeExtension

logger = logging.getLogger(__name__)

_WARMUP_STEPS = 100


class SimLauncher:
    def __init__(
        self,
        sim_app,
        usd_path: Path,
        ready: threading.Event,
        stop: threading.Event,
        io,
        state_rate_hz: int,
    ) -> None:
        self._sim_app = sim_app
        self._usd_path = usd_path
        self._ready = ready
        # `stop` is flipped by the NodeBuilder thread's finally when peppylib's
        # shutdown service runs (peppy node stop, SIGTERM). The sim loop owns
        # the main thread and won't see asyncio cancellation otherwise.
        self._stop = stop
        self._io = io
        self._state_rate_hz = state_rate_hz
        self._timeline = None
        self._world = None
        self._extension: Optional[IsaacBridgeExtension] = None

    def run(self) -> None:
        try:
            self._load_stage()
            self._setup_lighting()
            self._warmup()
            self._start_timeline()
            self._extension = IsaacBridgeExtension(self._io, self._state_rate_hz)
            logger.info("Scene loaded — waiting for bridge setup")
            self._run_loop()
        except FileNotFoundError as exc:
            logger.error(str(exc))
            self._sim_app.close()
        except Exception:
            logger.exception("SimLauncher.run failed")
            raise

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
            while self._sim_app.is_running() and not self._stop.is_set():
                # Isaac advances physics inside update(); we then drive the
                # bridge step on the same thread (Articulation reads require
                # Isaac's main thread). The extension defers its own setup until
                # the stage is live, so early steps are cheap no-ops.
                self._sim_app.update()
                if self._extension is not None:
                    self._extension.step()
                    # Only signal readiness once the deferred ext setup has
                    # actually succeeded, so the backbone doesn't proceed while
                    # the articulation is still initialising.
                    if self._extension.is_ready and not self._ready.is_set():
                        self._ready.set()
                        logger.info("Scene loaded; states will flow")
        except KeyboardInterrupt:
            logger.info("Shutting down.")
        finally:
            self._ready.clear()
            if self._extension is not None:
                # An extension shutdown failure must not strand the Isaac process —
                # timeline.stop + sim_app.close still need to run.
                try:
                    self._extension.shutdown()
                except Exception:
                    logger.exception("IsaacBridgeExtension shutdown failed")
            if self._timeline is not None:
                self._timeline.stop()
            self._sim_app.close()
            logger.info("Isaac Sim closed.")
