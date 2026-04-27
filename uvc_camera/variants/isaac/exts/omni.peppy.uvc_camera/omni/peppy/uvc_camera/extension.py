from __future__ import annotations

import gc
import json
import logging
import time
from typing import Optional

import omni.ext  # pylint: disable=E0401
import omni.timeline  # pylint: disable=E0401
from omni.physx import acquire_physx_interface  # pylint: disable=E0401

from sim_ext_core import BridgeConfig, BridgePlugin, PeppylibIO  # pylint: disable=E0401

from .impl.camera_sensor import IsaacRgbCameraSensor

logger = logging.getLogger(__name__)

_DEFAULT_NODE_NAME = "uvc_camera"
_DEFAULT_PUBLISH_EVERY_N_STEPS: int = 2
_QOS = "sensor_data"
_ENCODING = "rgb8"
_FRAME_ID = 0


class VideoStreamBridge(BridgePlugin):
    """Captures RGB frames and emits video_stream via peppylib (isaac variant)."""

    def __init__(self, sensor: IsaacRgbCameraSensor, config: BridgeConfig, entry) -> None:
        self._sensor = sensor
        self._node_name = config.node_name
        self._topic = entry.topic
        self._publish_every_n_steps: int = int(
            entry.params.get("publish_every_n_steps", _DEFAULT_PUBLISH_EVERY_N_STEPS)
        )

    def setup(self) -> bool:
        return self._sensor.setup()

    def teardown(self) -> None:
        self._sensor.teardown()

    def on_step(self, step: int, io) -> None:
        if step % self._publish_every_n_steps != 0:
            return
        frame_bytes = self._sensor.get_frame()
        if frame_bytes is None:
            return
        payload = json.dumps(
            {
                "header": {"stamp": time.monotonic(), "frame_id": _FRAME_ID},
                "encoding": _ENCODING,
                "width": self._sensor.width,
                "height": self._sensor.height,
                "frame": list(frame_bytes),
            }
        ).encode()
        io.emit(self._node_name, self._topic, _QOS, payload)

    @property
    def is_ready(self) -> bool:
        return self._sensor.is_ready


_PLUGIN_REGISTRY = {
    "rgb_camera": VideoStreamBridge,
}


class UvcCameraExtension(omni.ext.IExt):  # pylint: disable=E0611,I1101
    """Isaac Sim extension — bridges camera frames from Isaac Sim to PeppyOS (read-only)."""

    def __init__(self) -> None:
        super().__init__()
        self._config: Optional[BridgeConfig] = None
        self._io: Optional[PeppylibIO] = None
        self._plugins: list = []
        self._physx = None
        self._timeline = None
        self._physx_sub = None
        self._timeline_sub = None
        self._step: int = 0

    def on_startup(self, ext_id: str) -> None:
        logger.info(f"omni.peppy.uvc_camera starting (ext_id={ext_id}).")

        self._config = BridgeConfig.from_file(default_node_name=_DEFAULT_NODE_NAME)
        _validate_config(self._config, _PLUGIN_REGISTRY)
        self._io = PeppylibIO(self._config)
        self._plugins = _build_plugins(self._config)

        self._physx = acquire_physx_interface()
        self._timeline = omni.timeline.get_timeline_interface()
        stream = self._timeline.get_timeline_event_stream()
        self._timeline_sub = stream.create_subscription_to_pop(
            self._on_timeline_event, name="peppy_uvc_camera_timeline"
        )

        self._io.start()
        logger.info(
            f"omni.peppy.uvc_camera ready — {len(self._plugins)} plugin(s) registered"
            f" daemon_node='{self._config.daemon_node}'"
            f" node='{self._config.node_name}'"
        )

    def on_shutdown(self) -> None:
        logger.info("omni.peppy.uvc_camera shutting down.")
        self._timeline_sub = None
        self._physx_sub = None
        for plugin in self._plugins:
            plugin.teardown()
        if self._io is not None:
            self._io.stop()
        gc.collect()

    def _on_timeline_event(self, event) -> None:
        play = int(omni.timeline.TimelineEventType.PLAY)  # pylint: disable=I1101
        stop = int(omni.timeline.TimelineEventType.STOP)  # pylint: disable=I1101
        if int(event.type) == play:
            self._on_play()
        elif int(event.type) == stop:
            self._on_stop()

    def _on_play(self) -> None:
        if self._physx_sub is not None:
            return
        self._step = 0
        self._physx_sub = self._physx.subscribe_physics_step_events(self._on_physics_step)
        logger.info("Physics step subscription registered.")

    def _on_stop(self) -> None:
        self._physx_sub = None
        for plugin in self._plugins:
            plugin.teardown()
        logger.info("Physics step subscription released.")

    def _on_physics_step(self, _step_size: float) -> None:
        self._step += 1
        for plugin in self._plugins:
            if not plugin.is_ready and not plugin.try_setup():
                continue
            plugin.on_step(self._step, self._io)


def _validate_config(config: BridgeConfig, registry: dict) -> None:
    """Raise ValueError for any unknown publisher type at startup."""
    known = sorted(registry)
    for entry in config.publishers:
        if entry.type not in registry:
            raise ValueError(
                f"Unknown publisher type '{entry.type}' in sim_bridge.json5."
                f" Supported: {known}"
            )


def _make_sensor(entry) -> IsaacRgbCameraSensor:
    return IsaacRgbCameraSensor(
        prim_path=entry.prim,
        **{
            k: entry.params[k]
            for k in (
                "width", "height", "fx", "fy", "pixel_size_mm",
                "focus_distance", "f_stop", "clipping_near", "clipping_far",
            )
            if k in entry.params
        },
    )


def _build_plugins(config: BridgeConfig) -> list:
    plugins = []
    for entry in config.publishers:
        cls = _PLUGIN_REGISTRY[entry.type]
        sensor = _make_sensor(entry)
        plugins.append(cls(sensor, config, entry))
        logger.info(
            f"Registered publisher: {entry.type}"
            f" → prim='{entry.prim}' topic='{entry.topic}'"
        )
    return plugins
