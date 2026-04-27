from __future__ import annotations

import gc
import json
import logging
import time
from typing import Optional

from sim_ext_core import BridgeConfig, BridgePlugin, PeppylibIO  # pylint: disable=E0401

from .impl.camera_sensor import MujocoRgbCameraSensor

logger = logging.getLogger(__name__)

_DEFAULT_NODE_NAME = "uvc_camera"
_DEFAULT_PUBLISH_EVERY_N_STEPS: int = 2
_QOS = "sensor_data"
_ENCODING = "rgb8"
_FRAME_ID = 0


class VideoStreamBridge(BridgePlugin):
    """Captures RGB frames and emits video_stream via peppylib (mujoco variant)."""

    def __init__(self, sensor: MujocoRgbCameraSensor, config: BridgeConfig, entry) -> None:
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


class MujocoUvcCameraExtension:
    """MuJoCo uvc_camera — drives camera frames to PeppyOS each physics step (read-only)."""

    def __init__(self, model, data) -> None:
        self._model = model
        self._data = data
        self._config: Optional[BridgeConfig] = None
        self._io: Optional[PeppylibIO] = None
        self._plugins: list = []
        self._step: int = 0

    def startup(self) -> None:
        self._config = BridgeConfig.from_file(default_node_name=_DEFAULT_NODE_NAME)
        _validate_config(self._config, _PLUGIN_REGISTRY)
        self._io = PeppylibIO(self._config)
        self._plugins = _build_plugins(self._config, self._model, self._data)

        self._io.start()
        self._step = 0
        logger.info(
            f"mujoco.peppy.uvc_camera ready — {len(self._plugins)} plugin(s) registered"
            f"  daemon_node='{self._config.daemon_node}'"
            f"  node='{self._config.node_name}'"
        )

    def step(self) -> None:
        self._step += 1
        for plugin in self._plugins:
            if not plugin.is_ready and not plugin.try_setup():
                continue
            plugin.on_step(self._step, self._io)

    def shutdown(self) -> None:
        logger.info("mujoco.peppy.uvc_camera shutting down.")
        for plugin in self._plugins:
            plugin.teardown()
        if self._io is not None:
            self._io.stop()
        gc.collect()


def _validate_config(config: BridgeConfig, registry: dict) -> None:
    """Raise ValueError for any unknown publisher type at startup."""
    known = sorted(registry)
    for entry in config.publishers:
        if entry.type not in registry:
            raise ValueError(
                f"Unknown publisher type '{entry.type}' in sim_bridge.json5."
                f" Supported: {known}"
            )


def _make_sensor(entry, model, data) -> MujocoRgbCameraSensor:
    # prim is the camera name in the MuJoCo model (last path segment for consistency)
    camera_name = entry.prim.split("/")[-1] if entry.prim else "camera"
    return MujocoRgbCameraSensor(
        model=model,
        data=data,
        camera_name=camera_name,
        **{k: entry.params[k] for k in ("width", "height") if k in entry.params},
    )


def _build_plugins(config: BridgeConfig, model, data) -> list:
    plugins = []
    for entry in config.publishers:
        cls = _PLUGIN_REGISTRY[entry.type]
        sensor = _make_sensor(entry, model, data)
        plugins.append(cls(sensor, config, entry))
        logger.info(
            f"Registered publisher: {entry.type}"
            f" → camera='{entry.prim}' topic='{entry.topic}'"
        )
    return plugins
