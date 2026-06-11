#!/usr/bin/env python3
# pylint: disable=R0902,R0903,C0413
"""MujocoBridgeExtension owns the physics tick and the per-step bridge plugin
loop for the openarm scene. startup() loads config/sim_bridge.json5 and builds
the plugin set; step() advances physics and drives every plugin on each tick.
"""
from __future__ import annotations

import gc
import logging
from pathlib import Path
from typing import Optional

from sim_ext_core import (
    ActuatorCtrlBridge,
    BridgeConfig,
    ClockBridge,
    ContactForcesBridge,
    EePoseBridge,
    GripperStateBridge,
    ImuBridge,
    JointStatesBridge,
    SimControlBridge,
    TfTreeBridge,
    WrenchBridge,
)
from peppylib_io import PeppylibIO
from exts import (
    MujocoActuatorCtrl,
    MujocoArticulation,
    MujocoClockSensor,
    MujocoContactSensor,
    MujocoEePoseSensor,
    MujocoGripperSensor,
    MujocoImuSensor,
    MujocoSimControl,
    MujocoTransformTree,
    MujocoWrenchSensor,
)

logger = logging.getLogger(__name__)

_DEFAULT_NODE_NAME = "sim"
_DEFAULT_CONFIG_PATH = Path(__file__).resolve().parent / "config" / "sim_bridge.json5"

# Split by direction so a misplaced sim_bridge.json5 entry fails validation:
# a single registry would let "joint_states" in subscribers instantiate as a
# subscriber and silently publish.
_PUBLISHER_REGISTRY: dict = {
    "joint_states": JointStatesBridge,
    "imu": ImuBridge,
    "tf_tree": TfTreeBridge,
    "clock": ClockBridge,
    "ee_pose": EePoseBridge,
    "wrench": WrenchBridge,
    "contact_forces": ContactForcesBridge,
    "gripper_state": GripperStateBridge,
}
_SUBSCRIBER_REGISTRY: dict = {
    "actuator_ctrl": ActuatorCtrlBridge,
}


class MujocoBridgeExtension:

    def __init__(self, model, data) -> None:
        self._model = model
        self._data = data
        self._config: Optional[BridgeConfig] = None
        self._io: Optional[PeppylibIO] = None
        self._plugins: list = []
        self._writers: list = []
        self._readers: list = []
        self._sim_control: Optional[MujocoSimControl] = None
        self._step: int = 0

    def startup(self) -> None:
        """Load config, build plugins, register subscriptions, start I/O."""
        self._config = BridgeConfig.from_file(
            path=_DEFAULT_CONFIG_PATH,
            default_node_name=_DEFAULT_NODE_NAME,
        )
        _validate_config(self._config)
        self._io = PeppylibIO(self._config)
        self._plugins = _build_plugins(self._config, self._model, self._data)

        # SimControl is always present — not config-driven.
        self._sim_control = MujocoSimControl(self._model, self._data)
        self._plugins.append(SimControlBridge(self._sim_control, self._config))

        for plugin in self._plugins:
            for source_node, topic, qos in plugin.subscriptions():
                self._io.register_subscription(source_node, topic, qos)

        # Writers (subscribers) run before mj_step; readers (publishers) after.
        # SimControlBridge has subscriptions, so its set_joint_positions writes
        # qpos on the writer side before the step consumes it.
        self._writers = [p for p in self._plugins if p.subscriptions()]
        self._readers = [p for p in self._plugins if not p.subscriptions()]

        self._io.start()
        self._step = 0
        logger.info(
            f"MujocoBridgeExtension ready — {len(self._plugins)} plugin(s) "
            f"daemon_node='{self._config.daemon_node}' node='{self._config.node_name}'"
        )

    def step(self) -> None:
        """Drive subscriber plugins, advance physics, drive publisher plugins.
        When paused, only SimControlBridge runs so unpause/step/reset can still
        be processed."""
        if self._sim_control and self._sim_control.is_paused:
            for plugin in self._plugins:
                if isinstance(plugin, SimControlBridge):
                    plugin.on_step(self._step, self._io)
            return

        import mujoco  # pylint: disable=C0415

        # Writers (subscribers) first: actuator_ctrl writes ctrl[] and
        # sim_control's set_joint_positions writes qpos before mj_step
        # consumes them — otherwise the values land one tick late.
        for plugin in self._writers:
            if not plugin.is_ready and not plugin.try_setup():
                continue
            plugin.on_step(self._step, self._io)

        mujoco.mj_step(self._model, self._data)
        self._step += 1

        # Readers (publishers) after mj_step: state-emitting plugins read
        # the post-step world.
        for plugin in self._readers:
            if not plugin.is_ready and not plugin.try_setup():
                continue
            plugin.on_step(self._step, self._io)

    def shutdown(self) -> None:
        logger.info("MujocoBridgeExtension shutting down.")
        for plugin in self._plugins:
            plugin.teardown()
        if self._io is not None:
            self._io.stop()
        gc.collect()


def _validate_config(config: BridgeConfig) -> None:
    for entry in config.publishers:
        if entry.type not in _PUBLISHER_REGISTRY:
            raise ValueError(
                f"Unknown publisher type '{entry.type}' in sim_bridge.json5. "
                f"Supported publishers: {sorted(_PUBLISHER_REGISTRY)}"
            )
    for entry in config.subscribers:
        if entry.type not in _SUBSCRIBER_REGISTRY:
            raise ValueError(
                f"Unknown subscriber type '{entry.type}' in sim_bridge.json5. "
                f"Supported subscribers: {sorted(_SUBSCRIBER_REGISTRY)}"
            )


def _make_sensor(entry, model, data):  # pylint: disable=R0911
    body_name = entry.prim.split("/")[-1] if entry.prim else ""
    if entry.type == "imu":
        return MujocoImuSensor(model, data, body_name)
    if entry.type == "tf_tree":
        return MujocoTransformTree(model, data)
    if entry.type == "clock":
        return MujocoClockSensor(model, data)
    if entry.type == "ee_pose":
        return MujocoEePoseSensor(model, data, body_name)
    if entry.type == "wrench":
        return MujocoWrenchSensor(model, data, body_name)
    if entry.type == "contact_forces":
        return MujocoContactSensor(model, data, body_name)
    if entry.type == "gripper_state":
        finger_joints = (
            entry.params.get("finger_joints", []) if hasattr(entry, "params") else []
        )
        return MujocoGripperSensor(model, data, finger_joints=finger_joints)
    if entry.type == "actuator_ctrl":
        params = entry.params if hasattr(entry, "params") else {}
        return MujocoActuatorCtrl(model, data, params=params)
    return MujocoArticulation(model, data)  # joint_states default


def _build_plugins(config: BridgeConfig, model, data) -> list:
    plugins: list = []
    for entry in config.publishers:
        cls = _PUBLISHER_REGISTRY[entry.type]
        sensor = _make_sensor(entry, model, data)
        plugins.append(cls(sensor, config, entry))
        logger.info(f"Registered publisher: {entry.type} → topic='{entry.topic}'")

    seen_subscribers: set = set()
    for entry in config.subscribers:
        key = (entry.type, entry.prim, entry.topic)
        if key in seen_subscribers:
            logger.debug(
                f"Skipping duplicate subscriber: {entry.type} → topic='{entry.topic}'"
            )
            continue
        seen_subscribers.add(key)
        cls = _SUBSCRIBER_REGISTRY[entry.type]
        sensor = _make_sensor(entry, model, data)
        plugins.append(cls(sensor, config, entry))
        logger.info(f"Registered subscriber: {entry.type} → topic='{entry.topic}'")

    return plugins
