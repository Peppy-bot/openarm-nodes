#!/usr/bin/env python3
# pylint: disable=R0902,R0903,C0413
"""MujocoBridgeExtension — owns the physics tick and the per-step bridge
plugin loop for the openarm bimanual scene.

The extension is instantiated by _launcher.py once MuJoCo's model+data
exist. startup() loads config/sim_bridge.json5, builds the plugin set,
and starts the peppylib daemon connection. step() advances physics and
drives every plugin on each tick.
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
    PeppylibIO,
    SimControlBridge,
    TfTreeBridge,
    WrenchBridge,
)
from sim_ext_core.mujoco import (
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

_PLUGIN_REGISTRY: dict = {
    "joint_states":    JointStatesBridge,
    "imu":             ImuBridge,
    "tf_tree":         TfTreeBridge,
    "clock":           ClockBridge,
    "ee_pose":         EePoseBridge,
    "wrench":          WrenchBridge,
    "contact_forces":  ContactForcesBridge,
    "gripper_state":   GripperStateBridge,
    "actuator_ctrl":   ActuatorCtrlBridge,
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
            path=_DEFAULT_CONFIG_PATH, default_node_name=_DEFAULT_NODE_NAME,
        )
        _validate_config(self._config, _PLUGIN_REGISTRY)
        self._io = PeppylibIO(self._config)
        self._plugins = _build_plugins(self._config, self._model, self._data)

        # SimControl is always present — not config-driven.
        self._sim_control = MujocoSimControl(self._model, self._data)
        self._plugins.append(SimControlBridge(self._sim_control, self._config))

        for plugin in self._plugins:
            for source_node, topic, qos in plugin.subscriptions():
                self._io.register_subscription(source_node, topic, qos)

        # Split plugins by direction so the step loop can run subscribers
        # (which write ctrl[] / qpos) before mj_step and publishers (which
        # read post-step state) after. SimControlBridge has subscriptions
        # so it lands on the writer side — its set_joint_positions dispatch
        # writes qpos and must precede the physics step.
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

        When paused, only SimControlBridge runs so unpause/step/reset
        requests can still be processed.
        """
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


def _validate_config(config: BridgeConfig, registry: dict) -> None:
    known = sorted(registry)
    for entry in config.publishers:
        if entry.type not in registry:
            raise ValueError(
                f"Unknown publisher type '{entry.type}' in sim_bridge.json5. "
                f"Supported: {known}"
            )
    for entry in config.subscribers:
        if entry.type not in registry:
            raise ValueError(
                f"Unknown subscriber type '{entry.type}' in sim_bridge.json5. "
                f"Supported: {known}"
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
        finger_joints = entry.params.get("finger_joints", []) if hasattr(entry, "params") else []
        return MujocoGripperSensor(model, data, finger_joints=finger_joints)
    if entry.type == "actuator_ctrl":
        return MujocoActuatorCtrl(model, data)
    return MujocoArticulation(model, data)  # joint_states default


def _build_plugins(config: BridgeConfig, model, data) -> list:
    plugins: list = []
    for entry in config.publishers:
        cls = _PLUGIN_REGISTRY[entry.type]
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
        cls = _PLUGIN_REGISTRY[entry.type]
        sensor = _make_sensor(entry, model, data)
        plugins.append(cls(sensor, config, entry))
        logger.info(f"Registered subscriber: {entry.type} → topic='{entry.topic}'")

    return plugins
