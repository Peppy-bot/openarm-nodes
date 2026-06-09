#!/usr/bin/env python3
# pylint: disable=R0902,R0903,C0413
"""IsaacBridgeExtension drives per-step bridge plugins for the openarm Isaac
scene. Isaac's sim_app.update() advances physics; this extension only runs
the plugin loop.
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
from exts import (
    IsaacActuatorCtrl,
    IsaacArticulation,
    IsaacClockSensor,
    IsaacContactSensor,
    IsaacEePoseSensor,
    IsaacGripperSensor,
    IsaacImuSensor,
    IsaacSimControl,
    IsaacTransformTree,
    IsaacWrenchSensor,
)

logger = logging.getLogger(__name__)

_DEFAULT_NODE_NAME = "sim"
_DEFAULT_CONFIG_PATH = Path(__file__).resolve().parent / "config" / "sim_bridge.json5"
_ROOT_ARTICULATION_PRIM = "/World/openarm"

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


class IsaacBridgeExtension:

    def __init__(self) -> None:
        self._config: Optional[BridgeConfig] = None
        self._io: Optional[PeppylibIO] = None
        self._plugins: list = []
        self._sim_articulation: Optional[IsaacArticulation] = None
        self._sim_control: Optional[IsaacSimControl] = None
        self._step: int = 0

    def startup(self) -> None:
        """Load config, build plugins, register subscriptions, start I/O."""
        self._config = BridgeConfig.from_file(
            path=_DEFAULT_CONFIG_PATH,
            default_node_name=_DEFAULT_NODE_NAME,
        )
        _validate_config(self._config)
        self._io = PeppylibIO(self._config)
        self._plugins = _build_plugins(self._config)

        # SimControl is always present (not config-driven). Articulation
        # setup is deferred to first set_joint_positions call to avoid racing
        # the USD stage load. omni.timeline is imported lazily because
        # top-level omni.* breaks before SimulationApp initialises.
        import omni.timeline  # pylint: disable=E0401,C0415

        self._sim_articulation = IsaacArticulation(_ROOT_ARTICULATION_PRIM)
        self._sim_control = IsaacSimControl(
            articulation=self._sim_articulation,
            timeline=omni.timeline.get_timeline_interface(),
        )
        self._plugins.append(SimControlBridge(self._sim_control, self._config))

        for plugin in self._plugins:
            for source_node, topic, qos in plugin.subscriptions():
                self._io.register_subscription(source_node, topic, qos)

        self._io.start()
        self._step = 0
        logger.info(
            f"IsaacBridgeExtension ready — {len(self._plugins)} plugin(s) "
            f"daemon_node='{self._config.daemon_node}' node='{self._config.node_name}'"
        )

    def step(self) -> None:
        """Drive the plugin loop. When paused, only SimControlBridge runs so
        unpause/step/reset can still be processed."""
        if self._sim_control and self._sim_control.is_paused:
            for plugin in self._plugins:
                if isinstance(plugin, SimControlBridge):
                    plugin.on_step(self._step, self._io)
            return

        self._step += 1
        for plugin in self._plugins:
            if not plugin.is_ready and not plugin.try_setup():
                continue
            plugin.on_step(self._step, self._io)

    def shutdown(self) -> None:
        logger.info("IsaacBridgeExtension shutting down.")
        for plugin in self._plugins:
            plugin.teardown()
        if self._sim_articulation is not None:
            self._sim_articulation.teardown()
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


def _make_sensor(entry):  # pylint: disable=R0911
    """Return the right Isaac wrapper for a given entry. `entry.prim` is the
    USD prim path; for sensors that need extra params, read entry.params.
    """
    prim = entry.prim or ""
    if entry.type == "joint_states":
        return IsaacArticulation(prim)
    if entry.type == "imu":
        return IsaacImuSensor(prim)
    if entry.type == "tf_tree":
        return IsaacTransformTree(prim)
    if entry.type == "clock":
        return IsaacClockSensor()
    if entry.type == "ee_pose":
        return IsaacEePoseSensor(prim)
    if entry.type == "wrench":
        return IsaacWrenchSensor(prim)
    if entry.type == "contact_forces":
        return IsaacContactSensor(prim)
    if entry.type == "gripper_state":
        finger_joints = (
            entry.params.get("finger_joints", []) if hasattr(entry, "params") else []
        )
        return IsaacGripperSensor(prim, finger_joints=finger_joints)
    if entry.type == "actuator_ctrl":
        joint_names = (
            entry.params.get("joint_names", []) if hasattr(entry, "params") else []
        )
        return IsaacActuatorCtrl(prim, joint_names=joint_names)
    return IsaacArticulation(prim)


def _build_plugins(config: BridgeConfig) -> list:
    plugins: list = []
    for entry in config.publishers:
        cls = _PUBLISHER_REGISTRY[entry.type]
        sensor = _make_sensor(entry)
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
        sensor = _make_sensor(entry)
        plugins.append(cls(sensor, config, entry))
        logger.info(f"Registered subscriber: {entry.type} → topic='{entry.topic}'")

    return plugins
