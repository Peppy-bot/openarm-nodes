from __future__ import annotations

import gc
import logging
from typing import Optional

from sim_ext_core import (
    BridgeConfig,
    ClockBridge,
    ContactForcesBridge,
    EePoseBridge,
    GripperStateBridge,
    ImuBridge,
    JointStatesBridge,
    OdometryBridge,
    PeppylibIO,
    SimControlBridge,
    TfTreeBridge,
    WrenchBridge,
)  # pylint: disable=E0401

from .impl.articulation import MujocoArticulation
from .impl.clock_sensor import MujocoClockSensor
from .impl.contact_sensor import MujocoContactSensor
from .impl.ee_pose_sensor import MujocoEePoseSensor
from .impl.gripper_sensor import MujocoGripperSensor
from .impl.imu_sensor import MujocoImuSensor
from .impl.odometry_sensor import MujocoOdometrySensor
from .impl.sim_control import MujocoSimControl
from .impl.transform_tree import MujocoTransformTree
from .impl.wrench_sensor import MujocoWrenchSensor

logger = logging.getLogger(__name__)

_DEFAULT_NODE_NAME = "sim"
_PLUGIN_REGISTRY = {
    "joint_states": JointStatesBridge,
    "imu": ImuBridge,
    "tf_tree": TfTreeBridge,
    "clock": ClockBridge,
    "ee_pose": EePoseBridge,
    "odometry": OdometryBridge,
    "wrench": WrenchBridge,
    "contact_forces": ContactForcesBridge,
    "gripper_state": GripperStateBridge,
}


class MujocoBackboneExtension:
    """MuJoCo backbone — drives sensor topics to PeppyOS each physics step (read-only)."""

    def __init__(self, model, data) -> None:
        self._model = model
        self._data = data
        self._config: Optional[BridgeConfig] = None
        self._io: Optional[PeppylibIO] = None
        self._plugins: list = []
        self._sim_control: Optional[MujocoSimControl] = None
        self._step: int = 0

    def startup(self) -> None:
        """Load config, validate, build plugins, register subscriptions, start I/O."""
        self._config = BridgeConfig.from_file(default_node_name=_DEFAULT_NODE_NAME)
        _validate_config(self._config, _PLUGIN_REGISTRY)
        self._io = PeppylibIO(self._config)
        self._plugins = _build_plugins(self._config, self._model, self._data)

        # SimControl is always present — not config-driven.
        self._sim_control = MujocoSimControl(self._model, self._data)
        sc_bridge = SimControlBridge(self._sim_control, self._config)
        self._plugins.append(sc_bridge)

        for plugin in self._plugins:
            for source_node, topic, qos in plugin.subscriptions():
                self._io.register_subscription(source_node, topic, qos)

        self._io.start()
        self._step = 0
        logger.info(
            f"mujoco.peppy.backbone ready — {len(self._plugins)} plugin(s) registered"
            f"  daemon_node='{self._config.daemon_node}'"
            f"  node='{self._config.node_name}'"
        )

    def step(self) -> None:
        """Advance physics one step and drive all plugins.

        When paused, only SimControlBridge runs (to allow unpause/step/reset
        requests to be processed). All other plugins and mj_step are skipped.
        """
        if self._sim_control and self._sim_control.is_paused:
            for plugin in self._plugins:
                if isinstance(plugin, SimControlBridge):
                    plugin.on_step(self._step, self._io)
            return

        import mujoco  # pylint: disable=E0401

        mujoco.mj_step(self._model, self._data)
        mujoco.mj_rnePostConstraint(self._model, self._data)
        self._step += 1
        for plugin in self._plugins:
            if not plugin.is_ready and not plugin.try_setup():
                continue
            plugin.on_step(self._step, self._io)

    @property
    def is_paused(self) -> bool:
        return bool(self._sim_control and self._sim_control.is_paused)

    def shutdown(self) -> None:
        """Tear down plugins and stop I/O."""
        logger.info("mujoco.peppy.backbone shutting down.")
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


def _make_sensor(entry, model, data):  # pylint: disable=R0911
    """Return the correct sensor object for a given entry type."""
    body_name = entry.prim.split("/")[-1] if entry.prim else ""
    if entry.type == "imu":
        return MujocoImuSensor(model, data, body_name)
    if entry.type == "tf_tree":
        return MujocoTransformTree(model, data)
    if entry.type == "clock":
        return MujocoClockSensor(model, data)
    if entry.type == "ee_pose":
        return MujocoEePoseSensor(model, data, body_name)
    if entry.type == "odometry":
        return MujocoOdometrySensor(model, data, body_name)
    if entry.type == "wrench":
        return MujocoWrenchSensor(model, data, body_name)
    if entry.type == "contact_forces":
        return MujocoContactSensor(model, data, body_name)
    if entry.type == "gripper_state":
        return MujocoGripperSensor(
            model,
            data,
            finger_joints=(entry.params or {}).get("finger_joints", []),
        )
    return MujocoArticulation(model, data)


def _build_plugins(config: BridgeConfig, model, data) -> list:
    """Instantiate publisher plugins from sim_bridge.json5 config entries."""
    plugins = []
    for entry in config.publishers:
        cls = _PLUGIN_REGISTRY[entry.type]
        sensor = _make_sensor(entry, model, data)
        plugins.append(cls(sensor, config, entry))
        logger.info(f"Registered publisher: {entry.type} → topic='{entry.topic}'")
    return plugins
