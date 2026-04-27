from __future__ import annotations

import gc
import logging
from typing import Optional

import omni.ext  # pylint: disable=E0401
import omni.timeline  # pylint: disable=E0401
from omni.physx import acquire_physx_interface  # pylint: disable=E0401

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

from .impl.articulation import ArticulationBridge
from .impl.clock_sensor import IsaacClockSensor
from .impl.contact_sensor import IsaacContactSensor
from .impl.ee_pose_sensor import IsaacEePoseSensor
from .impl.gripper_sensor import IsaacGripperSensor
from .impl.imu_sensor import IsaacImuSensor
from .impl.odometry_sensor import IsaacOdometrySensor
from .impl.sim_control import IsaacSimControl
from .impl.transform_tree import IsaacTransformTree
from .impl.wrench_sensor import IsaacWrenchSensor

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


class BackboneExtension(omni.ext.IExt):  # pylint: disable=E0611,I1101
    """Isaac Sim extension — bridges sensor topics from Isaac Sim to PeppyOS (read-only)."""

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
        self._sim_control_articulation: Optional[ArticulationBridge] = None
        self._sc_bridge: Optional[SimControlBridge] = None
        self._app_update_sub = None

    def on_startup(self, ext_id: str) -> None:
        import omni.kit.app  # pylint: disable=E0401 # noqa: PLC0415

        logger.info(f"omni.peppy.backbone starting (ext_id={ext_id}).")

        self._config = BridgeConfig.from_file(default_node_name=_DEFAULT_NODE_NAME)
        _validate_config(self._config, _PLUGIN_REGISTRY)
        self._io = PeppylibIO(self._config)
        self._plugins = _build_plugins(self._config)

        self._physx = acquire_physx_interface()
        self._timeline = omni.timeline.get_timeline_interface()

        # SimControl is always present — not config-driven.
        # Use the joint_states publisher prim as the articulation root.
        js_entry = next(
            (e for e in self._config.publishers if e.type == "joint_states"), None
        )
        if js_entry is None or not js_entry.prim:
            raise ValueError(
                "omni.peppy.backbone: no 'joint_states' publisher with a valid prim "
                "found in config — cannot initialise SimControl articulation."
            )
        self._sim_control_articulation = ArticulationBridge(js_entry.prim)
        sc = IsaacSimControl(self._sim_control_articulation, self._timeline)
        self._sc_bridge = SimControlBridge(sc, self._config)
        self._step = 0

        # Register IO subscriptions once at startup — avoids duplicate registrations
        # on repeated PLAY→STOP→PLAY cycles.
        for plugin in [*self._plugins, self._sc_bridge]:
            if plugin is None:
                continue
            for source_node, topic, qos in plugin.subscriptions():
                self._io.register_subscription(source_node, topic, qos)

        app = omni.kit.app.get_app()
        self._app_update_sub = app.get_update_event_stream().create_subscription_to_pop(
            self._on_app_update, name="peppy_backbone_simcontrol"
        )

        stream = self._timeline.get_timeline_event_stream()
        self._timeline_sub = stream.create_subscription_to_pop(
            self._on_timeline_event, name="peppy_backbone_timeline"
        )

        self._io.start()
        logger.info(
            f"omni.peppy.backbone ready — {len(self._plugins)} plugin(s) registered"
            f"  daemon_node='{self._config.daemon_node}'"
            f"  node='{self._config.node_name}'"
        )

    def on_shutdown(self) -> None:
        logger.info("omni.peppy.backbone shutting down.")
        self._app_update_sub = None
        self._timeline_sub = None
        self._physx_sub = None
        if self._sc_bridge is not None:
            self._sc_bridge.teardown()
        for plugin in self._plugins:
            plugin.teardown()
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
        self._physx_sub = self._physx.subscribe_physics_step_events(
            self._on_physics_step
        )
        logger.info("Physics step subscription registered.")

    def _on_stop(self) -> None:
        self._physx_sub = None
        for plugin in self._plugins:
            plugin.teardown()
        logger.info("Physics step subscription released.")

    def _on_app_update(self, _event) -> None:
        if self._sc_bridge is None:
            return
        if not self._sc_bridge.is_ready and not self._sc_bridge.try_setup():
            return
        self._sc_bridge.on_step(self._step, self._io)

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


def _make_sensor(entry):  # pylint: disable=R0911
    """Return the correct sensor object for a given entry type."""
    if entry.type == "imu":
        return IsaacImuSensor(
            entry.prim,
            **{
                k: entry.params[k]
                for k in ("translation", "orientation", "frequency")
                if k in entry.params
            },
        )
    if entry.type == "tf_tree":
        return IsaacTransformTree(entry.prim)
    if entry.type == "clock":
        return IsaacClockSensor()
    if entry.type == "ee_pose":
        return IsaacEePoseSensor(entry.prim)
    if entry.type == "odometry":
        return IsaacOdometrySensor(entry.prim)
    if entry.type == "wrench":
        return IsaacWrenchSensor(
            entry.prim,
            **{k: entry.params[k] for k in ("joint_index",) if k in entry.params},
        )
    if entry.type == "contact_forces":
        return IsaacContactSensor(
            entry.prim,
            **{
                k: entry.params[k]
                for k in ("min_threshold", "max_threshold", "radius")
                if k in entry.params
            },
        )
    if entry.type == "gripper_state":
        return IsaacGripperSensor(
            entry.prim,
            finger_joints=entry.params.get("finger_joints", []),
        )
    return ArticulationBridge(entry.prim)


def _build_plugins(config: BridgeConfig) -> list:
    """Instantiate publisher plugins from sim_bridge.json5 config entries."""
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
