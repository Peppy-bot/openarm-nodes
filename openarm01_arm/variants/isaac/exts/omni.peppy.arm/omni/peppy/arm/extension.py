from __future__ import annotations

import gc
import json
import logging
import os
from typing import Optional

import omni.ext  # pylint: disable=E0401

from sim_ext_core import BridgeConfig, PeppylibIO  # pylint: disable=E0401

from .impl.articulation import IsaacArmActuator

logger = logging.getLogger(__name__)

_DEFAULT_NODE_NAME = "arm"
_JOINT_COMMAND_TOPIC = "joint_command"
_QOS = "standard"


class ArmExtension(omni.ext.IExt):  # pylint: disable=E0611,I1101
    """Isaac Sim extension — applies joint position commands from PeppyOS to the arm articulation."""

    def __init__(self) -> None:
        super().__init__()
        self._config: Optional[BridgeConfig] = None
        self._io: Optional[PeppylibIO] = None
        self._actuator: Optional[IsaacArmActuator] = None
        self._physx = None
        self._timeline = None
        self._physx_sub = None
        self._timeline_sub = None
        self._step: int = 0
        self._arm_id: int = int(os.environ.get("PEPPY_ARM_ID", "0"))

    def on_startup(self, ext_id: str) -> None:
        logger.info(f"omni.peppy.arm starting (ext_id={ext_id}, arm_id={self._arm_id}).")

        self._config = BridgeConfig.from_file(default_node_name=_DEFAULT_NODE_NAME)
        self._io = PeppylibIO(self._config)

        # TODO: resolve prim path and joint names from config or env
        prim_path = os.environ.get("PEPPY_ARM_PRIM", "/openarm")
        self._actuator = IsaacArmActuator(prim_path, arm_id=self._arm_id)

        from omni.physx import acquire_physx_interface  # pylint: disable=E0401,C0415
        import omni.timeline  # pylint: disable=E0401,C0415

        self._physx = acquire_physx_interface()
        self._timeline = omni.timeline.get_timeline_interface()
        stream = self._timeline.get_timeline_event_stream()
        self._timeline_sub = stream.create_subscription_to_pop(
            self._on_timeline_event, name="peppy_arm_timeline"
        )
        self._io.start()
        logger.info("omni.peppy.arm ready.")

    def on_shutdown(self) -> None:
        logger.info("omni.peppy.arm shutting down.")
        self._timeline_sub = None
        self._physx_sub = None
        if self._actuator is not None:
            self._actuator.teardown()
        self._io.stop()
        gc.collect()

    def _on_timeline_event(self, event) -> None:
        import omni.timeline  # pylint: disable=E0401,C0415

        if int(event.type) == int(omni.timeline.TimelineEventType.PLAY):  # pylint: disable=I1101
            self._on_play()
        elif int(event.type) == int(omni.timeline.TimelineEventType.STOP):  # pylint: disable=I1101
            self._on_stop()

    def _on_play(self) -> None:
        if self._physx_sub is not None:
            return
        self._io.register_subscription(
            self._config.daemon_node, _JOINT_COMMAND_TOPIC, _QOS
        )
        self._step = 0
        self._physx_sub = self._physx.subscribe_physics_step_events(self._on_physics_step)
        logger.info("omni.peppy.arm: physics step subscription registered.")

    def _on_stop(self) -> None:
        self._physx_sub = None
        if self._actuator is not None:
            self._actuator.teardown()

    def _on_physics_step(self, _step_size: float) -> None:
        self._step += 1
        if self._actuator is None:
            return
        if not self._actuator.is_ready and not self._actuator.setup():
            return
        raw = self._io.get_latest(self._config.daemon_node, _JOINT_COMMAND_TOPIC)
        if raw is None:
            return
        try:
            msg = json.loads(raw)
            positions = msg.get("joint_positions", [])
            if positions:
                self._actuator.set_joint_positions(positions)
        except Exception as exc:
            logger.warning(f"Could not apply joint command: {exc}")
