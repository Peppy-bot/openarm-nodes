from __future__ import annotations

import gc
import json
import logging
import time
from typing import Optional

# sim_ext_core is injected onto sys.path by __init__.py (extracted from backbone PR).
from sim_ext_core import BridgeConfig, PeppylibIO  # pylint: disable=E0401

from .impl.gripper_command import IsaacGripperCommand

logger = logging.getLogger(__name__)

_DEFAULT_NODE_NAME = "openarm01_gripper"
_COMMAND_TOPIC = "gripper_command"
_QOS = "standard"
_COMMAND_EXPIRY_S = 0.5


class IsaacGripperExtension:
    """Isaac Sim gripper extension — applies gripper position commands each physics step.

    Instantiated by the Isaac extension system. Subscribes to 'gripper_command' from
    the Rust sim_bridge via peppylib and drives finger DOFs via ArticulationView
    each simulation step.
    """

    def __init__(self) -> None:
        self._config: Optional[BridgeConfig] = None
        self._io: Optional[PeppylibIO] = None
        self._gripper: Optional[IsaacGripperCommand] = None
        self._last_cmd_time: float = 0.0
        self._stale_warned: bool = False

    def startup(self, prim_path: str, finger_joints: list[str]) -> None:
        """Build the gripper command handler, register subscription, start I/O."""
        self._config = BridgeConfig.from_env(default_node_name=_DEFAULT_NODE_NAME)
        self._io = PeppylibIO(self._config)
        self._gripper = IsaacGripperCommand(prim_path, finger_joints)

        self._io.register_subscription(self._config.sim_bridge_node, _COMMAND_TOPIC, _QOS)
        self._io.start()

        logger.info(
            f"omni.peppy.gripper ready — prim='{prim_path}'"
            f"  fingers={finger_joints}"
            f"  daemon_node='{self._config.daemon_node}'"
        )

    def on_physics_step(self, _step_size: float) -> None:
        """Drain the latest gripper command and apply to Isaac Sim articulation."""
        if self._io is None or self._gripper is None:
            return

        if not self._gripper.is_ready and not self._gripper.setup():
            return

        raw = self._io.get_latest(self._config.sim_bridge_node, _COMMAND_TOPIC)
        if raw is None:
            now = time.monotonic()
            if (
                self._last_cmd_time > 0
                and not self._stale_warned
                and now - self._last_cmd_time > _COMMAND_EXPIRY_S
            ):
                logger.warning(
                    f"No gripper command for >{_COMMAND_EXPIRY_S:.1f}s"
                    " — holding last position."
                )
                self._stale_warned = True
            return

        try:
            payload = json.loads(raw)
            positions = payload.get("positions")
            if isinstance(positions, list):
                self._last_cmd_time = time.monotonic()
                self._stale_warned = False
                self._gripper.apply(positions)
        except Exception as exc:
            logger.warning(f"Failed to apply gripper command: {exc}")

    def shutdown(self) -> None:
        """Tear down the gripper handler and stop I/O."""
        logger.info("omni.peppy.gripper shutting down.")
        if self._gripper is not None:
            self._gripper.teardown()
        if self._io is not None:
            self._io.stop()
        gc.collect()
