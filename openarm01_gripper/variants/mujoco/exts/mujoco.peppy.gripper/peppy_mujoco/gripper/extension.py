from __future__ import annotations

import gc
import json
import logging
import time
from typing import Optional

# sim_ext_core is injected onto sys.path by __init__.py (extracted from backbone PR).
from sim_ext_core import BridgeConfig, PeppylibIO  # pylint: disable=E0401

from .impl.gripper_command import MujocoGripperCommand

logger = logging.getLogger(__name__)

_DEFAULT_NODE_NAME = "openarm01_gripper"
_COMMAND_TOPIC = "gripper_command"
_QOS = "standard"
_COMMAND_EXPIRY_S = 0.5


class MujocoGripperExtension:
    """MuJoCo gripper extension — applies gripper position commands each physics step.

    Instantiated by the MuJoCo sim runner alongside backbone and arm extensions.
    Subscribes to 'gripper_command' from the Rust sim_bridge via peppylib and writes
    desired finger positions to data.ctrl each step.
    """

    def __init__(self, model, data) -> None:
        self._model = model
        self._data = data
        self._config: Optional[BridgeConfig] = None
        self._io: Optional[PeppylibIO] = None
        self._gripper: Optional[MujocoGripperCommand] = None
        self._last_cmd_time: float = 0.0
        self._stale_warned: bool = False

    def startup(self, finger_joints: list[str]) -> None:
        """Build the gripper command handler, register subscription, start I/O."""
        self._config = BridgeConfig.from_env(default_node_name=_DEFAULT_NODE_NAME)
        self._io = PeppylibIO(self._config)
        self._gripper = MujocoGripperCommand(self._model, self._data, finger_joints)

        self._io.register_subscription(self._config.sim_bridge_node, _COMMAND_TOPIC, _QOS)
        self._io.start()

        logger.info(
            f"mujoco.peppy.gripper ready — fingers={finger_joints}"
            f"  daemon_node='{self._config.daemon_node}'"
        )

    def step(self) -> None:
        """Drain the latest gripper command and apply it to MuJoCo finger actuators."""
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
        logger.info("mujoco.peppy.gripper shutting down.")
        if self._gripper is not None:
            self._gripper.teardown()
        if self._io is not None:
            self._io.stop()
        gc.collect()
