from __future__ import annotations

import gc
import json
import logging
import os
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
_ENV_PRIM_PATH = "GRIPPER_PRIM_PATH"
_ENV_FINGER_JOINTS = "GRIPPER_FINGER_JOINTS"  # comma-separated DOF names


class IsaacGripperExtension:
    """Isaac Sim gripper extension — applies gripper position commands each physics step.

    Loaded by Omniverse Kit via extension.toml. Configuration is read from environment
    variables at on_startup time so the caller (Kit) does not need to pass arguments.
    """

    def on_startup(self, ext_id: str) -> None:  # noqa: ARG002
        """Called by Omniverse Kit when the extension is loaded."""
        # omni.* imports are deferred to respect Isaac Sim load order.
        self._config: Optional[BridgeConfig] = None
        self._io: Optional[PeppylibIO] = None
        self._gripper: Optional[IsaacGripperCommand] = None
        self._last_cmd_time: float = 0.0
        self._stale_warned: bool = False

        prim_path = os.environ.get(_ENV_PRIM_PATH, "")
        finger_joints = [
            j.strip()
            for j in os.environ.get(_ENV_FINGER_JOINTS, "").split(",")
            if j.strip()
        ]

        if not prim_path or not finger_joints:
            logger.error(
                f"omni.peppy.gripper: {_ENV_PRIM_PATH} and {_ENV_FINGER_JOINTS}"
                " must be set before loading this extension."
            )
            return

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
        except json.JSONDecodeError as exc:
            logger.warning(f"Failed to decode gripper command JSON: {exc}")
            return

        positions = payload.get("positions")
        if not isinstance(positions, list):
            logger.warning(
                f"Gripper command missing or invalid positions"
                f" (got {type(positions).__name__}: {positions!r}) — dropped."
            )
            return

        if self._gripper.apply(positions):
            self._last_cmd_time = time.monotonic()
            self._stale_warned = False

    def on_shutdown(self) -> None:
        """Called by Omniverse Kit when the extension is unloaded."""
        logger.info("omni.peppy.gripper shutting down.")
        if self._gripper is not None:
            self._gripper.teardown()
        if self._io is not None:
            self._io.stop()
        gc.collect()
