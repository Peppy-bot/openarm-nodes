#!/usr/bin/env python3
# pylint: disable=R0902,C0413
"""IsaacBridgeExtension owns the per-step bridge for the openarm Isaac scene.
Isaac's sim_app.update() advances physics on the main thread; each step this
extension applies the latest sim-passthrough setpoint per side, reads the
measured joint and gripper state, and (throttled to state_rate_hz) publishes it.
Transport is typed peppygen via SimTopicIO; there is no JSON and no raw
peppylib on the path.
"""
from __future__ import annotations

import gc
import logging
import time
from pathlib import Path

import pyjson5

from sim_topics import SimTopicIO
from exts import IsaacActuatorCtrl, IsaacArticulation, IsaacGripperSensor

logger = logging.getLogger(__name__)

_CONFIG_PATH = Path(__file__).resolve().parent / "config" / "sim_bridge.json5"
# The articulation root prim in the loaded USD stage. Every ext (state read,
# actuator write, gripper sensor) targets this one articulation. The robot USD's
# defaultPrim is /openarm (the launcher only adds /World/defaultDomeLight at
# runtime; the robot itself is not reparented under /World).
_ROOT_ARTICULATION_PRIM = "/openarm"


class IsaacBridgeExtension:
    """Drives the engine from the typed command streams and publishes state.

    Articulation setup is deferred to the first step() that succeeds: the
    Articulation views cannot initialise until the USD stage has loaded and the
    timeline is playing, which races the bridge's construction. Until every ext
    is ready, step() is a no-op except for the setup retry.
    """

    def __init__(self, io: SimTopicIO, state_rate_hz: int) -> None:
        self._io = io
        # Telemetry is throttled to state_rate_hz: serializing every reader at
        # the physics tick saturates the single sim thread. Writers and the
        # physics step still run every tick.
        self._telemetry_period_s = 1.0 / state_rate_hz if state_rate_hz > 0 else 0.0
        self._last_publish_s = 0.0

        cfg = pyjson5.loads(_CONFIG_PATH.read_text())
        self._arms: list[dict] = cfg["arms"]
        self._grippers: list[dict] = cfg["grippers"]
        self._gains: dict = cfg.get("arm_gains", {})

        self._articulation = IsaacArticulation(_ROOT_ARTICULATION_PRIM)
        # One actuator controller per arm side: the MIT gains and torque caps
        # are applied to that side's PhysX drives at setup, and the side's
        # commands are written through it. Fingers keep their USD drive defaults
        # and are written through a gainless controller.
        self._arm_actuators: dict[int, IsaacActuatorCtrl] = {
            arm["arm_id"]: IsaacActuatorCtrl(
                _ROOT_ARTICULATION_PRIM,
                joint_names=arm["joints"],
                params=self._actuator_params(arm["joints"]),
            )
            for arm in self._arms
        }
        self._gripper_actuators: dict[int, IsaacActuatorCtrl] = {
            gripper["gripper_id"]: IsaacActuatorCtrl(
                _ROOT_ARTICULATION_PRIM,
                joint_names=gripper["fingers"],
            )
            for gripper in self._grippers
        }
        self._gripper_sensors: dict[int, IsaacGripperSensor] = {
            gripper["gripper_id"]: IsaacGripperSensor(
                _ROOT_ARTICULATION_PRIM, finger_joints=gripper["fingers"]
            )
            for gripper in self._grippers
        }
        self._joint_index: dict[str, int] = {}
        self._ready: bool = False

    def _actuator_params(self, joints: list[str]) -> dict:
        return {
            "joint_names": joints,
            "kp": list(self._gains.get("kp", [])),
            "kd": list(self._gains.get("kd", [])),
            "max_efforts": list(self._gains.get("max_efforts", [])),
            "gravity_compensation": self._gains.get("gravity_compensation", False),
        }

    def _all_exts(self) -> list:
        return [
            self._articulation,
            *self._arm_actuators.values(),
            *self._gripper_actuators.values(),
            *self._gripper_sensors.values(),
        ]

    def _try_setup(self) -> bool:
        """Attempt to initialise every ext against the live stage. Returns True
        once all are ready; safe to call every step until it succeeds."""
        if self._ready:
            return True
        if not all(ext.setup() for ext in self._all_exts()):
            return False
        self._joint_index = {
            name: i for i, name in enumerate(self._articulation.get_joint_names())
        }
        self._ready = True
        logger.info(
            f"IsaacBridgeExtension ready — {len(self._arms)} arm(s), "
            f"{len(self._grippers)} gripper(s)"
        )
        return True

    def step(self) -> None:
        """Physics has already advanced in sim_app.update(); apply the latest
        commands and (throttled) publish measured state."""
        if not self._try_setup():
            return

        self._apply_commands()

        now = time.monotonic()
        if now - self._last_publish_s < self._telemetry_period_s:
            return
        self._last_publish_s = now
        self._publish_state()

    def _apply_commands(self) -> None:
        for arm in self._arms:
            command = self._io.latest_arm_command(arm["arm_id"])
            if command is None:
                continue
            positions, velocities = command
            joints = arm["joints"]
            if len(positions) != len(joints):
                continue
            velocity_values = (
                dict(zip(joints, velocities)) if len(velocities) == len(joints) else None
            )
            self._arm_actuators[arm["arm_id"]].write_targets(
                dict(zip(joints, positions)), velocity_values
            )

        for gripper in self._grippers:
            opening = self._io.latest_gripper_command(gripper["gripper_id"])
            if opening is None:
                continue
            # Both fingers hold half the aperture.
            per_finger = opening / 2.0
            self._gripper_actuators[gripper["gripper_id"]].write_targets(
                {f: per_finger for f in gripper["fingers"]}
            )

    def _publish_state(self) -> None:
        states = self._articulation.get_joint_states()
        if states is not None:
            positions, velocities = states
            for arm in self._arms:
                indices = [self._joint_index.get(name) for name in arm["joints"]]
                if any(i is None for i in indices):
                    continue
                self._io.publish_joint_states(
                    arm["arm_id"],
                    [positions[i] for i in indices],
                    [velocities[i] for i in indices],
                )

        for gripper_id, sensor in self._gripper_sensors.items():
            data = sensor.get_gripper_state()
            if data and data["positions"]:
                # Opening = total aperture = sum of finger positions.
                self._io.publish_gripper_states(gripper_id, float(sum(data["positions"])))

    def shutdown(self) -> None:
        logger.info("IsaacBridgeExtension shutting down.")
        for ext in self._all_exts():
            ext.teardown()
        gc.collect()
