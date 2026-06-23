#!/usr/bin/env python3
# pylint: disable=C0413
"""MujocoBridgeExtension owns the physics tick for the openarm scene. Each step
it applies the latest sim-passthrough setpoint per side, advances physics, and
(throttled to state_rate_hz) publishes the measured joint and gripper state.
Transport is typed peppygen via SimTopicIO; there is no JSON and no raw
peppylib on the path.
"""
from __future__ import annotations

import logging
import time
from pathlib import Path

import pyjson5

from sim_topics import SimTopicIO
from exts import MujocoActuatorCtrl, MujocoArticulation, MujocoGripperSensor

logger = logging.getLogger(__name__)

_CONFIG_PATH = Path(__file__).resolve().parent / "config" / "sim_bridge.json5"


class MujocoBridgeExtension:
    """Drives the engine from the typed command streams and publishes state."""

    def __init__(self, model, data, io: SimTopicIO, state_rate_hz: int) -> None:
        self._model = model
        self._data = data
        self._io = io
        # Telemetry is throttled to state_rate_hz: serializing every reader at
        # the ~500 Hz physics tick saturates the single sim thread. Writers and
        # the physics step still run every tick.
        if state_rate_hz <= 0:
            raise ValueError(f"state_rate_hz must be positive, got {state_rate_hz}")
        self._telemetry_period_s = 1.0 / state_rate_hz
        self._last_publish_s = 0.0

        cfg = pyjson5.loads(_CONFIG_PATH.read_text())
        self._arms: list[dict] = cfg["arms"]
        self._grippers: list[dict] = cfg["grippers"]
        self._gains: dict = cfg.get("arm_gains", {})

        self._articulation = MujocoArticulation(model, data)
        # One actuator controller for the whole robot: it resolves every actuator
        # by joint name, applies the MIT gains to the arm joints, and leaves the
        # finger joints on their MJCF defaults.
        self._actuator = MujocoActuatorCtrl(model, data, params=self._actuator_params())
        self._gripper_sensors: dict[int, MujocoGripperSensor] = {}
        self._joint_index: dict[str, int] = {}

    def _actuator_params(self) -> dict:
        arm_joints = [name for arm in self._arms for name in arm["joints"]]
        # Same per-joint gains for each arm (j1..j7), repeated per side.
        kp = list(self._gains.get("kp", [])) * len(self._arms)
        kd = list(self._gains.get("kd", [])) * len(self._arms)
        return {
            "joint_names": arm_joints,
            "kp": kp,
            "kd": kd,
            "gravity_compensation": self._gains.get("gravity_compensation", False),
        }

    def startup(self) -> None:
        if not self._articulation.setup():
            raise RuntimeError("MujocoArticulation setup failed")
        self._joint_index = {
            name: i for i, name in enumerate(self._articulation.get_joint_names())
        }
        # Fail loudly on a sim_bridge.json5 typo: a joint the model doesn't have
        # would otherwise silently drop that side's commands + telemetry.
        configured = [j for arm in self._arms for j in arm["joints"]] + [
            f for g in self._grippers for f in g["fingers"]
        ]
        missing = sorted({n for n in configured if n not in self._joint_index})
        if missing:
            raise RuntimeError(
                f"sim_bridge.json5 references joints not in the MuJoCo model: {missing}"
            )
        if not self._actuator.setup():
            raise RuntimeError("MujocoActuatorCtrl setup failed")
        for gripper in self._grippers:
            sensor = MujocoGripperSensor(
                self._model, self._data, finger_joints=gripper["fingers"]
            )
            if not sensor.setup():
                raise RuntimeError(
                    f"MujocoGripperSensor setup failed for gripper_id={gripper['gripper_id']}"
                )
            self._gripper_sensors[gripper["gripper_id"]] = sensor
        logger.info(
            f"MujocoBridgeExtension ready with {len(self._arms)} arm(s), "
            f"{len(self._grippers)} gripper(s)"
        )

    def step(self) -> None:
        import mujoco  # pylint: disable=C0415

        self._apply_commands()
        mujoco.mj_step(self._model, self._data)

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
            self._actuator.write_targets(dict(zip(joints, positions)), velocity_values)

        for gripper in self._grippers:
            opening = self._io.latest_gripper_command(gripper["gripper_id"])
            if opening is None:
                continue
            # Both fingers hold half the aperture.
            per_finger = opening / 2.0
            self._actuator.write_targets({f: per_finger for f in gripper["fingers"]})

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
        logger.info("MujocoBridgeExtension shutting down.")
        self._articulation.teardown()
        self._actuator.teardown()
        for sensor in self._gripper_sensors.values():
            sensor.teardown()
