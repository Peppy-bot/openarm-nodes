"""Typed peppygen topic IO for the openarm sim.

Bridges the physics thread (sync, runs mj_step in an executor) to the
node_runner's asyncio loop, where peppygen topic pub/sub lives. Consume tasks on
the loop each hold one generated `subscribe()` subscription (gap-free, in-order)
and keep the latest actuator command per side in thread-safe slots; the physics
thread reads those and publishes measured state back through the loop. Every
hop is a generated peppygen topic: no JSON, no raw peppylib.
"""

from __future__ import annotations

import asyncio
import logging
import math
import threading
from typing import Optional

import peppylib
from peppygen.emitted_topics.openarm_arm_states.v1 import arm_states
from peppygen.emitted_topics.openarm_gripper_states.v1 import gripper_states
from peppygen.consumed_topics import left_arm_cmd_arm_sim_passthrough as left_arm_cmd
from peppygen.consumed_topics import left_gripper_cmd_gripper_sim_passthrough as left_gripper_cmd
from peppygen.consumed_topics import right_arm_cmd_arm_sim_passthrough as right_arm_cmd
from peppygen.consumed_topics import right_gripper_cmd_gripper_sim_passthrough as right_gripper_cmd

logger = logging.getLogger(__name__)

# Left = 0, right = 1, matching arm_id / gripper_id across the stack.
_SIDES = (0, 1)


class _LatestSlot:
    """Thread-safe latest-wins single value, written on the loop and read on the
    physics thread."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._value = None

    def set(self, value) -> None:
        with self._lock:
            self._value = value

    def get(self):
        with self._lock:
            return self._value


class SimTopicIO:
    """Owns the typed publishers + command-consume tasks on the node loop, and
    exposes thread-safe accessors the physics thread calls each step."""

    def __init__(self, node_runner: peppylib.NodeRunner, loop: asyncio.AbstractEventLoop) -> None:
        self._node_runner = node_runner
        self._loop = loop
        self._arm_pub: Optional[peppylib.TopicPublisher] = None
        self._gripper_pub: Optional[peppylib.TopicPublisher] = None
        self._arm_cmd = {side: _LatestSlot() for side in _SIDES}
        self._gripper_cmd = {side: _LatestSlot() for side in _SIDES}
        self._tasks: list[asyncio.Task] = []

    async def start(self) -> None:
        """Declare publishers and spawn the command-consume loops. Runs on the
        node loop before the sim thread starts."""
        self._arm_pub = await arm_states.declare_publisher(self._node_runner)
        self._gripper_pub = await gripper_states.declare_publisher(self._node_runner)
        self._tasks = [
            asyncio.create_task(self._consume_arm(left_arm_cmd, 0, "left_arm_cmd")),
            asyncio.create_task(self._consume_arm(right_arm_cmd, 1, "right_arm_cmd")),
            asyncio.create_task(self._consume_gripper(left_gripper_cmd, 0, "left_gripper_cmd")),
            asyncio.create_task(self._consume_gripper(right_gripper_cmd, 1, "right_gripper_cmd")),
        ]

    async def stop(self) -> None:
        for task in self._tasks:
            task.cancel()
        # Let the cancellations land so the consume loops exit before teardown.
        await asyncio.gather(*self._tasks, return_exceptions=True)

    async def _consume_arm(self, topic, expected_arm_id: int, slot_name: str) -> None:
        subscription = await topic.subscribe(self._node_runner)
        while True:
            try:
                pair = await subscription.next()
                if pair is None:
                    return
            except asyncio.CancelledError:
                return
            except Exception as exc:
                # A corrupt frame is dropped and logged rather than killing
                # this consume task; the pause keeps a persistent fault from
                # hot-spinning the loop.
                logger.warning(f"{slot_name} command consume error: {exc}")
                await asyncio.sleep(0.1)
                continue
            _producer, msg = pair
            # The slot fixes the side; a command tagged for the other arm is a
            # mis-binding, not a routing hint.
            if msg.arm_id != expected_arm_id:
                logger.warning(
                    f"dropping arm command with arm_id={msg.arm_id} on {slot_name}"
                )
                continue
            # Drop a poisoned command rather than writing NaN/Inf into the sim.
            if not all(math.isfinite(v) for v in msg.positions) or not all(
                math.isfinite(v) for v in msg.velocities
            ):
                logger.warning(f"dropping non-finite arm command for arm_id={msg.arm_id}")
                continue
            self._arm_cmd[expected_arm_id].set((msg.positions, msg.velocities))

    async def _consume_gripper(self, topic, expected_gripper_id: int, slot_name: str) -> None:
        subscription = await topic.subscribe(self._node_runner)
        while True:
            try:
                pair = await subscription.next()
                if pair is None:
                    return
            except asyncio.CancelledError:
                return
            except Exception as exc:
                # A corrupt frame is dropped and logged rather than killing
                # this consume task; the pause keeps a persistent fault from
                # hot-spinning the loop.
                logger.warning(f"{slot_name} command consume error: {exc}")
                await asyncio.sleep(0.1)
                continue
            _producer, msg = pair
            if msg.gripper_id != expected_gripper_id:
                logger.warning(
                    f"dropping gripper command with gripper_id={msg.gripper_id} on {slot_name}"
                )
                continue
            if not math.isfinite(msg.opening):
                logger.warning(
                    f"dropping non-finite gripper command for gripper_id={msg.gripper_id}"
                )
                continue
            self._gripper_cmd[expected_gripper_id].set(msg.opening)

    # --- called from the physics thread ---

    def latest_arm_command(self, arm_id: int) -> Optional[tuple[list[float], list[float]]]:
        slot = self._arm_cmd.get(arm_id)
        return slot.get() if slot is not None else None

    def latest_gripper_command(self, gripper_id: int) -> Optional[float]:
        slot = self._gripper_cmd.get(gripper_id)
        return slot.get() if slot is not None else None

    def publish_arm_states(self, arm_id: int, positions: list[float], velocities: list[float]) -> None:
        if self._arm_pub is not None:
            self._schedule_publish(self._arm_pub, arm_states.build_message(arm_id, positions, velocities))

    def publish_gripper_states(self, gripper_id: int, opening: float, force: float = 0.0) -> None:
        if self._gripper_pub is not None:
            self._schedule_publish(self._gripper_pub, gripper_states.build_message(gripper_id, opening, force))

    def _schedule_publish(self, publisher: peppylib.TopicPublisher, payload: bytes) -> None:
        # Hand the publish to the node loop and return immediately; the physics
        # thread must never block on messaging.
        def _publish() -> None:
            task = asyncio.ensure_future(publisher.publish(payload))
            task.add_done_callback(_log_publish_error)

        try:
            self._loop.call_soon_threadsafe(_publish)
        except RuntimeError:
            # Loop closed during shutdown; drop the sample.
            pass


def _log_publish_error(task: asyncio.Task) -> None:
    if task.cancelled():
        return
    exc = task.exception()
    if exc is not None:
        logger.warning(f"state publish failed: {exc}")
