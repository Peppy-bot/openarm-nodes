"""Typed peppygen pairing IO for the openarm sim.

Bridges the physics thread (sync, runs the engine step in an executor) to the
node_runner's asyncio loop, where peppygen pairing pub/sub lives. The engine
plays the follower role of every limb's joint_link / gripper_link pairing, one
slot per limb, so there is no id demux: consume tasks on the loop each hold one
generated `subscribe()` subscription (gap-free, in-order) and keep the latest
governed setpoint per limb in thread-safe slots; the physics thread reads those
and publishes stamped measured state back up the same pair. Every hop is a
generated peppygen pairing topic: no JSON, no raw peppylib.
"""

from __future__ import annotations

import asyncio
import logging
import math
import threading
from typing import Optional

import peppylib
from peppygen import clock
from peppygen.paired_topics.left_arm import joint_setpoints as left_arm_setpoints
from peppygen.paired_topics.left_arm import joint_states as left_arm_states
from peppygen.paired_topics.left_gripper import gripper_setpoints as left_gripper_setpoints
from peppygen.paired_topics.left_gripper import gripper_states as left_gripper_states
from peppygen.paired_topics.right_arm import joint_setpoints as right_arm_setpoints
from peppygen.paired_topics.right_arm import joint_states as right_arm_states
from peppygen.paired_topics.right_gripper import gripper_setpoints as right_gripper_setpoints
from peppygen.paired_topics.right_gripper import gripper_states as right_gripper_states

logger = logging.getLogger(__name__)

# Left = 0, right = 1: the slot layout mirrors the arm_id / gripper_id
# convention the rest of the stack uses for sides.
_ARM_SLOTS = {0: (left_arm_setpoints, left_arm_states), 1: (right_arm_setpoints, right_arm_states)}
_GRIPPER_SLOTS = {
    0: (left_gripper_setpoints, left_gripper_states),
    1: (right_gripper_setpoints, right_gripper_states),
}


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
    """Owns the typed pairing publishers + setpoint-consume tasks on the node
    loop, and exposes thread-safe accessors the physics thread calls each step."""

    def __init__(self, node_runner: peppylib.NodeRunner, loop: asyncio.AbstractEventLoop) -> None:
        self._node_runner = node_runner
        self._loop = loop
        self._arm_pubs: dict[int, peppylib.TopicPublisher] = {}
        self._gripper_pubs: dict[int, peppylib.TopicPublisher] = {}
        self._arm_cmd = {side: _LatestSlot() for side in _ARM_SLOTS}
        self._gripper_cmd = {side: _LatestSlot() for side in _GRIPPER_SLOTS}
        self._tasks: list[asyncio.Task] = []

    async def start(self) -> None:
        """Declare publishers and spawn the setpoint-consume loops. Runs on the
        node loop before the sim thread starts. Publishing while a slot is
        unpaired is a legal no-op, so bringup order never matters."""
        # State stamps read the daemon-resolved clock, the same source every
        # follower uses, so consumers age samples on one timeline.
        await clock.init(self._node_runner)
        for side, (_, states) in _ARM_SLOTS.items():
            self._arm_pubs[side] = await states.declare_publisher(self._node_runner)
        for side, (_, states) in _GRIPPER_SLOTS.items():
            self._gripper_pubs[side] = await states.declare_publisher(self._node_runner)
        self._tasks = [
            asyncio.create_task(self._consume_arm(mod, side))
            for side, (mod, _) in _ARM_SLOTS.items()
        ] + [
            asyncio.create_task(self._consume_gripper(mod, side))
            for side, (mod, _) in _GRIPPER_SLOTS.items()
        ]

    async def stop(self) -> None:
        for task in self._tasks:
            task.cancel()
        # Let the cancellations land so the consume loops exit before teardown.
        await asyncio.gather(*self._tasks, return_exceptions=True)

    async def _consume_arm(self, topic, side: int) -> None:
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
                logger.warning(f"{topic.LINK_ID} setpoint consume error: {exc}")
                await asyncio.sleep(0.1)
                continue
            _producer, msg = pair
            # Drop a poisoned setpoint rather than writing NaN/Inf into the sim.
            if not all(math.isfinite(v) for v in msg.positions) or not all(
                math.isfinite(v) for v in msg.velocities
            ):
                logger.warning(f"dropping non-finite arm setpoint on {topic.LINK_ID}")
                continue
            if self._arm_cmd[side].get() is None:
                logger.info(f"first arm setpoint on {topic.LINK_ID}")
            self._arm_cmd[side].set((msg.positions, msg.velocities))

    async def _consume_gripper(self, topic, side: int) -> None:
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
                logger.warning(f"{topic.LINK_ID} setpoint consume error: {exc}")
                await asyncio.sleep(0.1)
                continue
            _producer, msg = pair
            if not math.isfinite(msg.opening):
                logger.warning(f"dropping non-finite gripper setpoint on {topic.LINK_ID}")
                continue
            # max_effort is ignored: the sim engine applies no grip-force cap
            # and reports a 0 ceiling on its states.
            if self._gripper_cmd[side].get() is None:
                logger.info(f"first gripper setpoint on {topic.LINK_ID}")
            self._gripper_cmd[side].set(msg.opening)

    # --- called from the physics thread ---

    def latest_arm_command(self, arm_id: int) -> Optional[tuple[list[float], list[float]]]:
        slot = self._arm_cmd.get(arm_id)
        return slot.get() if slot is not None else None

    def latest_gripper_command(self, gripper_id: int) -> Optional[float]:
        slot = self._gripper_cmd.get(gripper_id)
        return slot.get() if slot is not None else None

    def publish_arm_states(self, arm_id: int, positions: list[float], velocities: list[float]) -> None:
        pub = self._arm_pubs.get(arm_id)
        if pub is not None:
            # Efforts are empty: the engine measures no joint torques.
            payload = _ARM_SLOTS[arm_id][1].build_message(
                clock.now_ns() / 1e9, positions, velocities, []
            )
            self._schedule_publish(pub, payload)

    def publish_gripper_states(self, gripper_id: int, opening: float, force: float = 0.0) -> None:
        pub = self._gripper_pubs.get(gripper_id)
        if pub is not None:
            # The engine torque rides as the pairing effort; the ceiling is 0
            # (no effort control).
            payload = _GRIPPER_SLOTS[gripper_id][1].build_message(
                clock.now_ns() / 1e9, opening, force, 0.0
            )
            self._schedule_publish(pub, payload)

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
