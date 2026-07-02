"""Typed peppygen topic IO for the openarm sim.

Bridges the physics thread (sync, runs mj_step in an executor) to the
node_runner's asyncio loop, where peppygen topic pub/sub lives. Consume tasks on
the loop keep the latest actuator command per side in thread-safe slots; the
physics thread reads those and publishes measured state back through the loop.

Publishers and payload (de)serialization go through generated peppygen topics.
The command consumers hold one persistent subscriber each: the generated Python
consumed-topic API only exposes single-shot on_next_message_received, which
re-subscribes on every call and so both drops any command published in the gap
between calls and re-trips the messenger's one-per-(name, tag) from_any
reservation in a loop. The persistent subscribe below mirrors the Rust generated
`subscribe`; it reuses the generated subscribe arguments and _deserialize_payload.
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
from peppygen.consumed_topics import arm_cmd_arm_sim_passthrough as arm_cmd
from peppygen.consumed_topics import gripper_cmd_gripper_sim_passthrough as gripper_cmd

logger = logging.getLogger(__name__)

# Left = 0, right = 1, matching arm_id / gripper_id across the stack.
_SIDES = (0, 1)


async def _subscribe_persistent(node_runner, interface: str, topic_name: str, producer_slot: str):
    """Declare one persistent from_any subscriber, mirroring the generated
    single-shot on_next_message_received's subscribe arguments. Loop on the
    returned subscription's on_next_message() to receive every message with no
    re-subscribe gap. Arguments are duplicated from the generated consumed-topic
    module because the Python API does not (yet) expose a persistent subscribe."""
    return await peppylib.TopicMessenger.subscribe(
        node_runner.messenger(),
        node_runner.bound_core_node(),
        node_runner.bound_instance_id(),
        peppylib.SenderTarget.interface(interface, "v1"),
        topic_name,
        node_runner.pinned_producer_for(producer_slot),
        peppylib.QoSProfile.Standard,
        is_from_any=True,
    )


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
            asyncio.create_task(self._consume_arm()),
            asyncio.create_task(self._consume_gripper()),
        ]

    async def stop(self) -> None:
        for task in self._tasks:
            task.cancel()
        # Let the cancellations land so the consume loops exit before teardown.
        await asyncio.gather(*self._tasks, return_exceptions=True)

    async def _consume_arm(self) -> None:
        subscription = await _subscribe_persistent(
            self._node_runner, "openarm_arm_sim_passthrough", "arm_sim_passthrough", "arm_cmd"
        )
        while True:
            try:
                raw = await subscription.on_next_message()
                if raw is None:
                    return
                # Deserialize inside the guard: a corrupt frame is dropped and
                # logged rather than killing this consume task.
                msg = arm_cmd._deserialize_payload(raw.payload)
            except asyncio.CancelledError:
                return
            except Exception as exc:
                logger.warning(f"arm command consume error: {exc}")
                continue
            # Drop a poisoned command rather than writing NaN/Inf into the sim.
            if not all(math.isfinite(v) for v in msg.positions) or not all(
                math.isfinite(v) for v in msg.velocities
            ):
                logger.warning(f"dropping non-finite arm command for arm_id={msg.arm_id}")
                continue
            slot = self._arm_cmd.get(msg.arm_id)
            if slot is not None:
                slot.set((msg.positions, msg.velocities))

    async def _consume_gripper(self) -> None:
        subscription = await _subscribe_persistent(
            self._node_runner,
            "openarm_gripper_sim_passthrough",
            "gripper_sim_passthrough",
            "gripper_cmd",
        )
        while True:
            try:
                raw = await subscription.on_next_message()
                if raw is None:
                    return
                # Deserialize inside the guard: a corrupt frame is dropped and
                # logged rather than killing this consume task.
                msg = gripper_cmd._deserialize_payload(raw.payload)
            except asyncio.CancelledError:
                return
            except Exception as exc:
                logger.warning(f"gripper command consume error: {exc}")
                continue
            if not math.isfinite(msg.position):
                logger.warning(
                    f"dropping non-finite gripper command for gripper_id={msg.gripper_id}"
                )
                continue
            slot = self._gripper_cmd.get(msg.gripper_id)
            if slot is not None:
                slot.set(msg.position)

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

    def publish_gripper_states(self, gripper_id: int, position: float, force: float = 0.0) -> None:
        if self._gripper_pub is not None:
            self._schedule_publish(self._gripper_pub, gripper_states.build_message(gripper_id, position, force))

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
