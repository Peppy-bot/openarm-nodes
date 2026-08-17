#!/usr/bin/env python3
"""OpenArm Isaac scene commander."""

from __future__ import annotations

import asyncio
import json
import logging

import peppylib
from peppygen import NodeBuilder, NodeRunner
from peppygen.consumed_actions.simulation import (
    spawn_object,
)
from peppygen.consumed_services.simulation import (
    get_assets_list,
    get_objects_list,
)

logger = logging.getLogger(__name__)

SERVICE_TIMEOUT_S = 10.0


async def _fetch_assets(
    node_runner: NodeRunner,
) -> list:
    producer = get_assets_list.bound_producer(
        node_runner
    )

    response = await get_assets_list.poll(
        node_runner,
        producer,
        timeout=SERVICE_TIMEOUT_S,
    )

    data = response.data

    if not data.success:
        raise RuntimeError(
            data.message
        )

    return json.loads(
        data.assets_json
    )


async def _fetch_objects(
    node_runner: NodeRunner,
) -> list:
    producer = get_objects_list.bound_producer(
        node_runner
    )

    response = await get_objects_list.poll(
        node_runner,
        producer,
        timeout=SERVICE_TIMEOUT_S,
    )

    data = response.data

    if not data.success:
        raise RuntimeError(
            data.message
        )

    return json.loads(
        data.objects_json
    )


async def _spawn_test_object(
    node_runner: NodeRunner,
) -> str:
    """Spawn one red block through the Peppy action API."""

    producer = spawn_object.bound_producer(
        node_runner
    )

    request = spawn_object.GoalRequest(
        asset_id="props/blocks/red_block",
        position=[0.5, 0.0, 0.8],
        scale=1.0,
        physics="none",
        mass=0.1,
    )

    logger.info(
        "Sending spawn_object goal for %s",
        request.asset_id,
    )

    handle = await spawn_object.ActionHandle.fire_goal(
        node_runner,
        producer,
        request,
        timeout=SERVICE_TIMEOUT_S,
        feedback_qos=peppylib.QoSProfile.Standard,
    )

    if not handle.accepted:
        raise RuntimeError(
            "spawn_object goal rejected: "
            + str(handle.reason)
        )

    logger.info(
        "spawn_object goal accepted"
    )

    result = await handle.get_result(
        timeout=SERVICE_TIMEOUT_S
    )

    if result.status != spawn_object.ResultStatus.COMPLETED:
        raise RuntimeError(
            "spawn_object did not complete successfully; "
            f"status={result.status.name}"
        )

    if result.data is None:
        raise RuntimeError(
            "spawn_object completed without result data"
        )

    if not result.data.success:
        raise RuntimeError(
            "spawn_object failed: "
            + result.data.message
        )

    logger.info(
        "Spawned runtime object: %s | %s",
        result.data.object_id,
        result.data.message,
    )

    return result.data.object_id


async def setup(
    _params,
    node_runner: NodeRunner,
) -> list[asyncio.Task]:
    logger.info(
        "OpenArm scene commander starting"
    )

    assets = await _fetch_assets(
        node_runner
    )

    logger.info(
        "Fetched %d Isaac assets",
        len(assets),
    )

    for asset in assets[:10]:
        logger.info(
            "Asset: %s | %s | %s",
            asset.get("asset_id"),
            asset.get("display_name"),
            asset.get("category"),
        )

    objects_before = await _fetch_objects(
        node_runner
    )

    logger.info(
        "Fetched %d runtime objects before spawn",
        len(objects_before),
    )

    object_id = await _spawn_test_object(
        node_runner
    )

    objects = await _fetch_objects(
        node_runner
    )

    logger.info(
        "Fetched %d runtime objects after spawn",
        len(objects),
    )

    for obj in objects:
        logger.info(
            "Runtime object: %s | asset=%s | position=%s",
            obj.get("object_id"),
            obj.get("asset_id"),
            obj.get("position"),
        )

    if not any(
        obj.get("object_id") == object_id
        for obj in objects
    ):
        raise RuntimeError(
            "Spawned object was not returned by get_objects_list"
        )

    logger.info(
        "Peppy spawn-object round trip verified for %s",
        object_id,
    )

    return []


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(levelname)s - %(message)s",
    )

    NodeBuilder().run(
        setup
    )


if __name__ == "__main__":
    main()
