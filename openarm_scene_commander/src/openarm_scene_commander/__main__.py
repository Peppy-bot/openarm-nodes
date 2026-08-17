#!/usr/bin/env python3
"""OpenArm Isaac scene commander."""

from __future__ import annotations

import asyncio
import json
import logging

from peppygen import NodeBuilder, NodeRunner
from peppygen.consumed_services.simulation import (
    get_assets_list,
    get_objects_list,
)

logger = logging.getLogger(__name__)

SERVICE_TIMEOUT_S = 10.0


async def _fetch_assets(
    node_runner: NodeRunner,
) -> list:
    producers = get_assets_list.bound_producers(
        node_runner
    )

    if not producers:
        raise RuntimeError(
            "No openarm_sim_isaac producer is bound"
        )

    response = await get_assets_list.poll(
        node_runner,
        producers[0],
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
    producers = get_objects_list.bound_producers(
        node_runner
    )

    if not producers:
        raise RuntimeError(
            "No openarm_sim_isaac producer is bound"
        )

    response = await get_objects_list.poll(
        node_runner,
        producers[0],
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

    objects = await _fetch_objects(
        node_runner
    )

    logger.info(
        "Fetched %d runtime objects",
        len(objects),
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
