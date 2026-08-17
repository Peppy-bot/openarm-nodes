#!/usr/bin/env python3
"""Peppy scene services/actions bridge for Isaac Sim."""

from __future__ import annotations

import asyncio
import json
import logging
import threading

from peppygen.exposed_services import (
    get_assets_list,
    get_objects_list,
)

logger = logging.getLogger(__name__)


class SceneActionIO:
    """Bridge Peppy scene services to Isaac-owned runtime state."""

    def __init__(
        self,
        node_runner,
        loop: asyncio.AbstractEventLoop,
    ) -> None:
        self._node_runner = node_runner
        self._loop = loop

        self._lock = threading.Lock()
        self._assets = {}
        self._objects = {}

        self._tasks = []

    async def start(self) -> None:
        """Start exposed Peppy scene service loops."""

        self._tasks = [
            asyncio.create_task(
                self._serve_assets()
            ),
            asyncio.create_task(
                self._serve_objects()
            ),
        ]

        logger.info(
            "SceneActionIO services started"
        )

    async def stop(self) -> None:
        """Stop exposed Peppy scene service loops."""

        for task in self._tasks:
            task.cancel()

        if self._tasks:
            await asyncio.gather(
                *self._tasks,
                return_exceptions=True,
            )

        self._tasks = []

        logger.info(
            "SceneActionIO services stopped"
        )

    def set_assets(self, assets: dict) -> None:
        """Replace the private Isaac asset catalogue."""

        with self._lock:
            self._assets = {
                asset_id: dict(asset)
                for asset_id, asset in assets.items()
            }

        logger.info(
            "SceneActionIO received %d Isaac assets",
            len(assets),
        )

    def set_objects(self, objects: dict) -> None:
        """Replace the current runtime object registry."""

        with self._lock:
            self._objects = {
                object_id: dict(obj)
                for object_id, obj in objects.items()
            }

    def _public_assets(self) -> list:
        """Return asset metadata without exposing raw Isaac paths."""

        with self._lock:
            assets = list(
                self._assets.values()
            )

        public = []

        for asset in assets:
            public.append(
                {
                    "asset_id": asset.get(
                        "asset_id",
                        "",
                    ),
                    "display_name": asset.get(
                        "display_name",
                        "",
                    ),
                    "kind": asset.get(
                        "kind",
                        "",
                    ),
                    "category": asset.get(
                        "category",
                        "",
                    ),
                }
            )

        public.sort(
            key=lambda item: (
                item["category"].lower(),
                item["display_name"].lower(),
                item["asset_id"],
            )
        )

        return public

    def _public_objects(self) -> list:
        """Return currently known runtime object metadata."""

        with self._lock:
            objects = [
                dict(obj)
                for obj in self._objects.values()
            ]

        objects.sort(
            key=lambda item: item.get(
                "object_id",
                ""
            )
        )

        return objects

    def _handle_get_assets(
        self,
        _request,
    ) -> get_assets_list.Response:
        assets = self._public_assets()

        return get_assets_list.Response(
            success=True,
            message=(
                f"{len(assets)} assets available"
            ),
            assets_json=json.dumps(
                assets,
                separators=(",", ":"),
            ),
        )

    def _handle_get_objects(
        self,
        _request,
    ) -> get_objects_list.Response:
        objects = self._public_objects()

        return get_objects_list.Response(
            success=True,
            message=(
                f"{len(objects)} runtime objects"
            ),
            objects_json=json.dumps(
                objects,
                separators=(",", ":"),
            ),
        )

    async def _serve_assets(self) -> None:
        while True:
            try:
                await get_assets_list.handle_next_request(
                    self._node_runner,
                    self._handle_get_assets,
                )

            except asyncio.CancelledError:
                raise

            except Exception:
                logger.exception(
                    "get_assets_list service failed"
                )
                await asyncio.sleep(1.0)

    async def _serve_objects(self) -> None:
        while True:
            try:
                await get_objects_list.handle_next_request(
                    self._node_runner,
                    self._handle_get_objects,
                )

            except asyncio.CancelledError:
                raise

            except Exception:
                logger.exception(
                    "get_objects_list service failed"
                )
                await asyncio.sleep(1.0)
