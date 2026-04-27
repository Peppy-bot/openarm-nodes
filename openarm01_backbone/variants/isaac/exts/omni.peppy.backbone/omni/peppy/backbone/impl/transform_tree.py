from __future__ import annotations

import logging
from typing import Optional

logger = logging.getLogger(__name__)


class IsaacTransformTree:
    """Reads world-frame transforms for every body in an Isaac Sim articulation."""

    def __init__(self, prim_path: str) -> None:
        self._prim_path = prim_path
        self._articulation = None
        self._body_prim_paths: dict[str, str] = {}
        self._body_parents: dict[str, str] = {}
        self._ready: bool = False

    def setup(self) -> bool:
        """Initialise articulation and build body name → full prim path map via stage traversal."""
        if self._articulation is not None and self._ready:
            return True
        try:
            import omni.usd  # pylint: disable=E0401
            from isaacsim.core.prims import Articulation  # pylint: disable=E0401

            self._articulation = Articulation(prim_paths_expr=self._prim_path)
            self._articulation.initialize()

            stage = omni.usd.get_context().get_stage()
            body_names = set(self._articulation.body_names)

            # Traverse the full stage to find each body prim regardless of nesting depth.
            self._body_prim_paths = {}
            self._body_parents = {}
            for prim in stage.Traverse():
                name = prim.GetName()
                if name in body_names and name not in self._body_prim_paths:
                    self._body_prim_paths[name] = str(prim.GetPath())
                    parent = prim.GetParent()
                    self._body_parents[name] = parent.GetName() if parent and parent.IsValid() else "world"

            missing = body_names - set(self._body_prim_paths)
            if missing:
                logger.warning(
                    f"IsaacTransformTree: could not find prims for bodies: {sorted(missing)}"
                    " — they will be skipped in tf_tree output."
                )

            self._ready = True
        except Exception as exc:
            logger.error(
                f"Failed to setup IsaacTransformTree at '{self._prim_path}': {exc}"
            )
            self._articulation = None
            self._ready = False
            return False

        logger.info(
            f"IsaacTransformTree ready — prim='{self._prim_path}'"
            f" bodies={len(self._body_prim_paths)}"
        )
        return True

    def teardown(self) -> None:
        self._articulation = None
        self._body_prim_paths = {}
        self._body_parents = {}
        self._ready = False

    def _body_frame(self, stage, time_code, name: str, parent_name: str) -> dict:
        from pxr import UsdGeom  # pylint: disable=E0401

        prim_path = self._body_prim_paths[name]
        prim = stage.GetPrimAtPath(prim_path)
        matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(time_code)
        translation = matrix.ExtractTranslation()
        rotation = matrix.ExtractRotationQuat()
        img = rotation.GetImaginary()
        return {
            "name": name,
            "parent": parent_name,
            "position": [translation[0], translation[1], translation[2]],
            "orientation": [rotation.GetReal(), img[0], img[1], img[2]],
        }

    def get_tf_data(self) -> Optional[list[dict]]:
        """Return world-frame position and orientation for every body."""
        if not self._ready or self._articulation is None:
            return None

        try:
            import omni.usd  # pylint: disable=E0401
            from pxr import Usd  # pylint: disable=E0401

            stage = omni.usd.get_context().get_stage()
            time_code = Usd.TimeCode.Default()
            frames = []
            for name in self._articulation.body_names:
                if name not in self._body_prim_paths:
                    continue
                frames.append(
                    self._body_frame(stage, time_code, name, self._body_parents[name])
                )
            return frames
        except Exception as exc:
            logger.warning(f"Could not read transform tree data: {exc}")
            return None

    @property
    def is_ready(self) -> bool:
        return self._ready
