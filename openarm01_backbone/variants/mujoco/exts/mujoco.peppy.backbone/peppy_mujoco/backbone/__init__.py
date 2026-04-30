import sys
from pathlib import Path

# vendor/ — bundled peppylib wheel
_VENDOR = Path(__file__).resolve().parents[2] / "vendor"
if _VENDOR.is_dir() and str(_VENDOR) not in sys.path:
    sys.path.insert(0, str(_VENDOR))

# nodes_shared_code — exposes sim_ext_core package (dev fallback; containers set PYTHONPATH)
_INIT_PARENTS = Path(__file__).resolve().parents
_NODES_SHARED_CODE = _INIT_PARENTS[8] / "nodes_shared_code" if len(_INIT_PARENTS) > 8 else None
if _NODES_SHARED_CODE is not None and (_NODES_SHARED_CODE / "sim_ext_core").is_dir() and str(_NODES_SHARED_CODE) not in sys.path:
    sys.path.insert(0, str(_NODES_SHARED_CODE))

from .extension import MujocoBackboneExtension  # pylint: disable=C0413

__all__ = ["MujocoBackboneExtension"]
