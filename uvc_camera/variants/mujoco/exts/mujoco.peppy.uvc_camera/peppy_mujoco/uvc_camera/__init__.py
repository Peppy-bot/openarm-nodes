import sys
from pathlib import Path

# vendor/ — bundled peppylib wheel
# parents[2] = mujoco.peppy.uvc_camera/ (ext root)
_VENDOR = Path(__file__).resolve().parents[2] / "vendor"
if _VENDOR.is_dir() and str(_VENDOR) not in sys.path:
    sys.path.insert(0, str(_VENDOR))

# nodes_shared_code — exposes sim_ext_core package (dev fallback; containers set PYTHONPATH)
# parents[8] = src/ (nodes_shared_code lives here as sibling to openarm01_nodes/)
_NODES_SHARED_CODE = Path(__file__).resolve().parents[8] / "nodes_shared_code"
if (_NODES_SHARED_CODE / "sim_ext_core").is_dir() and str(_NODES_SHARED_CODE) not in sys.path:
    sys.path.insert(0, str(_NODES_SHARED_CODE))

from .extension import MujocoUvcCameraExtension  # pylint: disable=C0413

__all__ = ["MujocoUvcCameraExtension"]
