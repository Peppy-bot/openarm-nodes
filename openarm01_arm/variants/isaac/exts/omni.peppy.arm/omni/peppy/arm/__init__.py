from __future__ import annotations

import sys
from pathlib import Path

_VENDOR = Path(__file__).resolve().parents[3] / "vendor"
if _VENDOR.is_dir() and str(_VENDOR) not in sys.path:
    sys.path.insert(0, str(_VENDOR))

_NODES_SHARED_CODE = Path(__file__).resolve().parents[9] / "nodes_shared_code"
if (_NODES_SHARED_CODE / "sim_ext_core").is_dir() and str(_NODES_SHARED_CODE) not in sys.path:
    sys.path.insert(0, str(_NODES_SHARED_CODE))

from .extension import ArmExtension  # pylint: disable=C0413

__all__ = ["ArmExtension"]
