from __future__ import annotations

import sys
from pathlib import Path

# Inject shared/sim_ext_core (extracted from peppy-bridge interfaces/ by the backbone PR).
_root = Path(__file__).parents[8]
_sim_ext_core = _root / "shared" / "sim_ext_core"
if str(_sim_ext_core) not in sys.path:
    sys.path.insert(0, str(_sim_ext_core))
