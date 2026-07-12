"""vayu_headless — SDK for driving the REAL Vayu flight-controller logic headlessly.

Drives the actual firmware (estimator → angle/rate cascade → mixer → arming →
telemetry) against the in-process physics engine, with no hardware and no human on
the sticks. See PLAN.md for the standardisation roadmap.

Phase 0: package skeleton only. The public API surface (SitlSession, Pilot, …)
is carved out of sim/host/sitl_lab.py in Phase 1 and re-exported here.
"""

__version__ = "0.0.1"

from .session import SitlSession, SitlLab    # noqa: E402
from .autopilot import Pilot, DEFAULT_GAINS  # noqa: E402

__all__ = [
    "__version__",
    "SitlSession", "SitlLab",   # SitlLab is a back-compat alias of SitlSession
    "Pilot", "DEFAULT_GAINS",
]
