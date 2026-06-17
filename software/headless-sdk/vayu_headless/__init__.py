"""vayu_headless — SDK for driving the REAL Vayu flight-controller logic headlessly.

Drives the actual firmware (estimator → angle/rate cascade → mixer → arming →
telemetry) against the vsim_d physics daemon, with no hardware and no human on
the sticks. See PLAN.md for the standardisation roadmap.

Phase 0: package skeleton only. The public API surface (SitlSession, Pilot, …)
is carved out of tools/sim_host/sitl_lab.py in Phase 1 and re-exported here.
"""

__version__ = "0.0.1"

__all__ = ["__version__"]
