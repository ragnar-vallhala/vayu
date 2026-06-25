"""Phase 1 integration: drive a flight through the PUBLIC package API.

The golden (test_golden_flight) drives the legacy sitl_lab.py shim to prove the
carve-out is regression-free; this test proves the SDK stands on its own —
`from vayu_headless import SitlSession, Pilot` boots, flies, and lands. Concise
(takeoff + hold + land) so it doesn't double the full-course runtime.
"""
import time

import pytest

# Armed and flying: ARMED on the ground, or IN_AIR once the baro-driven takeoff
# detector fires (Phase 2). Both mean "armed" — see test_golden_flight.py.
NAV_FLYING = (4, 5)  # ARMED, IN_AIR


@pytest.mark.integration
def test_public_api_takeoff_hold_land(require_binaries, gcs_conf):
    from vayu_headless import SitlSession, Pilot

    with SitlSession(gcs=False, conf=gcs_conf) as sess:
        pilot = Pilot(sess, alt=-5.0)
        pilot.arm_takeoff(alt=-5.0)
        time.sleep(4.0)

        tr = sess.truth()
        assert tr is not None
        # Wide band: the SITL outer loop is known-wobbly and settles anywhere
        # ~5-7 m for a 5 m target (host-scheduling-sensitive) — same band as
        # test_vertical_sitl.py.
        assert 3.5 < -tr["pos"][2] < 8.0, "altitude not held near target"

        hb = sess.telem.get("Heartbeat")
        assert hb is not None and hb.nav_state in NAV_FLYING
        assert sess.telem_counts.get("ImuCompressed", 0) > 0

        pilot.land()
        time.sleep(0.5)
        assert pilot.armed is False
