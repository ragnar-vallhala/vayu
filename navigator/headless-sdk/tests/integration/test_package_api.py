"""Phase 1 integration: drive a flight through the PUBLIC package API.

The golden (test_golden_flight) drives the legacy sitl_lab.py shim to prove the
carve-out is regression-free; this test proves the SDK stands on its own —
`from vayu_headless import SitlSession, Pilot` boots, flies, and lands. Concise
(takeoff + hold + land) so it doesn't double the full-course runtime.
"""
import time

import pytest

NAV_ARMED = 4


@pytest.mark.integration
def test_public_api_takeoff_hold_land(require_binaries, gcs_conf):
    from vayu_headless import SitlSession, Pilot

    with SitlSession(gcs=False, conf=gcs_conf) as sess:
        pilot = Pilot(sess, alt=-5.0)
        pilot.arm_takeoff(alt=-5.0)
        time.sleep(4.0)

        tr = sess.truth()
        assert tr is not None
        assert 4.0 < -tr["pos"][2] < 6.5, "altitude not held near 5 m"

        hb = sess.telem.get("Heartbeat")
        assert hb is not None and hb.nav_state == NAV_ARMED
        assert sess.telem_counts.get("ImuCompressed", 0) > 0

        pilot.land()
        time.sleep(0.5)
        assert pilot.armed is False
