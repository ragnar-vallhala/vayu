"""Phase 4 integration: run the examples/box_mission end-to-end via the SDK API."""
import os
import sys

import pytest

# Armed and flying: ARMED on the ground, or IN_AIR once the baro-driven takeoff
# detector fires (Phase 2). Both mean "armed" — see test_golden_flight.py.
NAV_FLYING = (4, 5)  # ARMED, IN_AIR
_EXAMPLES = os.path.join(os.path.dirname(__file__), "..", "..", "examples")


@pytest.mark.integration
def test_box_mission(require_binaries, gcs_conf):
    sys.path.insert(0, os.path.abspath(_EXAMPLES))
    import box_mission

    from vayu_headless import SitlSession

    with SitlSession(gcs=False, conf=gcs_conf) as sess:
        result = box_mission.fly_box(sess, side=8.0, alt=-5.0, timeout=45.0,
                                     land=False)
        # advanced through the course (tuning-independent) and held altitude.
        # Wide band: the SITL outer loop is known-wobbly and settles anywhere
        # ~5-7 m for a 5 m target (host-scheduling-sensitive) — same band as
        # test_vertical_sitl.py.
        assert result.get("wp", 0) >= 2, f"only reached wp {result.get('wp')}"
        assert 3.5 < -result.get("z", 0.0) < 8.0
        hb = sess.telem.get("Heartbeat")
        assert hb is not None and hb.nav_state in NAV_FLYING
