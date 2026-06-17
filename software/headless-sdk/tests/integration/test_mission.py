"""Phase 4 integration: run the examples/box_mission end-to-end via the SDK API."""
import os
import sys

import pytest

NAV_ARMED = 4
_EXAMPLES = os.path.join(os.path.dirname(__file__), "..", "..", "examples")


@pytest.mark.integration
def test_box_mission(require_binaries, gcs_conf):
    sys.path.insert(0, os.path.abspath(_EXAMPLES))
    import box_mission

    from vayu_headless import SitlSession

    with SitlSession(gcs=False, conf=gcs_conf) as sess:
        result = box_mission.fly_box(sess, side=8.0, alt=-5.0, timeout=45.0,
                                     land=False)
        # advanced through the course (tuning-independent) and held altitude
        assert result.get("wp", 0) >= 2, f"only reached wp {result.get('wp')}"
        assert 3.0 < -result.get("z", 0.0) < 7.0
        hb = sess.telem.get("Heartbeat")
        assert hb is not None and hb.nav_state == NAV_ARMED
