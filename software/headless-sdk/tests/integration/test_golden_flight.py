"""Phase 0 GOLDEN: pin today's boot → takeoff → hover → box → land behaviour.

This drives the CURRENT tools/sim_host/sitl_lab.py code path (via the
legacy_sitl_lab fixture). Phase 1 carves that logic into the package and this
same test must stay green — it is the no-regression anchor for the hard-cut
(PLAN.md decision #3, Phase 6).

Runs the real firmware-in-the-loop, so it is marked `integration` and skips if
the binaries aren't built. No GCS bridges (gcs=False) so it never touches the
shared /tmp/vsim_pose, UART2 advert, or control socket singletons.
"""
import time

import pytest

# NavLink nav_state codes (see memory/sitl-test-harness.md): ARMED=4, STANDBY=2.
NAV_ARMED = 4


@pytest.mark.integration
def test_boot_takeoff_hover_box_land(require_binaries, gcs_conf, legacy_sitl_lab):
    S = legacy_sitl_lab
    lab = S.SitlLab(gcs=False, conf=gcs_conf)
    try:
        pilot = S.Pilot(lab, alt=-5.0)

        # --- takeoff + hold ---
        pilot.arm_takeoff(alt=-5.0)
        time.sleep(4.0)
        tr = lab.truth()
        assert tr is not None, "no ground-truth pose after takeoff"
        alt = -tr["pos"][2]
        assert 4.0 < alt < 6.5, f"altitude not held near 5 m (got {alt:.2f})"

        hb = lab.telem.get("Heartbeat")
        assert hb is not None, "no Heartbeat telemetry decoded"
        assert hb.nav_state == NAV_ARMED, f"expected ARMED, nav={hb.nav_state}"

        # telemetry is actually streaming (not a one-shot burst)
        assert lab.telem_counts.get("AttitudeEuler", 0) > 0
        assert lab.telem_counts.get("ImuCompressed", 0) > 0

        # --- fly a box: assert tuning-INDEPENDENT properties only ---
        # The outer guidance is the known-wobbly loop (precise settling is a
        # separate tuning task, out of scope), so the golden asserts what must
        # hold regardless of tuning: it stays ARMED, holds altitude, and
        # ADVANCES THROUGH the course (waypoint-advance + translation work).
        pilot.goto([(8, 0), (8, 8), (0, 8), (0, 0)], alt=-5.0)
        max_wp = 0
        t0 = time.time()
        while time.time() - t0 < 45.0:
            st = pilot.last
            if st:
                max_wp = max(max_wp, st.get("wp", 0))
                alt = -st.get("z", 0.0)
                assert 3.0 < alt < 7.0, f"altitude lost during course (got {alt:.2f})"
            hb = lab.telem.get("Heartbeat")
            assert hb is None or hb.nav_state == NAV_ARMED, \
                f"left ARMED during course (nav={getattr(hb,'nav_state',None)})"
            if pilot.reached_last():
                break
            time.sleep(0.2)
        assert max_wp >= 2, f"course not followed: only advanced to wp {max_wp}/3"

        # --- land + disarm ---
        pilot.land()
        time.sleep(0.5)
        assert pilot.armed is False
    finally:
        lab.close()
