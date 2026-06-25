"""Phase 2 SITL verify (altitude-hold plan §6 step 2): baro-driven IN_AIR.

Before the barometer the FC had no vertical observability, so nothing ever called
system_state_set(SYSTEM_STATE_IN_AIR) — IN_AIR was a dead state. The VERT
estimator + flight_phase detector close that gap: takeoff (ARMED→IN_AIR) when
AGL, climb rate, and throttle all agree past a debounce; touchdown the reverse.

This drives the real firmware in the loop and checks the two robustly-observable
properties end-to-end:
  - armed-on-the-ground does NOT false-trip IN_AIR (idle throttle, no climb);
  - a real takeoff DOES transition ARMED→IN_AIR, and the FC-authoritative AGL
    (VERTICAL_STATE.agl, decision D4) reads airborne.

The touchdown edge (IN_AIR→ARMED) is exercised by the host unit test
(tools/sim_host/tests/test_flight_phase.c, FP-005) — through the Pilot it is
masked by the RC disarm-on-land (IN_AIR→STANDBY), so it isn't asserted here.

No GCS bridges (gcs=False) so it never touches the shared singletons.
"""
import time

import pytest

# NavLink nav_state codes: ARMED=4, IN_AIR=5 (navlink/dialect.json nav_state enum).
NAV_ARMED = 4
NAV_IN_AIR = 5


@pytest.mark.integration
def test_armed_on_ground_does_not_trip_in_air(require_binaries, gcs_conf):
    from vayu_headless import SitlSession
    lab = SitlSession(gcs=False, conf=gcs_conf)
    try:
        # Spawn level on the ground and let the estimator settle.
        lab.set_rc(swa=1000, thr=1000)          # disarm → STANDBY
        time.sleep(0.3)
        lab.reset_pose((0, 0, -0.05))
        time.sleep(0.8)
        lab.set_rc(swa=2000, thr=1000)          # arm gesture at idle throttle

        # Reaches ARMED...
        t0 = time.time()
        armed = False
        while time.time() - t0 < 5.0:
            hb = lab.telem.get("Heartbeat")
            if hb is not None and hb.nav_state == NAV_ARMED:
                armed = True
                break
            time.sleep(0.1)
        assert armed, "never reached ARMED on the ground"

        # ...and STAYS armed: idle throttle + no climb must never trip takeoff.
        t0 = time.time()
        while time.time() - t0 < 2.5:
            hb = lab.telem.get("Heartbeat")
            assert hb is None or hb.nav_state != NAV_IN_AIR, \
                "false IN_AIR while armed idle on the ground"
            time.sleep(0.1)

        # FC-authoritative AGL should read ~0 on the ground.
        vs = lab.telem.get("VerticalState")
        if vs is not None and vs.valid:
            assert abs(vs.agl) < 0.5, f"AGL not ~0 on the ground (got {vs.agl:.2f})"
    finally:
        lab.close()


@pytest.mark.integration
def test_takeoff_transitions_to_in_air(require_binaries, gcs_conf):
    from vayu_headless import Pilot, SitlSession
    lab = SitlSession(gcs=False, conf=gcs_conf)
    try:
        pilot = Pilot(lab, alt=-5.0)
        pilot.arm_takeoff(alt=-5.0)

        # Climbing to ~5 m: the detector should fire ARMED→IN_AIR.
        t0 = time.time()
        in_air = False
        while time.time() - t0 < 8.0:
            hb = lab.telem.get("Heartbeat")
            if hb is not None and hb.nav_state == NAV_IN_AIR:
                in_air = True
                break
            time.sleep(0.1)
        assert in_air, "FC never transitioned ARMED→IN_AIR during takeoff"

        # The FC-authoritative AGL should read clearly airborne.
        vs = lab.telem.get("VerticalState")
        assert vs is not None and vs.valid, "no valid VerticalState after takeoff"
        assert vs.agl > 1.0, f"AGL not airborne (got {vs.agl:.2f})"

        pilot.land()
        time.sleep(0.5)
    finally:
        lab.close()
