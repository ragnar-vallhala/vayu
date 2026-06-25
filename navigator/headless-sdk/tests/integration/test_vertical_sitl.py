"""Phase 1 SITL verify (altitude-hold plan §6 step 1): the VERT vertical
estimator tracks ground truth in the simulator.

The 2-state filter core is unit-tested headlessly (tools/sim_host/tests/
test_vertical_est.c). This test closes the loop end-to-end: real firmware VERT
task fed the modelled baro, publishing VERTICAL_STATE telemetry, checked against
vsim ground truth. No control change is exercised (Phase 1 is read-only on
actuators) — we just confirm the fused estimate is trustworthy before any
controller leans on it.

The vsim baro model derives pressure from the true altitude (-pos_z) with the
exact ISA inverse the firmware uses (tools/vsim/src/main.cpp:519, sea-level
101325 Pa), and the spawn is at z≈0, so fused `altitude` / raw `baro_altitude`
both read AGL-above-spawn with no MSL offset — directly comparable to -pos_z.

Runs the real firmware-in-the-loop, so it is marked `integration` and skips if
the binaries aren't built. No GCS bridges (gcs=False) so it never touches the
shared /tmp/vsim_pose, UART2 advert, or control socket singletons.
"""
import time

import pytest

NAV_ARMED = 4
NAV_IN_AIR = 5


@pytest.mark.integration
def test_vertical_estimate_tracks_truth(require_binaries, gcs_conf):
    from vayu_headless import Pilot, SitlSession
    lab = SitlSession(gcs=False, conf=gcs_conf)
    try:
        pilot = Pilot(lab, alt=-5.0)

        # --- climb: the fused climb rate should go positive while ascending ---
        pilot.arm_takeoff(alt=-5.0)
        saw_positive_climb = False
        t0 = time.time()
        while time.time() - t0 < 4.0:
            vs = lab.telem.get("VerticalState")
            if vs is not None and vs.valid and vs.climb_rate > 0.5:
                saw_positive_climb = True
                break
            time.sleep(0.05)
        assert saw_positive_climb, "VERT never reported a positive climb during takeoff"

        # --- settle in hover, then compare the fused estimate to ground truth ---
        time.sleep(4.0)

        tr = lab.truth()
        assert tr is not None, "no ground-truth pose after takeoff"
        truth_alt = -tr["pos"][2]          # NED down -> up-positive AGL
        truth_climb = -tr["vel"][2]        # NED vel down -> climb rate up-positive

        # streaming, not a one-shot burst
        assert lab.telem_counts.get("VerticalState", 0) > 0, \
            "no VerticalState telemetry decoded"

        vs = lab.telem.get("VerticalState")
        assert vs is not None, "no VerticalState message"
        assert vs.valid, "VERT filter never seeded (valid=0)"

        # raw baro recovers true altitude near-exactly (model is the ISA inverse)
        assert abs(vs.baro_altitude - truth_alt) < 0.6, \
            f"raw baro alt {vs.baro_altitude:.2f} != truth {truth_alt:.2f}"

        # fused altitude tracks truth (allow filter lag + the 2-state offset)
        assert abs(vs.altitude - truth_alt) < 1.0, \
            f"fused alt {vs.altitude:.2f} != truth {truth_alt:.2f}"

        # the filter follows baro rather than diverging
        assert abs(vs.altitude - vs.baro_altitude) < 1.0, \
            f"fused alt {vs.altitude:.2f} diverged from baro {vs.baro_altitude:.2f}"

        # settled hover: fused climb rate is small and tracks the true climb sign
        assert abs(vs.climb_rate) < 0.8, \
            f"fused climb_rate {vs.climb_rate:.2f} not settled in hover"
        assert abs(vs.climb_rate - truth_climb) < 0.8, \
            f"fused climb_rate {vs.climb_rate:.2f} != truth {truth_climb:.2f}"

        # craft is genuinely flying (sanity that we tested a real hover). Wide
        # band: the SITL outer loop is known-wobbly and settles anywhere ~5-7 m
        # for a 5 m target (host-scheduling-sensitive) — this test verifies the
        # VERT estimate vs truth, not guidance tuning, so it only needs "airborne
        # and roughly at the commanded height".
        assert 3.5 < truth_alt < 8.0, f"craft not airborne near target (got {truth_alt:.2f})"
        # Armed and flying: ARMED on the ground, or IN_AIR once the baro-driven
        # takeoff detector fires (Phase 2) — both are "armed". (Not FAILSAFE etc.)
        hb = lab.telem.get("Heartbeat")
        assert hb is None or hb.nav_state in (NAV_ARMED, NAV_IN_AIR), \
            f"expected ARMED/IN_AIR, nav={getattr(hb, 'nav_state', None)}"

        pilot.land()
        time.sleep(0.5)
    finally:
        lab.close()
