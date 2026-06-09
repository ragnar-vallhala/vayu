#!/usr/bin/env python3
"""Verify (1) outer loop tied to inner rate, and (2) flight-mode round-trip.

(1) Reads control telemetry and checks outer_dt ≈ OUTER_LOOP_DECIM × inner_dt
    (inner now faster than outer — the conventional cascade). At the 1 kHz IMU
    default that means inner ≈ 1 ms, outer ≈ 4 ms.
(2) Sends CMD_SET_FLIGHT_MODE {acro, angle, release} and confirms the firmware's
    SYSTEM_ORIGIN_FLIGHT_MODE telemetry reflects mode + source each time.
"""
import os
import statistics
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import protocol as P            # noqa: E402
from sitl import SitlStack      # noqa: E402

ROOT = os.path.join(os.path.dirname(__file__), "..", "..")


def median_dts(stack, secs=1.5):
    samples = stack.collect(secs)
    outer = [s[1]["outer_dt"] for s in samples if s[1]["outer_dt"] > 0]
    inner = [s[1]["inner_dt"] for s in samples if s[1]["inner_dt"] > 0]
    return (statistics.median(outer) if outer else None,
            statistics.median(inner) if inner else None,
            len(samples))


def wait_mode(stack, want_mode, want_src, timeout=3.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if stack._last_mode == want_mode and stack._last_mode_src == want_src:
            return True
        time.sleep(0.02)
    return False


def main():
    stack = SitlStack(ROOT, quiet=True)            # rates default (1000, 8000, 120)
    print("launching SITL stack (imu=1000, physics=8000) ...")
    stack.start()
    ok_all = True
    try:
        # ---- (1) loop-rate tie -------------------------------------------
        outer, inner, n = median_dts(stack)
        print("\n== (1) outer↔inner loop rate ==")
        if outer and inner:
            ratio = outer / inner
            print(f"  inner_dt={inner*1e3:.3f} ms (~{1/inner:.0f} Hz)   "
                  f"outer_dt={outer*1e3:.3f} ms (~{1/outer:.0f} Hz)   "
                  f"ratio={ratio:.2f}  ({n} samples)")
            inner_faster = inner < outer
            ratio_ok = 3.0 <= ratio <= 5.0
            print(f"  inner faster than outer: {inner_faster}    "
                  f"ratio≈DECIM(4): {ratio_ok}")
            ok_all &= inner_faster and ratio_ok
        else:
            print("  FAIL: no dt telemetry"); ok_all = False

        # ---- (2) flight-mode round-trip ----------------------------------
        print("\n== (2) flight-mode message (to/fro) ==")
        time.sleep(0.5)
        print(f"  initial: mode={stack._last_mode} src={stack._last_mode_src} "
              f"(expect 0/0 = angle/RC)")
        cases = [
            ("acro (GCS)",    P.FLIGHT_MODE_ACRO,    P.FLIGHT_MODE_ACRO,  1),
            ("angle (GCS)",   P.FLIGHT_MODE_ANGLE,   P.FLIGHT_MODE_ANGLE, 1),
            ("release to RC", P.FLIGHT_MODE_RELEASE, P.FLIGHT_MODE_ANGLE, 0),
        ]
        for label, arg, want_mode, want_src in cases:
            stack.set_flight_mode(arg)
            got = wait_mode(stack, want_mode, want_src)
            print(f"  send {label:<14} -> mode={stack._last_mode} "
                  f"src={stack._last_mode_src}  {'OK' if got else 'FAIL'}")
            ok_all &= got

        print("\n================ VERDICT ================")
        print("PASS" if ok_all else "FAIL")
    finally:
        stack.stop()


if __name__ == "__main__":
    main()
