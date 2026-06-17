#!/usr/bin/env python3
"""Example mission: fly a square box and land, using only the public SDK API.

    from vayu_headless import SitlSession
    from box_mission import fly_box
    with SitlSession(gcs=False) as s:
        fly_box(s)

Or run standalone:  python examples/box_mission.py --side 8 --alt -5
"""
import time

from vayu_headless import Pilot


def fly_box(session, side=8.0, alt=-5.0, timeout=45.0, land=True):
    """Take off, fly a `side`×`side` box at `alt`, optionally land. Returns the
    final Pilot.last telemetry dict."""
    pilot = Pilot(session, alt=alt)
    pilot.arm_takeoff(alt=alt)
    pilot.goto([(side, 0), (side, side), (0, side), (0, 0)], alt=alt)
    t0 = time.time()
    while time.time() - t0 < timeout and not pilot.reached_last():
        time.sleep(0.2)
    result = dict(pilot.last or {})
    if land:
        pilot.land()
    return result


def main():
    import argparse

    from vayu_headless import SitlSession

    ap = argparse.ArgumentParser()
    ap.add_argument("--side", type=float, default=8.0)
    ap.add_argument("--alt", type=float, default=-5.0)
    ap.add_argument("--gcs", action="store_true")
    args = ap.parse_args()
    with SitlSession(gcs=args.gcs) as s:
        r = fly_box(s, side=args.side, alt=args.alt)
        print(f"final pos=({r.get('x', 0):+.1f},{r.get('y', 0):+.1f}) "
              f"alt={-r.get('z', 0):+.1f}m")


if __name__ == "__main__":
    main()
