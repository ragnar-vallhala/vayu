#!/usr/bin/env python3
"""Fidelity: position TRACKING over a box path, free flight.

Fly a square via waypoints and measure how closely the craft tracks the active
target (mean / max horizontal error). Exercises the full outer position loop +
estimator together.

    python fidelity/tracking.py [--size 5] [--dwell 4] [--vveh path]
"""
import argparse
import math
import os
import time

from vayu_headless import SitlSession, Pilot
from vayu_headless.transport.vsim import quat_to_euler

import _fidelity as F
from _common import Recorder  # noqa: E402

COLS = ["t", "tgt_n", "tgt_e", "n", "e", "alt"]


def run(size=5.0, alt=-5.0, dwell=4.0, vveh=None, csv=None):
    csv = csv or os.path.join(F.OUT_DIR, "tracking.csv")
    os.makedirs(F.OUT_DIR, exist_ok=True)
    rec = Recorder(csv, COLS)
    box = [(size, 0.0), (size, size), (0.0, size), (0.0, 0.0)]
    frame = None
    with SitlSession(**F.session_kwargs(vveh)) as s:
        frame = F.verify_frame(s, "tracking")
        pilot = Pilot(s, alt=alt)
        pilot.arm_takeoff(alt=alt)
        time.sleep(3.0)
        for (tn, te) in box:
            pilot.goto([(tn, te)], alt=alt)
            t1 = time.time()
            while time.time() - t1 < dwell:
                tr, _, _ = F.read_att(s, quat_to_euler)
                if tr:
                    rec.add(time.time() - t1 + 0, tn, te,
                            tr["pos"][0], tr["pos"][1], -tr["pos"][2])
                time.sleep(0.02)
        pilot.land()
    rec.close()

    if not rec.rows:
        print("  tracking: no data"); return []
    # error to the active target, ignoring the first second after each switch
    errs = [math.hypot(r[1] - r[3], r[2] - r[4]) for r in rec.rows]
    m = {"mean_err_m": sum(errs) / len(errs), "max_err_m": max(errs)}
    score = F.write_metrics("tracking", m, frame)
    print(f"  tracking: mean_err={m['mean_err_m']:.2f} m, max_err={m['max_err_m']:.2f} m"
          f"  ->  fidelity {score}/100")
    return rec.rows


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--size", type=float, default=5.0)
    ap.add_argument("--alt", type=float, default=-5.0)
    ap.add_argument("--dwell", type=float, default=4.0)
    ap.add_argument("--vveh", default="")
    ap.add_argument("--csv", default="")
    a = ap.parse_args()
    run(size=a.size, alt=a.alt, dwell=a.dwell, vveh=a.vveh or None, csv=a.csv or None)
