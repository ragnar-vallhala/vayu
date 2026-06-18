#!/usr/bin/env python3
"""Fidelity: HOVER hold.

Takeoff, hold the origin at a fixed altitude, and log station-keeping drift,
altitude hold, attitude steadiness, and the estimator-vs-truth gap. Free flight
(outer loop active), so this is where the attitude under-read (deferred #1) and
any slow position drift show up.

    python fidelity/hover.py [--alt -5] [--secs 25] [--vveh path]
"""
import argparse
import os
import time

from vayu_headless import SitlSession, Pilot
from vayu_headless.transport.vsim import quat_to_euler

import _fidelity as F
from _common import Recorder  # noqa: E402  (examples/ on path)

COLS = ["t", "n", "e", "alt", "true_roll", "true_pitch", "true_yaw",
        "est_roll", "est_pitch", "est_yaw"]
IDX = {c: i for i, c in enumerate(COLS)}


def run(alt=-5.0, secs=25.0, vveh=None, csv=None):
    csv = csv or os.path.join(F.OUT_DIR, "hover.csv")
    os.makedirs(F.OUT_DIR, exist_ok=True)
    rec = Recorder(csv, COLS)
    frame = None
    with SitlSession(**F.session_kwargs(vveh)) as s:
        frame = F.verify_frame(s, "hover")
        pilot = Pilot(s, alt=alt)
        pilot.arm_takeoff(alt=alt)
        time.sleep(3.0)                         # settle at hover
        pilot.goto([(0.0, 0.0)], alt=alt)       # hold origin
        t0 = time.time()
        while time.time() - t0 < secs:
            tr, tru, est = F.read_att(s, quat_to_euler)
            if tr:
                rec.add(time.time() - t0, tr["pos"][0], tr["pos"][1],
                        -tr["pos"][2], tru[0], tru[1], tru[2],
                        est[0], est[1], est[2])
            time.sleep(0.02)
        pilot.land()
    rec.close()

    m = F.hover_metrics(rec.rows, IDX)
    gap = F.est_truth_gap(rec.rows, IDX["est_pitch"], IDX["true_pitch"])
    m["est_pitch_mae_deg"] = gap["mae_deg"]
    score = F.write_metrics("hover", m, frame)
    print(f"  hover: drift={m['max_drift_m']:.2f} m, alt_rms={m['alt_rms_m']:.2f} m, "
          f"att_rms={m['att_rms_deg']:.2f} deg  ->  fidelity {score}/100")
    return rec.rows


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--alt", type=float, default=-5.0)
    ap.add_argument("--secs", type=float, default=25.0)
    ap.add_argument("--vveh", default="")
    ap.add_argument("--csv", default="")
    a = ap.parse_args()
    run(alt=a.alt, secs=a.secs, vveh=a.vveh or None, csv=a.csv or None)
