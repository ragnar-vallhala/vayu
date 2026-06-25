#!/usr/bin/env python3
"""Fidelity: ALTITUDE step (vertical / throttle loop), free flight.

Hover, then command a climb and measure the altitude response: rise, overshoot,
settle and steady-state error against the commanded altitude change.

    python fidelity/throttle_step.py [--alt0 -4] [--alt1 -8] [--vveh path]
"""
import argparse
import os
import time

from vayu_headless import SitlSession, Pilot
from vayu_headless.transport.vsim import quat_to_euler

import _fidelity as F
from _common import Recorder  # noqa: E402

COLS = ["t", "cmd_alt", "alt"]


def run(alt0=-4.0, alt1=-8.0, settle=4.0, watch=8.0, vveh=None, csv=None):
    csv = csv or os.path.join(F.OUT_DIR, "throttle.csv")
    os.makedirs(F.OUT_DIR, exist_ok=True)
    rec = Recorder(csv, COLS)
    frame = None
    with SitlSession(**F.session_kwargs(vveh)) as s:
        frame = F.verify_frame(s, "throttle")
        pilot = Pilot(s, alt=alt0)
        pilot.arm_takeoff(alt=alt0)
        time.sleep(settle)
        pilot.goto([(0.0, 0.0)], alt=alt1)        # commanded climb
        t0 = time.time()
        while time.time() - t0 < watch:
            tr, _, _ = F.read_att(s, quat_to_euler)
            if tr:
                rec.add(time.time() - t0, -alt1, -tr["pos"][2])
            time.sleep(0.02)
        pilot.land()
    rec.close()

    if not rec.rows:
        print("  throttle: no data"); return []
    a0 = rec.rows[0][2]
    ts = [r[0] for r in rec.rows]
    delta = [r[2] - a0 for r in rec.rows]
    target = (-alt1) - a0
    m = F.step_metrics(ts, delta, target, 0.0)
    m["target"] = target
    score = F.write_metrics("throttle", m, frame)
    print(f"  throttle: climb={target:+.1f} m, rise={m.get('rise_s') and round(m['rise_s'],2)}s, "
          f"overshoot={m.get('overshoot_pct',0):.0f}%, ss_err={m.get('ss_err',0):.2f} m"
          f"  ->  fidelity {score}/100")
    return rec.rows


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--alt0", type=float, default=-4.0)
    ap.add_argument("--alt1", type=float, default=-8.0)
    ap.add_argument("--vveh", default="")
    ap.add_argument("--csv", default="")
    a = ap.parse_args()
    run(alt0=a.alt0, alt1=a.alt1, vveh=a.vveh or None, csv=a.csv or None)
