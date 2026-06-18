#!/usr/bin/env python3
"""Fidelity: attitude STEP response (roll / pitch) on a tuning rig.

Translation is pinned (rig=True) so the inner attitude loop is measured cleanly:
command a step on one axis and record the TRUE angle (ground truth) plus the
estimate. Yields rise/overshoot/settle/steady-state-error per axis — the core
control-loop fidelity numbers. (Yaw has its own script.)

    python fidelity/attitude_steps.py --axis roll [--amp 0.4] [--vveh path]
"""
import argparse
import os
import time

from vayu_headless import SitlSession
from vayu_headless.transport.vsim import quat_to_euler

import _fidelity as F
from _common import Recorder, arm_on_rig  # noqa: E402

AXES = {"roll": 0, "pitch": 1}
COLS = ["t", "cmd", "true_roll", "true_pitch", "true_yaw",
        "est_roll", "est_pitch", "est_yaw"]


def run(axis="roll", amp=0.4, pre=1.0, hold=2.0, thr=0.5, vveh=None, csv=None):
    name = "step_" + axis
    csv = csv or os.path.join(F.OUT_DIR, name + ".csv")
    os.makedirs(F.OUT_DIR, exist_ok=True)
    rec = Recorder(csv, COLS)
    ai = AXES[axis]
    frame = None
    with SitlSession(rig=True, **F.session_kwargs(vveh)) as s:
        frame = F.verify_frame(s, name)
        arm_on_rig(s, thr=thr)
        t0 = time.time()
        total = pre + hold + 1.5
        while time.time() - t0 < total:
            t = time.time() - t0
            cmd = amp if pre <= t < pre + hold else 0.0
            s.stick(thr=thr, **{axis: cmd})
            s.set_rc(swa=2000)
            _, tru, est = F.read_att(s, quat_to_euler)
            rec.add(t, cmd, tru[0], tru[1], tru[2], est[0], est[1], est[2])
            time.sleep(0.02)
    rec.close()

    # true angle for the commanded axis is the step signal; measure over the
    # HOLD window only (exclude the post-release return so SS is the plateau).
    col_true = 2 + ai
    hold_end = pre + hold
    seg = [r for r in rec.rows if r[0] <= hold_end]
    ts = [r[0] for r in seg]
    sig = [r[col_true] for r in seg]
    peak = max(sig, key=abs) if sig else 0.0
    m = F.step_metrics(ts, sig, peak, pre)   # measure against achieved peak
    m["target"] = peak
    gap = F.est_truth_gap(rec.rows, 5 + ai, col_true)   # est col = 5+ai
    m["est_mae_deg"] = gap["mae_deg"]
    m["est_underread_ratio"] = gap["underread_ratio"]
    score = F.write_metrics(name, m, frame)
    rise = m.get("rise_s")
    print(f"  {name}: peak={peak:+.1f} deg, rise={rise and round(rise,3)}s, "
          f"overshoot={m.get('overshoot_pct',0):.0f}%, "
          f"ss_err={m.get('ss_err',0):.2f} deg  ->  fidelity {score}/100")
    return rec.rows


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--axis", choices=list(AXES), default="roll")
    ap.add_argument("--amp", type=float, default=0.4)
    ap.add_argument("--vveh", default="")
    ap.add_argument("--csv", default="")
    a = ap.parse_args()
    run(axis=a.axis, amp=a.amp, vveh=a.vveh or None, csv=a.csv or None)
