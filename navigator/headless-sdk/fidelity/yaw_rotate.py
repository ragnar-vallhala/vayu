#!/usr/bin/env python3
"""Fidelity: YAW-rate response on a tuning rig.

Command a yaw-stick step and measure the TRUE yaw rate (ground-truth body wz):
how fast and cleanly the heading-rate loop reaches and holds its plateau.

    python fidelity/yaw_rotate.py [--amp 0.5] [--vveh path]
"""
import argparse
import os
import time

from vayu_headless import SitlSession
from vayu_headless.transport.vsim import quat_to_euler

import _fidelity as F
from _common import Recorder, arm_on_rig  # noqa: E402

R2D = 57.29578
COLS = ["t", "cmd", "yaw_rate_dps", "true_yaw", "est_yaw"]


def run(amp=0.5, pre=1.0, hold=2.5, thr=0.5, vveh=None, csv=None):
    csv = csv or os.path.join(F.OUT_DIR, "yaw.csv")
    os.makedirs(F.OUT_DIR, exist_ok=True)
    rec = Recorder(csv, COLS)
    frame = None
    with SitlSession(rig=True, **F.session_kwargs(vveh)) as s:
        frame = F.verify_frame(s, "yaw")
        arm_on_rig(s, thr=thr)
        t0 = time.time()
        total = pre + hold + 1.5
        while time.time() - t0 < total:
            t = time.time() - t0
            cmd = amp if pre <= t < pre + hold else 0.0
            s.stick(thr=thr, yaw=cmd)
            s.set_rc(swa=2000)
            tr, tru, est = F.read_att(s, quat_to_euler)
            wz = tr["omega"][2] * R2D if tr else 0.0
            rec.add(t, cmd, wz, tru[2], est[2])
            time.sleep(0.02)
    rec.close()

    hold_end = pre + hold
    seg = [r for r in rec.rows if r[0] <= hold_end]
    ts = [r[0] for r in seg]
    rate = [r[2] for r in seg]
    plateau = max(rate, key=abs) if rate else 0.0
    m = F.step_metrics(ts, rate, plateau, pre)
    m["target"] = plateau
    score = F.write_metrics("yaw", m, frame)
    print(f"  yaw: plateau={plateau:+.1f} deg/s, rise={m.get('rise_s') and round(m['rise_s'],3)}s, "
          f"overshoot={m.get('overshoot_pct',0):.0f}%  ->  fidelity {score}/100")
    return rec.rows


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--amp", type=float, default=0.5)
    ap.add_argument("--vveh", default="")
    ap.add_argument("--csv", default="")
    a = ap.parse_args()
    run(amp=a.amp, vveh=a.vveh or None, csv=a.csv or None)
