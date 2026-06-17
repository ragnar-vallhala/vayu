#!/usr/bin/env python3
"""Frequency-sweep (CHIRP) on one axis — for frequency-domain system-ID.

Drives a linear chirp  cmd(t) = amp * sin(2π · (f0 + (f1-f0)·t/T)/2 · t)  on the
chosen axis on a tuning rig, recording the TRUE attitude/rate response. Feed the
CSV to your favourite FFT / Bode estimator to get the airframe's frequency
response.

    python examples/chirp.py --axis roll --amp 0.3 --f0 0.2 --f1 4 --secs 20 --csv chirp.csv
"""
import argparse
import math
import time

from vayu_headless import SitlSession

from _common import Recorder, arm_on_rig, gcs_conf, quat_to_euler

AXES = ("roll", "pitch", "yaw")


def run(axis="roll", amp=0.3, f0=0.2, f1=4.0, secs=20.0, thr=0.5, csv=None):
    rec = Recorder(csv, ["t", "freq", "cmd", "roll", "pitch", "yaw",
                         "wx", "wy", "wz"])
    with SitlSession(rig=True, conf=gcs_conf()) as s:
        arm_on_rig(s, thr=thr)
        t0 = time.time()
        while True:
            t = time.time() - t0
            if t >= secs:
                break
            f = f0 + (f1 - f0) * (t / secs)               # instantaneous frequency
            phase = 2 * math.pi * (f0 + (f1 - f0) * (t / secs) / 2.0) * t
            cmd = amp * math.sin(phase)
            s.stick(thr=thr, **{axis: cmd})
            s.set_rc(swa=2000)
            tr = s.truth()
            if tr:
                r, p, y = quat_to_euler(*tr["quat"])
                wx, wy, wz = tr["omega"]
                rec.add(t, f, cmd, r, p, y, wx, wy, wz)
            time.sleep(0.01)                              # ~100 Hz sampling
    rec.close()
    print(f"chirp {axis}: {f0}→{f1} Hz over {secs:.0f}s, amp={amp}, "
          f"{len(rec.rows)} samples"
          + (f" → {csv}" if csv else ""))
    return rec.rows


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--axis", choices=AXES, default="roll")
    ap.add_argument("--amp", type=float, default=0.3)
    ap.add_argument("--f0", type=float, default=0.2)
    ap.add_argument("--f1", type=float, default=4.0)
    ap.add_argument("--secs", type=float, default=20.0)
    ap.add_argument("--thr", type=float, default=0.5)
    ap.add_argument("--csv", default="")
    a = ap.parse_args()
    run(a.axis, a.amp, a.f0, a.f1, a.secs, a.thr, a.csv or None)
