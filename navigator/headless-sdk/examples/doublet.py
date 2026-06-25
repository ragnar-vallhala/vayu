#!/usr/bin/env python3
"""DOUBLET (+amp then −amp pulse) on one axis — the classic system-ID input.

A doublet excites the airframe symmetrically with minimal net attitude change,
so it's the standard maneuver for fitting a transfer function. Runs on a tuning
rig and records the TRUE attitude/rate response.

    python examples/doublet.py --axis pitch --amp 0.4 --width 0.4 --csv doublet.csv
"""
import argparse
import time

from vayu_headless import SitlSession

from _common import Recorder, arm_on_rig, gcs_conf, quat_to_euler

AXES = ("roll", "pitch", "yaw")


def run(axis="roll", amp=0.4, width=0.4, pre=1.0, post=2.0, thr=0.5, csv=None):
    rec = Recorder(csv, ["t", "cmd", "roll", "pitch", "yaw", "wx", "wy", "wz"])
    with SitlSession(rig=True, conf=gcs_conf()) as s:
        arm_on_rig(s, thr=thr)
        t0 = time.time()
        total = pre + 2 * width + post
        while True:
            t = time.time() - t0
            if t >= total:
                break
            if pre <= t < pre + width:
                cmd = amp                                 # first half: +
            elif pre + width <= t < pre + 2 * width:
                cmd = -amp                                # second half: −
            else:
                cmd = 0.0
            s.stick(thr=thr, **{axis: cmd})
            s.set_rc(swa=2000)
            tr = s.truth()
            if tr:
                r, p, y = quat_to_euler(*tr["quat"])
                wx, wy, wz = tr["omega"]
                rec.add(t, cmd, r, p, y, wx, wy, wz)
            time.sleep(0.02)
    rec.close()
    print(f"doublet {axis}: ±{amp} × {width}s each, {len(rec.rows)} samples"
          + (f" → {csv}" if csv else ""))
    return rec.rows


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--axis", choices=AXES, default="roll")
    ap.add_argument("--amp", type=float, default=0.4)
    ap.add_argument("--width", type=float, default=0.4)
    ap.add_argument("--pre", type=float, default=1.0)
    ap.add_argument("--post", type=float, default=2.0)
    ap.add_argument("--thr", type=float, default=0.5)
    ap.add_argument("--csv", default="")
    a = ap.parse_args()
    run(a.axis, a.amp, a.width, a.pre, a.post, a.thr, a.csv or None)
