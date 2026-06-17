#!/usr/bin/env python3
"""FIGURE-8 trajectory (free flight).

Takes off, then chases a setpoint that sweeps a Gerono lemniscate
    N(θ) = size·sin(θ),   E(θ) = size·sin(θ)·cos(θ)
by continuously re-issuing Pilot.goto() with the moving target — so the real FC
flies the path under its own control. Records the TRUE trajectory vs the
commanded path. Pass --gcs to watch it in Navigator (Attach Ext + SITL UART2).

    python examples/figure8.py --size 8 --alt -5 --laps 2 --csv fig8.csv
    python examples/figure8.py --size 10 --gcs --gcs-wait 20   # watch live
"""
import argparse
import math
import time

from vayu_headless import Pilot, SitlSession

from _common import Recorder, gcs_conf


def lemniscate(theta, size):
    return (size * math.sin(theta), size * math.sin(theta) * math.cos(theta))


def run(size=8.0, alt=-5.0, laps=2.0, lap_secs=20.0, gcs=False, gcs_wait=0.0,
        csv=None):
    rec = Recorder(csv, ["t", "cmd_n", "cmd_e", "n", "e", "alt", "wp_reached"])
    with SitlSession(gcs=gcs, conf=gcs_conf()) as s:
        if gcs:
            print("  ▶ Navigator: click 'Attach Ext' + Connect → 'SITL UART2'")
            if gcs_wait > 0:
                time.sleep(gcs_wait)
        pilot = Pilot(s, alt=alt)
        pilot.arm_takeoff(alt=alt)
        time.sleep(2.0)                                   # settle at hover

        t0 = time.time()
        dt = 0.05
        total = laps * lap_secs
        next_t = t0
        while time.time() - t0 < total:
            t = time.time() - t0
            theta = 2 * math.pi * (t / lap_secs)
            cn, ce = lemniscate(theta, size)
            pilot.goto([(cn, ce)], alt=alt)               # moving setpoint
            last = pilot.last or {}
            rec.add(t, cn, ce, last.get("x", 0.0), last.get("y", 0.0),
                    -last.get("z", 0.0), 0)
            next_t += dt
            sl = next_t - time.time()
            if sl > 0:
                time.sleep(sl)
        pilot.goto([(0.0, 0.0)], alt=alt)                 # recover to centre
        time.sleep(2.0)
        pilot.land()
    rec.close()
    if rec.rows:
        ns = [r[3] for r in rec.rows]
        es = [r[4] for r in rec.rows]
        # mean tracking error vs the commanded lemniscate
        err = sum(math.hypot(r[1] - r[3], r[2] - r[4]) for r in rec.rows) / len(rec.rows)
        print(f"figure-8 size={size}m, {laps:g} lap(s): "
              f"flew N∈[{min(ns):+.1f},{max(ns):+.1f}] E∈[{min(es):+.1f},{max(es):+.1f}], "
              f"mean track err={err:.1f}m, {len(rec.rows)} samples"
              + (f" → {csv}" if csv else ""))
    return rec.rows


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--size", type=float, default=8.0)
    ap.add_argument("--alt", type=float, default=-5.0)
    ap.add_argument("--laps", type=float, default=2.0)
    ap.add_argument("--lap-secs", type=float, default=20.0)
    ap.add_argument("--gcs", action="store_true")
    ap.add_argument("--gcs-wait", type=float, default=0.0)
    ap.add_argument("--csv", default="")
    a = ap.parse_args()
    run(a.size, a.alt, a.laps, a.lap_secs, a.gcs, a.gcs_wait, a.csv or None)
