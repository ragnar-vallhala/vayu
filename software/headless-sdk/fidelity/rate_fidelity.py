#!/usr/bin/env python3
"""Aggregate the per-maneuver fidelity sidecars (out/*.json) into one scorecard.

Prints a table of each maneuver's headline metric + score, the frame that was
verified for the runs, and a weighted overall control-loop fidelity score.

    python fidelity/rate_fidelity.py
"""
import glob
import json
import os

import _fidelity as F

# Relative weight of each maneuver in the overall score.
WEIGHTS = {"step_roll": 1.0, "step_pitch": 1.0, "yaw": 0.8, "hover": 1.2,
           "throttle": 0.8, "tracking": 1.2}

HEADLINE = {
    "step_roll":  lambda m: f"peak {m.get('target',0):+.1f}deg rise {_s(m.get('rise_s'))} ov {m.get('overshoot_pct',0):.0f}%",
    "step_pitch": lambda m: f"peak {m.get('target',0):+.1f}deg rise {_s(m.get('rise_s'))} ov {m.get('overshoot_pct',0):.0f}%",
    "yaw":        lambda m: f"plateau {m.get('target',0):+.1f}deg/s rise {_s(m.get('rise_s'))}",
    "hover":      lambda m: f"drift {m.get('max_drift_m',0):.2f}m alt_rms {m.get('alt_rms_m',0):.2f}m att_rms {m.get('att_rms_deg',0):.1f}deg",
    "throttle":   lambda m: f"climb {m.get('target',0):+.1f}m ss_err {m.get('ss_err',0):.2f}m",
    "tracking":   lambda m: f"mean_err {m.get('mean_err_m',0):.2f}m max {m.get('max_err_m',0):.2f}m",
}


def _s(x):
    return "-" if x is None else f"{x:.2f}s"


def main():
    files = sorted(glob.glob(os.path.join(F.OUT_DIR, "*.json")))
    if not files:
        print("no fidelity results in", F.OUT_DIR, "— run run_all.py first")
        return
    cards = [json.load(open(f)) for f in files]

    frame = next((c["frame"] for c in cards if c.get("frame")), {})
    print("=" * 74)
    print("CONTROL-LOOP FIDELITY SCORECARD")
    if frame:
        warn = "  !! DEFAULTS, NOT THE LOADED FRAME" if frame.get("on_defaults") else ""
        print(f"  frame: mass={frame.get('mass'):.3f}kg motor-diag~{frame.get('motor_diag_mm'):.0f}mm "
              f"M0=({frame.get('m0',[0,0])[0]:+.3f},{frame.get('m0',[0,0])[1]:+.3f}){warn}")
    print("-" * 74)
    print(f"  {'maneuver':<12} {'score':>6}   headline")
    print("-" * 74)

    num = den = 0.0
    for c in cards:
        name, sc, m = c["maneuver"], c.get("score", 0), c.get("metrics", {})
        head = HEADLINE.get(name, lambda _m: "")(m)
        print(f"  {name:<12} {sc:>5.1f}   {head}")
        w = WEIGHTS.get(name, 1.0)
        num += w * sc
        den += w
    overall = num / den if den else 0.0
    # est-vs-truth gap surfaced separately — it's the known fidelity defect.
    gaps = [c["metrics"].get("est_underread_ratio") for c in cards
            if c["metrics"].get("est_underread_ratio")]
    print("-" * 74)
    print(f"  OVERALL CONTROL-LOOP FIDELITY: {overall:.1f}/100")
    if gaps:
        print(f"  (estimator under-read ratio seen: ~{max(gaps):.1f}x — see "
              "docs/deferred/01-attitude-estimate-underread.md)")
    print("=" * 74)
    return overall


if __name__ == "__main__":
    main()
