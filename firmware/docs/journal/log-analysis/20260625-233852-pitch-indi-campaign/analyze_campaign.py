#!/usr/bin/env python3
"""Single, segmented, sensor->actuator pipeline analysis across the whole
pitch-indi campaign.

Design principle (per request): NO blanket averaging over a bin. Every metric
is computed inside a homogeneous flight regime (IDLE / SPOOL / AIRBORNE) or a
single active window. Ground bouncing, idle, free flight and crashes are
different plants and are never mixed.

Pipeline stages reported, per regime/window:
  SENSOR     gyro + accel vibration (|acc| deviation), aliasing caveat
  ESTIMATOR  attitude tracking error (sp - curr); rate fed from gyro directly
  PILOT      RC stick activity in the oscillation band (cause vs symptom)
  CONTROL    rate RMS, controller-output saturation, limit-cycle frequency
  ACTUATOR   motor spread, floor/rail %, both-ends saturation, F/B asymmetry
  TIMING     inner_dt cadence + jitter

Usage:  python3 analyze_campaign.py            # all bins
        python3 analyze_campaign.py <file.bin> ...
"""
import os, sys, glob
import numpy as np
import _pipeline as P
import _segment as S

HERE = os.path.dirname(os.path.abspath(__file__))
G = 9.80665
FLOOR_NEAR = 0.06     # motor cmd <= this == pinned at idle floor (FLOOR=0.005)
RAIL_NEAR = 0.95      # motor cmd >= this == railed


def domfreq(x, fs, fmin=0.5):
    x = np.asarray(x, float)
    if len(x) < 16:
        return float("nan"), float("nan")
    x = x - x.mean()
    n = len(x)
    F = np.abs(np.fft.rfft(x * np.hanning(n)))
    fr = np.fft.rfftfreq(n, 1.0 / fs)
    band = fr > fmin
    if not band.any():
        return float("nan"), float("nan")
    k = np.argmax(F * band)
    rel = F[k] / F[band].sum()        # share of in-band energy in the peak bin
    return fr[k], rel


def window_metrics(d, s):
    ct, mot, imu = d["ct"], d["mot"], d["imu"]
    t = ct[:, 0]
    sl = slice(s["i0"], s["i1"])
    tt = t[sl]
    fs = len(tt) / max(1e-6, tt[-1] - tt[0])
    m = {}
    m["fs"] = fs
    m["thr"] = ct[sl, P.CT_I["thr"]].mean()
    m["thr_max"] = ct[sl, P.CT_I["thr"]].max()
    # control
    pr = ct[sl, P.CT_I["prate"]]; rr = ct[sl, P.CT_I["rrate"]]
    pout = ct[sl, P.CT_I["pitch_out"]]; rout = ct[sl, P.CT_I["roll_out"]]
    m["pitchRMS"] = pr.std(); m["rollRMS"] = rr.std()
    m["poutSat"] = 100 * np.mean(np.abs(pout) >= 0.99)
    m["routSat"] = 100 * np.mean(np.abs(rout) >= 0.99)
    m["pfreq"], m["prel"] = domfreq(pr, fs)
    # estimator / attitude tracking
    m["pAngErr"] = (ct[sl, P.CT_I["pitch_sp"]] - ct[sl, P.CT_I["pitch"]]).std()
    m["rAngErr"] = (ct[sl, P.CT_I["roll_sp"]] - ct[sl, P.CT_I["roll"]]).std()
    # actuator
    mm = (mot[:, 0] >= s["t0"]) & (mot[:, 0] <= s["t1"])
    if mm.sum() > 4:
        M = mot[mm, 1:5]
        lo = M.min(axis=1); hi = M.max(axis=1)
        m["mspread"] = (hi - lo).mean()
        m["floor"] = 100 * np.mean(lo <= FLOOR_NEAR)
        m["rail"] = 100 * np.mean(hi >= RAIL_NEAR)
        m["bothsat"] = 100 * np.mean((lo <= FLOOR_NEAR) & (hi >= RAIL_NEAR))
        front = M[:, [0, 3]].mean(); back = M[:, [1, 2]].mean()
        m["fb_asym"] = 100 * (front - back) / max(back, 1e-3)
        pdiff = M[:, [0, 3]].mean(1) - M[:, [1, 2]].mean(1)
        m["mfreq"], _ = domfreq(pdiff, fs)
    else:
        for k in ("mspread", "floor", "rail", "bothsat", "fb_asym", "mfreq"):
            m[k] = float("nan")
    # sensor vibration (|acc| deviation removes gravity-projection; amplitude
    # only -- spectrum is unreliable at this aliased ~36-48 Hz sample rate)
    mi = (imu[:, 0] >= s["t0"]) & (imu[:, 0] <= s["t1"])
    if mi.sum() > 4:
        am = np.linalg.norm(imu[mi, 1:4], axis=1) / G
        m["accVib"] = am.std()
        m["gyroVib"] = np.linalg.norm(imu[mi, 4:7], axis=1).std()
    else:
        m["accVib"] = m["gyroVib"] = float("nan")
    return m


def rc_band_activity(d, s):
    """Std of each moving RC channel within the window -> is the pilot driving
    the oscillation, or holding still while it happens?"""
    rc = d["rc"]
    if len(rc) == 0:
        return {}
    mm = (rc[:, 0] >= s["t0"]) & (rc[:, 0] <= s["t1"])
    out = {}
    for i in range(8):
        c = rc[:, 1 + i]
        if c.std() > 5 and mm.sum() > 5:
            out[i] = rc[mm, 1 + i].std()
    return out


def analyze(path):
    d = P.load(path)
    segs, info = S.segment(d)
    ct = d["ct"]; t = ct[:, 0]
    span = t[-1] - t[0]
    name = os.path.basename(os.path.dirname(path)) or os.path.basename(path)
    idt = ct[:, P.CT_I["inner_dt"]]

    # regime budget
    budget = {}
    for s in segs:
        budget[s["kind"]] = budget.get(s["kind"], 0.0) + s["dur"]

    print("=" * 78)
    print(f"{name}   span {span:.1f}s   loop {1/np.median(idt):.0f}Hz "
          f"(inner_dt {np.median(idt)*1e3:.2f}ms, jitter {idt.std()*1e3:.2f}ms)")
    print("  regime budget: " + "  ".join(
        f"{k} {100*v/span:.0f}%" for k, v in sorted(budget.items())))

    fly = [s for s in segs if s["kind"] in ("AIRBORNE", "SPOOL") and s["dur"] >= 1.0]
    print(f"  active windows >=1s: {len(fly)}   "
          f"impact/upset events: {len(info['events'])}")
    if info["events"]:
        accs = [e["acc_max_g"] for e in info["events"]]
        gyr = [e["gyro_max"] for e in info["events"]]
        print(f"    impacts |acc|max {max(accs):.1f}g  gyro upset max {max(gyr):.0f}dps")

    hdr = ("  win  kind     t0    dur  thr/mx | pRMS rRMS pf(Hz) pSat | "
           "mSprd flr rail both fb% mf(Hz) | aVib accErr")
    print(hdr)
    for n, s in enumerate(fly):
        m = window_metrics(d, s)
        rc = rc_band_activity(d, s)
        # flag pilot activity only if a stick moves a lot near the cycle band
        rc_flag = "stick" if any(v > 60 for v in rc.values()) else "still"
        print(f"  #{n:<2d} {s['kind'][:8]:8s} {s['t0']:5.1f} {s['dur']:4.1f} "
              f"{m['thr']:.2f}/{m['thr_max']:.2f} | "
              f"{m['pitchRMS']:4.0f} {m['rollRMS']:4.0f} {m['pfreq']:5.2f} "
              f"{m['poutSat']:3.0f}% | "
              f"{m['mspread']:.2f} {m['floor']:3.0f} {m['rail']:3.0f} "
              f"{m['bothsat']:3.0f} {m['fb_asym']:+4.0f} {m['mfreq']:5.2f} | "
              f"{m['accVib']:.2f}g {m['pAngErr']:3.0f}d [{rc_flag}]")
    return d, segs, info


def main():
    bins = sys.argv[1:] or sorted(glob.glob(os.path.join(HERE, "*", "*.bin")))
    print("LEGEND  pRMS/rRMS=pitch/roll rate RMS dps | pf=pitch-rate dom freq | "
          "pSat=pitch-out |u|>=.99 %% |\n        mSprd=motor max-min | "
          "flr/rail/both=%% motors at idle-floor/rail/both | fb%%=front-vs-back "
          "asym |\n        mf=motor pitch-diff dom freq | aVib=|acc| RMS (g, "
          "aliased) | accErr=pitch angle sp-curr RMS deg\n")
    for b in bins:
        analyze(b)


if __name__ == "__main__":
    main()
