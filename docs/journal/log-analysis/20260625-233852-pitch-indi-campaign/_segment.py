#!/usr/bin/env python3
"""Flight-regime segmentation. Operates on the ControlTrace timeline (the
control loop's own clock) and labels every sample into a regime, then groups
contiguous runs into segments. Thresholds are data-driven (see campaign
calibration). NO blanket averaging — downstream metrics run per-segment.

Regimes
-------
IDLE     throttle ~0, motors not commanding lift (disarmed or ground idle)
SPOOL    throttle present but craft on the ground (no sustained altitude gain)
AIRBORNE throttle present AND altitude elevated above the running ground floor
IMPACT   accel-magnitude shock (ground contact / crash); annotated as events

We deliberately separate AIRBORNE from SPOOL because a ground-resonating frame
and a free-flying frame are different plants — averaging across them is exactly
the "blanket" error to avoid.
"""
import numpy as np
import _pipeline as P

THR_ACTIVE = 0.15     # throttle above this = motors meaningfully commanding
MIN_ACTIVE_S = 0.6    # ignore sub-second throttle blips
ALT_RISE_M = 1.0      # altitude above running ground floor => airborne
IMPACT_G = 2.5        # |acc| / g above this = shock event
GYRO_TUMBLE = 300.0   # |gyro| dps: tumbling/upset


def _ground_floor(t_vs, alt, t):
    """Running minimum altitude over the trailing few seconds = 'ground' ref."""
    if len(t_vs) == 0:
        return np.full_like(t, np.nan)
    a = np.interp(t, t_vs, alt)
    floor = np.copy(a)
    # trailing-window running min (~6 s) to track the local ground level
    win = max(1, int(len(t) / max(1e-6, (t[-1] - t[0])) * 6))
    for i in range(len(a)):
        lo = max(0, i - win)
        floor[i] = np.min(a[lo:i + 1])
    return a, floor


def segment(d):
    ct = d["ct"]
    if len(ct) < 10:
        return [], {}
    t = ct[:, P.CT_I["t"]]
    thr = ct[:, P.CT_I["thr"]]
    gm = np.linalg.norm(ct[:, [P.CT_I["rrate"], P.CT_I["prate"],
                               P.CT_I["yrate"]]], axis=1)

    # altitude on the CT clock
    vs = d["vs"]
    if len(vs):
        alt_i, floor = _ground_floor(vs[:, P.VS_I["t"]], vs[:, P.VS_I["altitude"]], t)
        agl_i = np.interp(t, vs[:, P.VS_I["t"]], vs[:, P.VS_I["agl"]])
    else:
        alt_i = floor = agl_i = np.full_like(t, np.nan)

    # accel magnitude on the CT clock (impacts)
    imu = d["imu"]
    if len(imu):
        am = np.linalg.norm(imu[:, 1:4], axis=1)
        am_i = np.interp(t, imu[:, P.IMU_I["t"]], am)
    else:
        am_i = np.full_like(t, np.nan)

    active = thr > THR_ACTIVE
    elevated = (alt_i - floor) > ALT_RISE_M
    airborne_like = active & (elevated | (agl_i > ALT_RISE_M))

    # label
    lab = np.where(active, np.where(airborne_like, "AIRBORNE", "SPOOL"), "IDLE")

    # group contiguous runs
    segs = []
    i = 0
    while i < len(lab):
        j = i
        while j < len(lab) and lab[j] == lab[i]:
            j += 1
        dur = t[j - 1] - t[i]
        # demote too-short active blips to the surrounding regime label SPOOL->IDLE
        kind = lab[i]
        if kind in ("SPOOL", "AIRBORNE") and dur < MIN_ACTIVE_S:
            kind = "IDLE"
        segs.append({"kind": kind, "i0": i, "i1": j, "t0": t[i], "t1": t[j - 1],
                     "dur": dur})
        i = j

    # merge adjacent same-kind after demotion
    merged = []
    for s in segs:
        if merged and merged[-1]["kind"] == s["kind"]:
            merged[-1]["i1"] = s["i1"]; merged[-1]["t1"] = s["t1"]
            merged[-1]["dur"] = merged[-1]["t1"] - merged[-1]["t0"]
        else:
            merged.append(s)

    # impact events (|acc| shock or tumble), with throttle context
    g = 9.80665
    shock = (am_i > IMPACT_G * g) | (gm > GYRO_TUMBLE)
    events = []
    k = 0
    while k < len(shock):
        if shock[k]:
            m = k
            while m < len(shock) and shock[m]:
                m += 1
            events.append({"t0": t[k], "t1": t[m - 1],
                           "acc_max_g": np.nanmax(am_i[k:m]) / g,
                           "gyro_max": np.nanmax(gm[k:m]),
                           "thr": np.nanmedian(thr[k:m])})
            k = m
        else:
            k += 1

    ctx = {"t": t, "thr": thr, "gm": gm, "alt": alt_i, "floor": floor,
           "agl": agl_i, "am": am_i}
    return merged, {"events": events, "ctx": ctx}


def summarize(d, label=""):
    segs, info = segment(d)
    ct = d["ct"]
    t = ct[:, P.CT_I["t"]]
    span = t[-1] - t[0] if len(t) else 0
    agg = {}
    for s in segs:
        agg.setdefault(s["kind"], 0.0)
        agg[s["kind"]] += s["dur"]
    print(f"\n=== {label} ===  span {span:.1f}s, {len(segs)} segments")
    for k in ("IDLE", "SPOOL", "AIRBORNE"):
        if k in agg:
            print(f"   {k:9s} {agg[k]:6.1f}s total ({100*agg[k]/span:4.1f}%)")
    fly = [s for s in segs if s["kind"] in ("AIRBORNE", "SPOOL") and s["dur"] >= 1.0]
    print(f"   active windows >=1s: {len(fly)}")
    for s in fly:
        ctx = info["ctx"]
        sl = slice(s["i0"], s["i1"])
        print(f"     {s['kind']:8s} t {s['t0']:6.1f}-{s['t1']:6.1f} ({s['dur']:4.1f}s)"
              f"  thr {ctx['thr'][sl].mean():.2f}/max{ctx['thr'][sl].max():.2f}"
              f"  dAlt {ctx['alt'][sl].max()-ctx['floor'][sl][0]:+.1f}m"
              f"  pitchRMS {ct[sl,P.CT_I['prate']].std():.0f}"
              f"  rollRMS {ct[sl,P.CT_I['rrate']].std():.0f} dps")
    print(f"   impact/upset events: {len(info['events'])}")
    for e in info["events"][:12]:
        print(f"     t {e['t0']:6.1f}  |acc|max {e['acc_max_g']:4.1f}g"
              f"  gyromax {e['gyro_max']:4.0f}dps  thr {e['thr']:.2f}")
    return segs, info


if __name__ == "__main__":
    import sys, os
    for f in sys.argv[1:]:
        d = P.load(f)
        summarize(d, os.path.basename(os.path.dirname(f)) or f)
