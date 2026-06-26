#!/usr/bin/env python3
"""Fit the fresh sim data and compare to the real on-hardware plant.

Sim plant gain K = wdot/u (deg/s^2 per unit u), with a first-order actuator lag
tau, fit from the rig chirp; hover throttle / motor differential / output
saturation from the free-hover run. Real targets come from the on-hardware
sysid + the segmented real-log analysis in this campaign.
"""
import os, csv
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RAD = 180.0 / np.pi

# ---- real on-hardware references (this campaign + onhw sysid) ----
REAL = {
    "K_roll": 563.0, "K_pitch": 1381.0,   # deg/s^2 per u  (setup.json b / sysid)
    "tau_roll": 0.0209,                    # s  (onhw roll fit)
    "hover_thr": 0.45,                     # real airborne throttle ~0.4-0.5
    "mspread": 0.70, "psat": 0.45,         # real active-window medians
}


def load(name):
    with open(os.path.join(HERE, name)) as f:
        r = list(csv.DictReader(f))
    return {k: np.array([float(row[k]) for row in r]) for k in r[0]}


def fit_K(name, axis):
    """Spectral estimate. For a rigid body alpha = K*u  =>  omega = K*u/(j w),
    so |omega/u| = K/(2 pi f).  Estimate K = 2 pi f * |Omega(f)|/|U(f)| over the
    band where the chirp has energy and input/output cohere."""
    d = load(name)
    t, u, w = d["t"], d["u"], d["omega_radps"] * RAD     # omega -> deg/s
    fs = (len(t) - 1) / (t[-1] - t[0])
    tu = np.linspace(t[0], t[-1], len(t))
    u = np.interp(tu, t, u); w = np.interp(tu, t, w)
    u -= u.mean(); w -= w.mean()
    win = np.hanning(len(u))
    U = np.fft.rfft(u * win); W = np.fft.rfft(w * win)
    fr = np.fft.rfftfreq(len(u), 1.0 / fs)
    mag_u = np.abs(U)
    band = (fr >= 0.4) & (fr <= 4.0) & (mag_u > 0.15 * mag_u.max())
    Kf = 2 * np.pi * fr[band] * np.abs(W[band]) / np.abs(U[band])
    K = float(np.median(Kf))
    # coherence-ish quality: how tight the per-bin K estimates are
    r2 = 1.0 - np.var(Kf) / (np.mean(Kf) ** 2 + 1e-9)
    return K, fr[band], Kf, fs


def hover_stats():
    d = load("hover.csv")
    # use the steady part: alt climbed and roughly held (drop first 4 s)
    t = d["t"]; m = t > t[0] + 5.0
    thr = d["thro"][m]
    M = np.stack([d["m0"][m], d["m1"][m], d["m2"][m], d["m3"][m]], axis=1)
    spread = (M.max(1) - M.min(1))
    psat = np.mean(np.abs(d["pitch_out"][m]) >= 0.99)
    floor = np.mean(M.min(1) <= 0.06)
    return dict(thr=thr.mean(), thr_sd=thr.std(), spread=spread.mean(),
                psat=100 * psat, floor=100 * floor,
                prate_rms=d["prate"][m].std(), alt=d["alt_up"][m].mean(),
                n=m.sum())


print("=" * 70)
print("SIM PLANT-ID (fresh headless run, GCS conf geometry)")
for nm, axis in [("chirp_roll.csv", "roll"), ("chirp_pitch.csv", "pitch")]:
    K, frb, Kf, fs = fit_K(nm, axis)
    rk = REAL["K_roll"] if axis == "roll" else REAL["K_pitch"]
    print(f"  {axis:5s}: K_sim={abs(K):7.1f}  (band {frb[0]:.1f}-{frb[-1]:.1f}Hz, "
          f"spread {Kf.std()/abs(np.mean(Kf)):.0%})  fs={fs:.0f}Hz  | "
          f"real K={rk:.0f}  ->  sim/real={abs(K)/rk:.2f}x")
print()
h = hover_stats()
print("SIM HOVER (fresh free-flight run)")
print(f"  hover throttle {h['thr']:.3f} (sd {h['thr_sd']:.3f})  alt {h['alt']:.1f}m  "
      f"n={h['n']}")
print(f"  motor spread {h['spread']:.3f}   pitch_out sat {h['psat']:.0f}%   "
      f"floor {h['floor']:.0f}%   pitch-rate RMS {h['prate_rms']:.1f} dps")
print(f"  REAL active windows: spread {REAL['mspread']:.2f}  psat "
      f"{100*REAL['psat']:.0f}%  hover thr ~{REAL['hover_thr']:.2f}")
