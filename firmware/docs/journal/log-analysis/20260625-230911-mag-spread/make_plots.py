#!/usr/bin/env python3
"""Figures for the magnetometer-spread archive.

Reads the 50 Hz mag capture in this dir (reconstructed live from the FC over UDP:
ImuRaw keyframes + ImuCompressed f16 deltas, by tools/telemetry/udp_telem_sniff.py
--mag-csv) and writes the spread + deviation PNGs into ./plots/. Bespoke to this
archive.

    python3 docs/journal/log-analysis/20260625-mag-spread/make_plots.py
"""
import csv, math, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "plots"); os.makedirs(OUT, exist_ok=True)
plt.rcParams.update({"figure.dpi": 120, "font.size": 9, "axes.grid": True,
                     "grid.alpha": 0.3})


def load(name):
    cols = {}
    with open(os.path.join(HERE, name)) as f:
        r = csv.DictReader(f)
        for k in r.fieldnames:
            cols[k] = []
        for row in r:
            for k in r.fieldnames:
                try:
                    cols[k].append(float(row[k]))
                except ValueError:
                    pass
    return {k: np.array(v) for k, v in cols.items() if v}


m = load("mag_rotation_50hz.csv")
P = np.array([m["mx"], m["my"], m["mz"]]).T          # N x 3 (µT)
t = m["t"]
N = len(P)
mag = np.linalg.norm(P, axis=1)

# Two "centres": the raw sample MEAN (what the user asked to mark) and the
# least-squares SPHERE centre (the coverage-independent hard-iron estimate —
# the trustworthy one; mean drifts under non-uniform hand coverage).
mean_pt = P.mean(axis=0)
A = np.column_stack([2 * P[:, 0], 2 * P[:, 1], 2 * P[:, 2], np.ones(N)])
sol, *_ = np.linalg.lstsq(A, (P ** 2).sum(axis=1), rcond=None)
c_ls = sol[:3]
r_ls = math.sqrt(max(sol[3] + c_ls @ c_ls, 0.0))
ls_off = float(np.linalg.norm(c_ls))                 # hard-iron residual (µT)

mag_mean = mag.mean()
cov = 100.0 * mag.std() / mag_mean                   # coefficient of variation %
pct_off = 100.0 * (mag - mag_mean) / mag_mean        # per-sample % off the mean ‖m‖

# ----------------------------------------------------------- 01: plane spread
lim = float(np.abs(P - mean_pt).max()) * 1.15
fig, axs = plt.subplots(1, 3, figsize=(15, 5.4))


def scat(a, i, j, lu, lv):
    sc = a.scatter(P[:, i], P[:, j], s=9, c=t, cmap="viridis", alpha=0.6)
    a.plot(mean_pt[i], mean_pt[j], "b+", ms=16, mew=2.5,
           label=f"mean ({mean_pt[i]:+.1f},{mean_pt[j]:+.1f})")
    a.plot(c_ls[i], c_ls[j], "rx", ms=14, mew=2.5,
           label=f"LS centre ({c_ls[i]:+.1f},{c_ls[j]:+.1f})")
    th = np.linspace(0, 2 * math.pi, 200)
    a.plot(c_ls[i] + r_ls * np.cos(th), c_ls[j] + r_ls * np.sin(th),
           "r--", lw=1.1, alpha=0.8, label=f"fit r={r_ls:.0f}")
    a.set_xlim(mean_pt[i] - lim, mean_pt[i] + lim)
    a.set_ylim(mean_pt[j] - lim, mean_pt[j] + lim)
    a.set_aspect("equal")
    a.set_xlabel(lu + " (µT)", fontsize=11); a.set_ylabel(lv + " (µT)", fontsize=11)
    a.set_title(f"{lu}–{lv} plane", fontsize=12)
    a.legend(fontsize=8, loc="upper right")
    return sc


scat(axs[0], 0, 1, "mX", "mY")
scat(axs[1], 1, 2, "mY", "mZ")
sc = scat(axs[2], 0, 2, "mX", "mZ")
fig.suptitle(f"Magnetometer spread — {N} pts @50 Hz   "
             f"(mean centre + LS-sphere centre; LS offset {ls_off:.1f} µT = "
             f"hard-iron residual, centred = good)",
             fontweight="bold", fontsize=13)
fig.colorbar(sc, ax=axs, fraction=0.015, pad=0.01, label="capture time (s)")
fig.savefig(os.path.join(OUT, "01_mag_planes.png"),
            bbox_inches="tight"); plt.close(fig)

# ------------------------------------------------ 02: ‖m‖ + % off-from-mean
fig, (a0, a1, a2) = plt.subplots(1, 3, figsize=(15, 4),
                                 gridspec_kw={"width_ratios": [2, 2, 1]})
a0.plot(t, mag, lw=0.7, color="#2ca02c", label="‖m‖")
a0.axhline(mag_mean, color="#1f77b4", lw=1.2, label=f"mean {mag_mean:.1f} µT")
a0.fill_between(t, mag_mean * 0.95, mag_mean * 1.05, color="k", alpha=0.07,
                label="±5 %")
a0.set_title(f"‖m‖ over rotation  (CoV {cov:.1f} %)", fontweight="bold")
a0.set_xlabel("s"); a0.set_ylabel("µT"); a0.legend(fontsize=8)

a1.plot(t, pct_off, lw=0.7, color="#d62728")
a1.axhline(0, color="#1f77b4", lw=1.0)
for b in (5, -5):
    a1.axhline(b, color="k", lw=0.8, ls="--", alpha=0.5)
a1.fill_between(t, -cov, cov, color="#d62728", alpha=0.08, label=f"±CoV ({cov:.1f} %)")
a1.set_title("% off the mean ‖m‖ (per sample)", fontweight="bold")
a1.set_xlabel("s"); a1.set_ylabel("% off mean"); a1.legend(fontsize=8)

a2.hist(pct_off, bins=40, orientation="horizontal", color="#d62728",
        alpha=0.85, edgecolor="k", lw=0.3)
a2.axhline(0, color="#1f77b4", lw=1.0)
a2.set_title("distribution", fontweight="bold")
a2.set_xlabel("count"); a2.set_ylabel("% off mean")
fig.suptitle("Magnetometer magnitude consistency — flat / tight = well-centred, "
             "calibrated sphere", fontsize=12)
fig.tight_layout(rect=[0, 0, 1, 0.94])
fig.savefig(os.path.join(OUT, "02_mag_pct_off_mean.png")); plt.close(fig)

print(f"N={N}  |m| mean={mag_mean:.2f} uT  CoV={cov:.2f}%  "
      f"max|%off|={np.abs(pct_off).max():.2f}%")
print(f"mean centre=({mean_pt[0]:+.2f},{mean_pt[1]:+.2f},{mean_pt[2]:+.2f})  "
      f"LS centre=({c_ls[0]:+.2f},{c_ls[1]:+.2f},{c_ls[2]:+.2f})  "
      f"LS offset={ls_off:.2f} uT  r={r_ls:.2f}")
print("wrote plots/01_mag_planes.png plots/02_mag_pct_off_mean.png")
