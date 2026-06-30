#!/usr/bin/env python3
"""Generate the figures for the IMU-calibration-verification archive.

Reads the two persisted 50 Hz CSV captures in this dir (reconstructed from the
live FC over UDP — ImuRaw keyframes + ImuCompressed deltas) and writes focused
PNGs into ./plots/. Bespoke to this archive.

Usage (from repo root):
    python3 docs/journal/log-analysis/20260625-032526-imu-calib-verify/make_plots.py
"""
import csv, math, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "plots"); os.makedirs(OUT, exist_ok=True)
G = 9.80665
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

# ---------------------------------------------------------------- accel + gyro
s = load("imu_static_accel_gyro_50hz.csv")
t = s["t"]
amag = np.sqrt(s["ax"]**2 + s["ay"]**2 + s["az"]**2)

fig, (a0, a1) = plt.subplots(1, 2, figsize=(12, 4.2))
a0.plot(t, amag, lw=0.7, color="#2ca02c")
a0.axhline(G, color="k", ls="--", lw=1, label=f"g = {G:.3f}")
a0.axhline(10.215, color="#d62728", ls=":", lw=1.2, label="pre-cal 10.215 (+4.2%)")
a0.fill_between(t, G*0.99, G*1.01, color="k", alpha=0.07, label="±1%")
a0.set_title(f"‖a‖ static: mean {amag.mean():.4f}  ({100*(amag.mean()-G)/G:+.2f}% vs g)  σ={amag.std():.4f}")
a0.set_xlabel("s"); a0.set_ylabel("m/s²"); a0.legend(fontsize=8); a0.set_ylim(9.6, 10.35)
a1.hist(amag, bins=40, color="#2ca02c", alpha=0.85, edgecolor="k", lw=0.3)
a1.axvline(G, color="k", ls="--", lw=1.2, label=f"g={G:.3f}")
a1.axvline(amag.mean(), color="#1f77b4", lw=1.2, label=f"mean={amag.mean():.4f}")
a1.set_title("‖a‖ distribution (noise floor)"); a1.set_xlabel("m/s²"); a1.legend(fontsize=8)
fig.suptitle("Accelerometer — post pose-tolerant full-3×3 calibration (static, 50 Hz)",
             fontweight="bold")
fig.tight_layout(rect=[0, 0, 1, 0.95]); fig.savefig(os.path.join(OUT, "01_accel_magnitude.png")); plt.close(fig)

fig, (a0, a1) = plt.subplots(1, 2, figsize=(12, 4.2))
for k, c in (("gx", "#d62728"), ("gy", "#e6b800"), ("gz", "#17becf")):
    a0.plot(t, s[k], lw=0.7, color=c, label=f"{k} (μ{s[k].mean():+.3f})")
a0.axhline(0, color="k", ls="--", lw=0.8)
a0.set_title("Gyro axes static (dps) — residual bias all <0.04 dps")
a0.set_xlabel("s"); a0.set_ylabel("deg/s"); a0.legend(fontsize=8)
# before/after bias bars
axes = ["gx", "gy", "gz"]
before = [-0.049, 0.430, -0.473]  # measured pre-gyro-cal (session start)
after = [s[k].mean() for k in axes]
x = np.arange(3); w = 0.38
a1.bar(x - w/2, before, w, color="#d62728", alpha=0.8, label="before cal")
a1.bar(x + w/2, after, w, color="#2ca02c", alpha=0.9, label="after cal")
a1.axhline(0, color="k", lw=0.8); a1.set_xticks(x); a1.set_xticklabels(["gX", "gY", "gZ"])
a1.set_title("Gyro bias: before vs after"); a1.set_ylabel("deg/s"); a1.legend(fontsize=8)
fig.suptitle("Gyroscope — stillness-gated bias calibration (static, 50 Hz)", fontweight="bold")
fig.tight_layout(rect=[0, 0, 1, 0.95]); fig.savefig(os.path.join(OUT, "02_gyro_bias.png")); plt.close(fig)

# ---------------------------------------------------------------- mag sphere
m = load("mag_rotation_50hz.csv")
P = np.array([m["mx"], m["my"], m["mz"]]).T
mt = m["t"]
mmag = np.linalg.norm(P, axis=1)
n = len(P)
A = np.column_stack([2*P[:, 0], 2*P[:, 1], 2*P[:, 2], np.ones(n)])
sol, *_ = np.linalg.lstsq(A, (P**2).sum(axis=1), rcond=None)
c = sol[:3]; r = math.sqrt(max(sol[3] + c @ c, 0.0)); off = float(np.linalg.norm(c))

# Projections as a 1x3 row (each square, large markers/fonts so they survive a
# scale-to-page-width in the PDF), and |m| as its own figure — keeps each panel
# legible in print instead of a cramped 2x2.
lim = float(np.abs(P).max()) * 1.1
fig, axs = plt.subplots(1, 3, figsize=(15, 5.4))
def scat(a, i, j, lu, lv):
    a.scatter(P[:, i], P[:, j], s=9, c=mt, cmap="viridis", alpha=0.6)
    a.axhline(0, color="k", lw=0.7); a.axvline(0, color="k", lw=0.7)
    a.plot(c[i], c[j], "rx", ms=14, mew=2.5, label=f"LS centre\n({c[i]:+.1f},{c[j]:+.1f})")
    th = np.linspace(0, 2*math.pi, 200)
    a.plot(c[i]+r*np.cos(th), c[j]+r*np.sin(th), "r--", lw=1.1, alpha=0.8, label=f"fit r={r:.0f}")
    a.set_xlim(-lim, lim); a.set_ylim(-lim, lim); a.set_aspect("equal")
    a.set_xlabel(lu+" (µT)", fontsize=11); a.set_ylabel(lv+" (µT)", fontsize=11)
    a.set_title(f"{lu}–{lv} plane", fontsize=12); a.legend(fontsize=9, loc="upper right")
scat(axs[0], 0, 1, "mX", "mY"); scat(axs[1], 0, 2, "mX", "mZ"); scat(axs[2], 1, 2, "mY", "mZ")
fig.suptitle(f"Magnetometer sphere test — calibrated, {n} pts @50 Hz   "
             f"(LS-centre offset {off:.1f} µT = hard-iron residual; centred = good)",
             fontweight="bold", fontsize=13)
fig.tight_layout(rect=[0, 0, 1, 0.94]); fig.savefig(os.path.join(OUT, "03_mag_sphere.png")); plt.close(fig)

fig, a = plt.subplots(1, 1, figsize=(11, 3.8))
a.plot(mt, mmag, lw=0.7, color="#2ca02c", label="‖m‖")
a.axhline(mmag.mean(), color="#1f77b4", lw=1.2, label=f"mean {mmag.mean():.1f} µT")
a.fill_between(mt, mmag.mean()*0.95, mmag.mean()*1.05, color="k", alpha=0.07, label="±5%")
a.set_title(f"‖m‖ over rotation — flat = centred sphere (CoV {100*mmag.std()/mmag.mean():.1f}%, "
            f"was ~25% uncalibrated)", fontweight="bold")
a.set_xlabel("s"); a.set_ylabel("µT"); a.legend(fontsize=9)
fig.tight_layout(); fig.savefig(os.path.join(OUT, "04_mag_norm.png")); plt.close(fig)

print(f"accel |a| {amag.mean():.4f} ({100*(amag.mean()-G)/G:+.2f}%) sigma {amag.std():.4f}")
print(f"gyro bias {[round(x,3) for x in after]} dps")
print(f"mag CoV {100*mmag.std()/mmag.mean():.1f}% LS-centre-offset {off:.2f}uT r {r:.1f}")
print("wrote plots/01_accel_magnitude.png 02_gyro_bias.png 03_mag_sphere.png 04_mag_norm.png")
