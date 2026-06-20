#!/usr/bin/env python3
"""Session-specific analysis plots for the 2026-06-21 SITL autotune-validation triplet.

This is NOT the generic make_plots.py (that one is bespoke to the 2026-06-17
hardware session). These three back-to-back SITL flights share a headline of
their own — three aggressive manual ANGLE-mode flights whose ONLY material
difference is the roll/pitch rate gain: two tumble (weak gain), one flies clean
(~4x roll gain) — so they get their own comparison figures.

Reuses the decoder in ../parse_log.py. Writes PNGs into ./plots/.

Usage (from repo root):
    python3 docs/journal/log-analysis/20260621-001850/make_plots.py
"""
import importlib.util
import math
import os
import statistics as st

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
LA = os.path.abspath(os.path.join(HERE, ".."))          # docs/journal/log-analysis
spec = importlib.util.spec_from_file_location("pl", os.path.join(LA, "parse_log.py"))
pl = importlib.util.module_from_spec(spec); spec.loader.exec_module(pl)

plt.rcParams.update({"figure.dpi": 110, "font.size": 9, "axes.grid": True,
                     "grid.alpha": 0.3, "figure.autolayout": True})

SESSIONS = ["001607", "001716", "001850"]
LABELS = {"001607": "001607 — tumble", "001716": "001716 — clean",
          "001850": "001850 — late tumble"}
OUT = os.path.join(HERE, "plots")
os.makedirs(OUT, exist_ok=True)


def decode(sess):
    binp = os.path.join(HERE, f"export-20260621-{sess}.bin")
    _, recs = pl.read_records(binp)
    col = pl.Collector().run(recs)
    return col


def series(col, msg, field):
    lst = col.msgs.get(msg, [])
    t = np.array([t for (t, _, _) in lst], float)
    v = np.array([getattr(m, field) for (_, m, _) in lst], float)
    if len(t):
        t = (t - t[0]) / 1e6
    return t, v


def corr(a, b):
    n = min(len(a), len(b)); a, b = a[:n], b[:n]
    if n < 2:
        return 0.0
    return float(np.corrcoef(a, b)[0, 1])


COLS = {sess: decode(sess) for sess in SESSIONS}

# ---------------------------------------------------------------------------
# Fig 1 — attitude timeline per run: where roll loses it
# ---------------------------------------------------------------------------
fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=False)
for ax, sess in zip(axs, SESSIONS):
    c = COLS[sess]
    t, roll = series(c, "ControlTrace", "roll_angle_curr")
    _, pitch = series(c, "ControlTrace", "pitch_angle_curr")
    _, yaw = series(c, "ControlTrace", "yaw_angle_curr")
    ax.axhspan(90, 200, color="red", alpha=0.06)
    ax.axhspan(-200, -90, color="red", alpha=0.06)
    ax.axhline(90, color="red", lw=0.6, ls="--"); ax.axhline(-90, color="red", lw=0.6, ls="--")
    ax.plot(t, roll, label="roll", lw=0.9)
    ax.plot(t, pitch, label="pitch", lw=0.9)
    ax.plot(t, yaw, label="yaw", lw=0.6, alpha=0.6)
    ax.set_ylabel("angle (deg)"); ax.set_title(LABELS[sess]); ax.set_ylim(-200, 200)
    ax.legend(loc="upper right", ncol=3, fontsize=7)
axs[-1].set_xlabel("t (s, per-run)")
fig.suptitle("Measured attitude per run — shaded = past 90° (tumbled)")
fig.savefig(os.path.join(OUT, "01_attitude_timeline.png")); plt.close(fig)

# ---------------------------------------------------------------------------
# Fig 2 — rate-loop tracking scatter: sp vs curr, axes x sessions
# ---------------------------------------------------------------------------
fig, axs = plt.subplots(3, 3, figsize=(10, 9))
for j, sess in enumerate(SESSIONS):
    c = COLS[sess]
    for i, axn in enumerate(("roll", "pitch", "yaw")):
        _, sp = series(c, "ControlTrace", f"{axn}_rate_sp")
        _, cu = series(c, "ControlTrace", f"{axn}_rate_curr")
        a = axs[i][j]
        a.scatter(sp, cu, s=2, alpha=0.25)
        lim = 250
        a.plot([-lim, lim], [-lim, lim], "r-", lw=0.7)
        a.set_xlim(-lim, lim); a.set_ylim(-lim, lim)
        a.set_title(f"{axn}  corr={corr(sp, cu):+.2f}", fontsize=8)
        if j == 0:
            a.set_ylabel("rate_curr (°/s)")
        if i == 2:
            a.set_xlabel("rate_sp (°/s)")
    axs[0][j].annotate(LABELS[sess], xy=(0.5, 1.25), xycoords="axes fraction",
                       ha="center", fontsize=9, weight="bold")
fig.suptitle("Rate-loop tracking — diagonal = perfect; yaw rides it, roll/pitch don't")
fig.savefig(os.path.join(OUT, "02_rate_tracking.png")); plt.close(fig)

# ---------------------------------------------------------------------------
# Fig 3 — the differentiator: roll gain vs outcome across runs
# ---------------------------------------------------------------------------
def kp_eff(c, axn):
    _, sp = series(c, "ControlTrace", f"{axn}_rate_sp")
    _, cu = series(c, "ControlTrace", f"{axn}_rate_curr")
    _, out = series(c, "ControlTrace", f"{axn}_out")
    err = sp - cu
    n = min(len(err), len(out)); err, out = err[:n], out[:n]
    mx, my = err.mean(), out.mean()
    sxx = ((err - mx) ** 2).sum()
    return float(((err - mx) * (out - my)).sum() / sxx) if sxx > 1e-9 else 0.0


roll_kp = [kp_eff(COLS[s], "roll") for s in SESSIONS]
roll_corr = [corr(*series(COLS[s], "ControlTrace", "roll_rate_sp")[1:] +
                  (series(COLS[s], "ControlTrace", "roll_rate_curr")[1],))
             if False else
             corr(series(COLS[s], "ControlTrace", "roll_rate_sp")[1],
                  series(COLS[s], "ControlTrace", "roll_rate_curr")[1]) for s in SESSIONS]
def frac_inverted(s):
    _, roll = series(COLS[s], "ControlTrace", "roll_angle_curr")
    return 100.0 * np.mean(np.abs(roll) > 90)
inv = [frac_inverted(s) for s in SESSIONS]

fig, axs = plt.subplots(1, 3, figsize=(11, 3.6))
x = np.arange(3)
axs[0].bar(x, roll_kp, color="tab:blue"); axs[0].set_title("roll rate Kp_eff (out/err slope)")
axs[0].set_yscale("log")
axs[1].bar(x, roll_corr, color="tab:green"); axs[1].set_title("roll rate corr(sp,curr)")
axs[1].set_ylim(0, 1)
axs[2].bar(x, inv, color="tab:red"); axs[2].set_title("% of flight inverted (|roll|>90°)")
for a in axs:
    a.set_xticks(x); a.set_xticklabels(SESSIONS, rotation=0, fontsize=8)
fig.suptitle("The differentiator — 001716's ~4× roll gain tracks and never tumbles")
fig.savefig(os.path.join(OUT, "03_gain_vs_outcome.png")); plt.close(fig)

# ---------------------------------------------------------------------------
# Fig 4 — authority: out vs rate_err (roll/pitch hug zero, yaw fans out)
# ---------------------------------------------------------------------------
fig, axs = plt.subplots(1, 3, figsize=(11, 3.8))
for a, sess in zip(axs, SESSIONS):
    c = COLS[sess]
    for axn, col in (("roll", "tab:blue"), ("pitch", "tab:orange"), ("yaw", "tab:green")):
        _, sp = series(c, "ControlTrace", f"{axn}_rate_sp")
        _, cu = series(c, "ControlTrace", f"{axn}_rate_curr")
        _, out = series(c, "ControlTrace", f"{axn}_out")
        a.scatter(sp - cu, out, s=2, alpha=0.2, color=col, label=axn)
    a.axhline(1, color="k", lw=0.5, ls=":"); a.axhline(-1, color="k", lw=0.5, ls=":")
    a.set_ylim(-0.6, 0.6); a.set_xlabel("rate error (°/s)"); a.set_title(LABELS[sess], fontsize=8)
axs[0].set_ylabel("controller out [-1,1]"); axs[0].legend(fontsize=7)
fig.suptitle("Rate-loop authority — roll/pitch never leave ±0.12, yaw uses the range")
fig.savefig(os.path.join(OUT, "04_rate_authority.png")); plt.close(fig)

# ---------------------------------------------------------------------------
# Fig 5 — vertical estimator: fused vs baro + residual
# ---------------------------------------------------------------------------
fig, axs = plt.subplots(2, 3, figsize=(11, 6), sharex=False)
for j, sess in enumerate(SESSIONS):
    c = COLS[sess]
    t, alt = series(c, "VerticalState", "altitude")
    _, baro = series(c, "VerticalState", "baro_altitude")
    n = min(len(alt), len(baro))
    axs[0][j].plot(t[:n], alt[:n], label="fused", lw=0.8)
    axs[0][j].plot(t[:n], baro[:n], label="baro", lw=0.6, alpha=0.7)
    axs[0][j].set_title(LABELS[sess], fontsize=8); axs[0][j].legend(fontsize=7)
    axs[1][j].plot(t[:n], alt[:n] - baro[:n], color="tab:red", lw=0.6)
    axs[1][j].set_xlabel("t (s)")
    rms = math.sqrt(np.mean((alt[:n] - baro[:n]) ** 2))
    axs[1][j].set_title(f"fused − baro (RMS {rms:.2f} m)", fontsize=8)
axs[0][0].set_ylabel("altitude (m)"); axs[1][0].set_ylabel("residual (m)")
fig.suptitle("Vertical estimator — fused tracks baro across all three runs")
fig.savefig(os.path.join(OUT, "05_vertical_estimator.png")); plt.close(fig)

print(f"wrote 5 figures to {OUT}/")
