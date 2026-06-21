#!/usr/bin/env python3
"""Plots for the 2026-06-21 05:31 SITL run — aggressive free flight with the
lowered accel trust (EKF_R_ACC_DIR=2.5e-2). Dual-log: FC estimate vs physics
ground truth. Reuses ../parse_log.py and ../parse_gt.py. PNGs into ./plots/.

    python3 docs/journal/log-analysis/20260621-053142/make_plots.py
"""
import importlib.util
import math
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
LA = os.path.abspath(os.path.join(HERE, ".."))
def _load(n):
    s = importlib.util.spec_from_file_location(n, os.path.join(LA, n + ".py"))
    m = importlib.util.module_from_spec(s); s.loader.exec_module(m); return m
pl = _load("parse_log"); pg = _load("parse_gt")
plt.rcParams.update({"figure.dpi": 110, "font.size": 9, "axes.grid": True,
                     "grid.alpha": 0.3, "figure.autolayout": True})
OUT = os.path.join(HERE, "plots"); os.makedirs(OUT, exist_ok=True)
FC = os.path.join(HERE, "export-20260621-053142.bin")
GT = os.path.join(HERE, "gt-2026-06-21_05-31-38.bin")
EST, TRU = "tab:red", "tab:green"
OFF = 0.04  # motor-xcorr alignment lag (corr 1.000)

hdr, recs = pl.read_records(FC); col = pl.Collector().run(recs)
ct = col.msgs["ControlTrace"]; vs = col.msgs["VerticalState"]
def A(m, f): return np.array([getattr(x, f) for _, x, _ in m], float)
W = hdr["start_wall_clock_ms"] / 1000.0; t0 = np.array([t for t, _, _ in ct], float)[0] / 1e6
fc_abs = (np.array([t for t, _, _ in ct], float) / 1e6 - t0) + W + OFF
fcr, fcp, fcy = A(ct, "roll_angle_curr"), A(ct, "pitch_angle_curr"), A(ct, "yaw_angle_curr")
rsp, psp = A(ct, "roll_angle_sp"), A(ct, "pitch_angle_sp")
vt_abs = (np.array([t for t, _, _ in vs], float) / 1e6 - t0) + W + OFF; fca = A(vs, "altitude")
g = pg.read(GT); idx = {f: i for i, f in enumerate(pg.FIELDS)}
G = lambda f: np.array([r[idx[f]] for r in g[1]], float)
gt_abs = G("t_us") / 1e6 + g[0]["start_wall_ms"] / 1000.0
gtr, gtp, gty, gta = G("roll"), G("pitch"), G("yaw"), -G("pos_d")
hsp = np.sqrt(G("vel_n") ** 2 + G("vel_e") ** 2)
def wrap(d): return (d + 180) % 360 - 180
lo = max(fc_abs[0], gt_abs[0]); hi = min(fc_abs[-1], gt_abs[-1]); gr = np.arange(lo, hi, 0.05); tr = gr - gr[0]

# Fig 1 — estimate vs truth, all four states
fig, ax = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
for a, (nm, fv, gv) in zip(ax[:3], [("roll (deg)", fcr, gtr), ("pitch (deg)", fcp, gtp), ("yaw (deg)", fcy, gty)]):
    a.plot(gt_abs - gr[0], gv, color=TRU, lw=1.0, label="truth")
    a.plot(fc_abs - gr[0], fv, color=EST, lw=0.9, alpha=0.85, label="FC estimate")
    a.set_ylabel(nm); a.legend(loc="upper right", fontsize=7)
ax[3].plot(gt_abs - gr[0], gta, color=TRU, lw=1.0, label="truth alt")
ax[3].plot(vt_abs - gr[0], fca, color=EST, lw=0.9, alpha=0.85, label="FC fused alt")
ax[3].set_ylabel("alt (m)"); ax[3].legend(loc="upper left", fontsize=7); ax[3].set_xlabel("t (s)")
fig.suptitle("FC estimate (red) vs truth (green) — 2026-06-21 05:31, aggressive flight, EKF_R_ACC_DIR=2.5e-2")
fig.savefig(os.path.join(OUT, "01_estimate_vs_truth.png")); plt.close(fig)

# Fig 2 — roll/pitch tilt now tracks; error tracks acceleration, not speed
RC, PC = np.interp(gr, fc_abs, fcr), np.interp(gr, fc_abs, fcp)
TR, TP = np.interp(gr, gt_abs, gtr), np.interp(gr, gt_abs, gtp)
RSP, PSP = np.interp(gr, fc_abs, rsp), np.interp(gr, fc_abs, psp)
HS = np.interp(gr, gt_abs, hsp)
est_tilt, truth_tilt = np.sqrt(RC**2 + PC**2), np.sqrt(TR**2 + TP**2)
cen = (np.abs(RSP) < 3) & (np.abs(PSP) < 3)
fig, ax = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
ax[0].plot(tr, truth_tilt, color=TRU, lw=1.0, label="truth tilt")
ax[0].plot(tr, est_tilt, color=EST, lw=0.9, alpha=0.85, label="FC estimated tilt")
ax[0].fill_between(tr, 0, 80, where=cen, color="gray", alpha=0.07, label="sticks centered")
ax[0].set_ylabel("tilt magnitude (deg)"); ax[0].legend(loc="upper right", fontsize=7)
ax[0].set_title("Roll/pitch tilt: estimate now follows truth (was flat-near-0 at the old accel trust)")
ax[1].plot(tr, HS, color="tab:purple", lw=0.9); ax[1].set_ylabel("horiz speed (m/s)"); ax[1].set_xlabel("t (s)")
ax[1].set_title("Residual error tracks acceleration (speed edges), not constant speed")
fig.savefig(os.path.join(OUT, "02_rollpitch_now_tracks.png")); plt.close(fig)

# Fig 3 — per-axis error + the accel-trust before/after (stick-centered under-report)
gb_e = np.stack([-np.sin(np.radians(PC)), np.sin(np.radians(RC)) * np.cos(np.radians(PC)),
                 np.cos(np.radians(RC)) * np.cos(np.radians(PC))], 1)
qw, qx, qy, qz = [np.interp(gr, gt_abs, G(k)) for k in ("qw", "qx", "qy", "qz")]
n = np.sqrt(qw**2 + qx**2 + qy**2 + qz**2); qw, qx, qy, qz = qw/n, qx/n, qy/n, qz/n
gb_t = np.stack([2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx*qx + qy*qy)], 1)
gb_e /= np.linalg.norm(gb_e, 1, keepdims=True) if False else np.linalg.norm(gb_e, axis=1, keepdims=True)
gb_t /= np.linalg.norm(gb_t, axis=1, keepdims=True)
tilt_err = np.degrees(np.arccos(np.clip((gb_e * gb_t).sum(1), -1, 1)))
ye = wrap(np.interp(gr, fc_abs, fcy) - np.interp(gr, gt_abs, gty))
ae = np.interp(gr, vt_abs, fca) - np.interp(gr, gt_abs, gta)
fig, ax = plt.subplots(1, 2, figsize=(11, 3.8))
ax[0].bar(["yaw", "roll/pitch tilt", "alt (×10 m)"],
          [np.sqrt((ye**2).mean()), np.sqrt((tilt_err**2).mean()), np.sqrt((ae**2).mean()) * 10],
          color=["tab:green", "tab:orange", "tab:blue"])
ax[0].set_ylabel("est-vs-truth RMS"); ax[0].set_title("This run — error by axis (deg / ×10 m)")
# stick-centered tilt under-report: 021352 (R=2.5e-3) vs this (R=2.5e-2)
ax[1].bar(["R=2.5e-3\n(021352)", "R=2.5e-2\n(this)"], [2.15, 0.15], color=["tab:red", "tab:green"])
ax[1].set_ylabel("stick-centered tilt under-report (deg)")
ax[1].set_title("Accel-trust change: true tilt the FC misses when 'level'")
fig.savefig(os.path.join(OUT, "03_error_summary.png")); plt.close(fig)
print(f"wrote 3 figures to {OUT}/")
