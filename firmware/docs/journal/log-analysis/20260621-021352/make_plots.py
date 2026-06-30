#!/usr/bin/env python3
"""Session-specific plots for the 2026-06-21 02:13 SITL run — the FIRST archive
with BOTH the FC telemetry (estimate) and the physics ground truth, so the
figures are estimate-vs-truth overlays, not single-stream traces.

NOT the generic make_plots.py. Reuses the decoders in ../parse_log.py (FC VREC)
and ../parse_gt.py (physics gt-*.bin). Writes PNGs into ./plots/.

    python3 docs/journal/log-analysis/20260621-021352/make_plots.py
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
def _load(name):
    spec = importlib.util.spec_from_file_location(name, os.path.join(LA, name + ".py"))
    m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m); return m
pl = _load("parse_log"); pg = _load("parse_gt")

plt.rcParams.update({"figure.dpi": 110, "font.size": 9, "axes.grid": True,
                     "grid.alpha": 0.3, "figure.autolayout": True})
OUT = os.path.join(HERE, "plots"); os.makedirs(OUT, exist_ok=True)
FC = os.path.join(HERE, "export-20260621-021352.bin")
GT = os.path.join(HERE, "gt-2026-06-21_02-13-48.bin")
EST, TRU = "tab:red", "tab:green"

# ---- decode FC (estimate) ----
hdr, recs = pl.read_records(FC); col = pl.Collector().run(recs)
ct = col.msgs["ControlTrace"]; mt = col.msgs["MotorTelemetry"]; vs = col.msgs["VerticalState"]
def A(m, f): return np.array([getattr(x, f) for _, x, _ in m], float)
W = hdr["start_wall_clock_ms"] / 1000.0
t0 = np.array([t for t, _, _ in ct], float)[0] / 1e6
OFF = 0.04  # motor-xcorr alignment lag (corr 1.000)
fc_abs = (np.array([t for t, _, _ in ct], float) / 1e6 - t0) + W + OFF
fc_roll, fc_pitch, fc_yaw = A(ct, "roll_angle_curr"), A(ct, "pitch_angle_curr"), A(ct, "yaw_angle_curr")
rsp, psp = A(ct, "roll_angle_sp"), A(ct, "pitch_angle_sp")
mt_abs = (np.array([t for t, _, _ in mt], float) / 1e6 - t0) + W + OFF
mt_d = np.array([np.mean(m.cmd[:4]) for _, m, _ in mt])
vt_abs = (np.array([t for t, _, _ in vs], float) / 1e6 - t0) + W + OFF
fc_alt = A(vs, "altitude")

# ---- decode GT (truth) ----
g = pg.read(GT); idx = {f: i for i, f in enumerate(pg.FIELDS)}
G = lambda f: np.array([r[idx[f]] for r in g[1]], float)
gt_abs = G("t_us") / 1e6 + g[0]["start_wall_ms"] / 1000.0
gt_roll, gt_pitch, gt_yaw = G("roll"), G("pitch"), G("yaw")
gt_alt = -G("pos_d")
hspd = np.sqrt(G("vel_n") ** 2 + G("vel_e") ** 2)
def wrap(d): return (d + 180) % 360 - 180

# common grid
lo = max(fc_abs[0], gt_abs[0]); hi = min(fc_abs[-1], gt_abs[-1])
gr = np.arange(lo, hi, 0.05); tr = gr - gr[0]

# =====================================================================
# Fig 1 — estimate vs truth, all four states
# =====================================================================
fig, ax = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
for a, (nm, fa, fv, ga, gv) in zip(ax[:3], [
        ("roll (deg)", fc_abs, fc_roll, gt_abs, gt_roll),
        ("pitch (deg)", fc_abs, fc_pitch, gt_abs, gt_pitch),
        ("yaw (deg)", fc_abs, fc_yaw, gt_abs, gt_yaw)]):
    a.plot(ga - gr[0], gv, color=TRU, lw=1.0, label="truth (physics)")
    a.plot(fa - gr[0], fv, color=EST, lw=0.9, alpha=0.85, label="FC estimate")
    a.set_ylabel(nm); a.legend(loc="upper right", fontsize=7)
ax[3].plot(gt_abs - gr[0], gt_alt, color=TRU, lw=1.0, label="truth alt")
ax[3].plot(vt_abs - gr[0], fc_alt, color=EST, lw=0.9, alpha=0.85, label="FC fused alt")
ax[3].set_ylabel("alt (m)"); ax[3].legend(loc="upper left", fontsize=7); ax[3].set_xlabel("t (s)")
fig.suptitle("FC estimate (red) vs physics ground truth (green) — 2026-06-21 02:13 (mag_fusion fixed)")
fig.savefig(os.path.join(OUT, "01_estimate_vs_truth.png")); plt.close(fig)

# =====================================================================
# Fig 2 — yaw fix: estimate now tracks the full true heading swing
# =====================================================================
fig, a = plt.subplots(1, 1, figsize=(11, 4))
a.plot(gt_abs - gr[0], gt_yaw, color=TRU, lw=1.4, label="truth heading")
a.plot(fc_abs - gr[0], fc_yaw, color=EST, lw=1.0, label="FC yaw estimate")
ye = wrap(np.interp(gr, fc_abs, fc_yaw) - np.interp(gr, gt_abs, gt_yaw))
a.set_xlabel("t (s)"); a.set_ylabel("yaw (deg)")
a.set_title(f"Yaw FIXED — estimate tracks the true −134° swing  (RMS {np.sqrt((ye**2).mean()):.1f}°; "
            f"was 121° before the mag_fusion fix)")
a.legend(loc="lower left", fontsize=8)
fig.savefig(os.path.join(OUT, "02_yaw_fixed.png")); plt.close(fig)

# =====================================================================
# Fig 3 — roll/pitch: estimate levels but truth doesn't (accelerated flight)
# =====================================================================
RC = np.interp(gr, fc_abs, fc_roll); PC = np.interp(gr, fc_abs, fc_pitch)
TR = np.interp(gr, gt_abs, gt_roll); TP = np.interp(gr, gt_abs, gt_pitch)
RSP = np.interp(gr, fc_abs, rsp); PSP = np.interp(gr, fc_abs, psp)
HS = np.interp(gr, gt_abs, hspd)
est_tilt = np.sqrt(RC ** 2 + PC ** 2); truth_tilt = np.sqrt(TR ** 2 + TP ** 2)
cen = (np.abs(RSP) < 3) & (np.abs(PSP) < 3)
fig, ax = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
ax[0].plot(tr, truth_tilt, color=TRU, lw=1.0, label="truth tilt")
ax[0].plot(tr, est_tilt, color=EST, lw=0.9, alpha=0.85, label="FC estimated tilt")
ax[0].fill_between(tr, 0, 30, where=cen, color="gray", alpha=0.08, label="sticks centered (leveling)")
ax[0].set_ylabel("tilt magnitude (deg)"); ax[0].set_ylim(0, 25); ax[0].legend(loc="upper right", fontsize=7)
ax[0].set_title("Roll/pitch: when sticks are centered the FC reads ~level, but the craft stays tilted")
ax[1].plot(tr, HS, color="tab:purple", lw=0.9)
ax[1].set_ylabel("horiz speed (m/s)"); ax[1].set_xlabel("t (s)")
ax[1].set_title("Horizontal speed — the tilt the FC can't see drives drift (corr 0.46 with the tilt error)")
fig.savefig(os.path.join(OUT, "03_rollpitch_accel_flight.png")); plt.close(fig)

# =====================================================================
# Fig 4 — per-axis error summary + yaw before/after
# =====================================================================
# convention-robust tilt error: angle between est & true gravity-in-body
gb_e = np.stack([-np.sin(np.radians(PC)), np.sin(np.radians(RC)) * np.cos(np.radians(PC)),
                 np.cos(np.radians(RC)) * np.cos(np.radians(PC))], 1)
qw, qx, qy, qz = [np.interp(gr, gt_abs, G(k)) for k in ("qw", "qx", "qy", "qz")]
n = np.sqrt(qw**2 + qx**2 + qy**2 + qz**2); qw, qx, qy, qz = qw/n, qx/n, qy/n, qz/n
gb_t = np.stack([2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx*qx + qy*qy)], 1)
gb_e /= np.linalg.norm(gb_e, axis=1, keepdims=True); gb_t /= np.linalg.norm(gb_t, axis=1, keepdims=True)
tilt_err = np.degrees(np.arccos(np.clip((gb_e * gb_t).sum(1), -1, 1)))
ae = np.interp(gr, vt_abs, fc_alt) - np.interp(gr, gt_abs, gt_alt)
fig, ax = plt.subplots(1, 2, figsize=(11, 3.8))
rms = [np.sqrt((ye**2).mean()), np.sqrt((tilt_err**2).mean()), np.sqrt((ae**2).mean()) * 10]
ax[0].bar(["yaw", "roll/pitch tilt", "altitude (×10 m)"], rms,
          color=["tab:green", "tab:orange", "tab:blue"])
ax[0].set_ylabel("estimate-vs-truth RMS (deg / ×10 m)"); ax[0].set_title("This run — error by axis")
ax[1].bar(["before fix", "after fix"], [121.0, np.sqrt((ye**2).mean())], color=["tab:red", "tab:green"])
ax[1].set_ylabel("yaw error RMS (deg)"); ax[1].set_title("Yaw: mag_fusion fix effect")
fig.savefig(os.path.join(OUT, "04_error_summary.png")); plt.close(fig)

print(f"wrote 4 figures to {OUT}/")
