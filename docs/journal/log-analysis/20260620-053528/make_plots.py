#!/usr/bin/env python3
"""Session-specific analysis plots for the 2026-06-20 SITL run.

This is NOT the generic make_plots.py (that one is bespoke to the 2026-06-17
hardware session: mag spheres, kernel PerfFifo, tilt narrative). This sim run
has no PerfGlobal/PerfFifo/mag and a different headline — the roll/pitch inner
rate loop fails to track while yaw tracks fine — so it gets its own figures.

Reuses the decoder in ../parse_log.py. Writes PNGs into ./plots/.

Usage (from repo root):
    python3 docs/journal/log-analysis/20260620-053528/make_plots.py
"""
import importlib.util
import bisect
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
nl = pl.nl

plt.rcParams.update({"figure.dpi": 110, "font.size": 9, "axes.grid": True,
                     "grid.alpha": 0.3, "figure.autolayout": True})

BIN = os.path.join(HERE, "export-20260620-053528.bin")
OUT = os.path.join(HERE, "plots")
os.makedirs(OUT, exist_ok=True)

_, recs = pl.read_records(BIN)
col = pl.Collector().run(recs)
T0 = min(t for lst in col.msgs.values() for (t, _, _) in lst)
rel = lambda t: (t - T0) / 1e6

ct = col.msgs["ControlTrace"]
vs = col.msgs["VerticalState"]
fm = col.msgs["FlightMode"]
ctx = [rel(t) for t, _, _ in ct]

saved = []
def save(fig, name):
    p = os.path.join(OUT, name)
    fig.savefig(p, bbox_inches="tight"); plt.close(fig); saved.append(name)

def corr(a, b):
    ma, mb = st.mean(a), st.mean(b)
    num = sum((x-ma)*(y-mb) for x, y in zip(a, b))
    den = (sum((x-ma)**2 for x in a) * sum((y-mb)**2 for y in b))**.5
    return num/den if den else 0.0

# ---- 1. Session overview: flight mode + throttle + altitude ----
fig, (a1, a2, a3) = plt.subplots(3, 1, figsize=(10, 6.0), sharex=True)
modes = {0: "ANGLE", 1: "ACRO", 2: "RTRC"}
a1.step([rel(t) for t, _, _ in fm], [m.mode for _, m, _ in fm], where="post", lw=1.0, color="C4")
a1.set_yticks(sorted(set(m.mode for _, m, _ in fm)))
a1.set_yticklabels([modes.get(v, v) for v in sorted(set(m.mode for _, m, _ in fm))])
a1.set_title("Session overview — flight mode, throttle (manual, no alt-hold), altitude")
a1.set_ylabel("flight mode")
a2.plot(ctx, [m.thro_out for _, m, _ in ct], lw=0.5, color="C1")
a2.set_ylabel("thro_out [0..1]")
a3.plot([rel(t) for t, _, _ in vs], [m.altitude for _, m, _ in vs], lw=0.9, label="fused altitude")
a3.plot([rel(t) for t, _, _ in vs], [m.baro_altitude for _, m, _ in vs], lw=0.5, alpha=0.6, label="raw baro")
a3.set_ylabel("alt (m)"); a3.set_xlabel("t (s)"); a3.legend(fontsize=7, loc="upper right")
a3.axvspan(76, 79, color="red", alpha=0.10, label="_attitude upset")
save(fig, "01_session_overview.png")

# ---- 2. Angle tracking: sp (~level) vs actual, all 3 axes ----
fig, axs = plt.subplots(3, 1, figsize=(10, 6.0), sharex=True)
for ax, axis, c in zip(axs, ["roll", "pitch", "yaw"], ["C0", "C2", "C3"]):
    sp = [getattr(m, f"{axis}_angle_sp") for _, m, _ in ct]
    cu = [getattr(m, f"{axis}_angle_curr") for _, m, _ in ct]
    ax.plot(ctx, sp, lw=0.9, color="k", label="setpoint")
    ax.plot(ctx, cu, lw=0.6, color=c, label="actual")
    err = [s - u for s, u in zip(sp, cu)]
    rms = (st.mean(e*e for e in err))**.5
    ax.set_ylabel(f"{axis} (deg)")
    ax.set_title(f"{axis}: angle setpoint vs actual  (tracking RMS {rms:.1f}°)", fontsize=9)
    ax.legend(fontsize=7, loc="upper right")
axs[-1].set_xlabel("t (s)")
save(fig, "02_angle_tracking.png")

# ---- 3. Rate-loop tracking quality: yaw works, roll/pitch don't ----
fig, axs = plt.subplots(1, 3, figsize=(11, 3.6))
for ax, axis in zip(axs, ["roll", "pitch", "yaw"]):
    sp = [getattr(m, f"{axis}_rate_sp") for _, m, _ in ct]
    cu = [getattr(m, f"{axis}_rate_curr") for _, m, _ in ct]
    ax.scatter(sp, cu, s=3, alpha=0.25)
    lim = max(abs(min(sp+cu)), abs(max(sp+cu)))
    ax.plot([-lim, lim], [-lim, lim], "r--", lw=0.8, label="ideal (sp=curr)")
    ax.set_title(f"{axis} rate  r={corr(sp, cu):+.2f}")
    ax.set_xlabel("rate_sp (°/s)"); ax.set_ylabel("rate_curr (°/s)")
    ax.legend(fontsize=7)
fig.suptitle("Inner rate-loop tracking: yaw follows the diagonal (r=0.95); "
             "roll/pitch are a cloud (r≈0.12) — the loop is not tracking", y=1.04)
save(fig, "03_rate_tracking.png")

# ---- 4. Effective authority: output vs rate error (the 36x gain gap) ----
fig, ax = plt.subplots(figsize=(7, 4.4))
for axis, c in [("roll", "C0"), ("pitch", "C2"), ("yaw", "C3")]:
    e = [getattr(m, f"{axis}_rate_sp") - getattr(m, f"{axis}_rate_curr") for _, m, _ in ct]
    o = [getattr(m, f"{axis}_out") for _, m, _ in ct]
    k = sum(x*y for x, y in zip(e, o)) / sum(x*x for x in e)
    ax.scatter(e, o, s=3, alpha=0.25, color=c, label=f"{axis}  Kp_eff={k:.1e}")
ax.axhline(1, color="r", ls=":", lw=0.7); ax.axhline(-1, color="r", ls=":", lw=0.7,
          label="output limit ±1")
ax.set_title("Rate-loop authority: controller output vs rate error\n"
             "roll/pitch never leave ±0.13; yaw uses the range")
ax.set_xlabel("rate error (°/s)"); ax.set_ylabel("controller out [-1..1]")
ax.legend(fontsize=7); ax.axhline(0, color="k", lw=0.4); ax.axvline(0, color="k", lw=0.4)
save(fig, "04_rate_authority.png")

# ---- 5. The t=76-79s attitude upset zoom ----
seg = [(rel(t), m) for t, m, _ in ct if 75.5 <= rel(t) <= 79.5]
if seg:
    tx = [t for t, _ in seg]
    fig, (a1, a2, a3) = plt.subplots(3, 1, figsize=(10, 6.2), sharex=True)
    a1.plot(tx, [m.pitch_angle_curr for _, m in seg], color="C2", lw=1.0, label="pitch actual")
    a1.plot(tx, [m.pitch_angle_sp for _, m in seg], color="k", lw=0.8, label="pitch sp (~0)")
    a1.axhline(0, color="k", lw=0.4)
    a1.set_ylabel("pitch (deg)"); a1.legend(fontsize=7, loc="upper right")
    a1.set_title("Attitude upset t≈77 s: pitch reaches +17.8° from a level command")
    a2.plot(tx, [m.pitch_rate_sp for _, m in seg], color="k", lw=0.9, label="rate_sp (outer demands recovery)")
    a2.plot(tx, [m.pitch_rate_curr for _, m in seg], color="C2", lw=0.7, label="rate_curr (actual)")
    a2.axhline(0, color="k", lw=0.4); a2.set_ylabel("pitch rate (°/s)")
    a2.legend(fontsize=7, loc="upper right")
    a3.plot(tx, [m.pitch_out for _, m in seg], color="C3", lw=0.9, label="pitch_out")
    a3.axhline(0, color="k", lw=0.4)
    a3.set_ylim(-1.05, 1.05)
    a3.set_ylabel("pitch_out [-1..1]"); a3.set_xlabel("t (s)")
    a3.legend(fontsize=7, loc="upper right")
    a3.text(0.01, 0.06, "outer loop screams −53°/s; inner loop answers with −0.025 of ±1 authority",
            transform=a3.transAxes, fontsize=7.5, color="#a00")
    save(fig, "05_pitch_upset.png")

# ---- 6. Vertical estimator health ----
vt = [rel(t) for t, _, _ in vs]
fig, (a1, a2) = plt.subplots(2, 1, figsize=(10, 4.6), sharex=True)
a1.plot(vt, [m.altitude for _, m, _ in vs], lw=0.9, label="fused altitude")
a1.plot(vt, [m.baro_altitude for _, m, _ in vs], lw=0.5, alpha=0.6, label="raw baro")
a1.plot(vt, [m.agl for _, m, _ in vs], lw=0.5, alpha=0.6, label="AGL")
a1.set_ylabel("m")
res = [m.altitude - m.baro_altitude for _, m, _ in vs]
a1.set_title(f"Vertical estimator: fused vs baro (residual RMS {(st.mean(r*r for r in res))**.5:.2f} m) "
             f"— estimator is healthy")
a1.legend(fontsize=7, loc="upper right")
a2.plot(vt, [m.climb_rate for _, m, _ in vs], lw=0.7, color="C1", label="climb_rate")
a2.axhline(0, color="k", lw=0.4)
a2.set_ylabel("climb (m/s)"); a2.set_xlabel("t (s)"); a2.legend(fontsize=7, loc="upper right")
save(fig, "06_vertical_estimator.png")

print(f"wrote {len(saved)} plots to {OUT}/:")
for n in saved:
    print(" ", n)
