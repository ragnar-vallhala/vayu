#!/usr/bin/env python3
"""Regenerate every figure in this archive from data/imuhs-20260909-001748.bin.

Run from anywhere:  python3 <this file>
Figures land in plots/ and are referenced by number from analysis.md.
"""
import os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "tools", "telemetry"))
import hslog

DATA = os.path.join(HERE, "data", "imuhs-20260909-001748.bin")
OUT = os.path.join(HERE, "plots")
os.makedirs(OUT, exist_ok=True)

BLUE, RED, GRN, ORG, PUR, GRY = "#61AFEF", "#E06C75", "#98C379", "#D19A66", "#C678DD", "#7F848E"
plt.rcParams.update({"figure.dpi": 130, "font.size": 9, "axes.grid": True,
                     "grid.alpha": 0.25, "axes.titlesize": 10})

hdr, streams, frames = hslog.decode(open(DATA, "rb").read())
ss = hslog.sessions(frames)
MIX = np.c_[[-1, -1, 1, 1], [1, -1, -1, 1], [-1, 1, -1, 1]].astype(float)
ASC = abs(streams[1].fields[3][2])


def st(g, sid):
    b = [x for x in g["blocks"] if x["sid"] == sid]
    return hslog.samples(hdr, streams, b, sid) if b else (None, None)


def save(f, name):
    f.tight_layout()
    f.savefig(os.path.join(OUT, name))
    plt.close(f)
    print("  ", name)


# --- Fig 1 : staleness ------------------------------------------------------
g = ss[8]; t, I = st(g, 1)
m = (t >= 1.50) & (t < 1.60)
f, ax = plt.subplots(figsize=(7, 3.2))
ax.step(t[m] * 1000, I["az"][m], where="post", lw=1.3, color=PUR,
        label="az as logged (1827 Hz)")
ch = np.flatnonzero(np.diff(I["az"][m]) != 0) + 1
ax.plot(t[m][ch] * 1000, I["az"][m][ch], "o", ms=4, color=RED,
        label="sensor actually updated (~98 Hz)")
ax.set_xlabel("time within run 9 (ms)"); ax.set_ylabel("vertical accel  az  (m s$^{-2}$)")
ax.set_title("Fig 1 — the IMU is polled 19x per fresh sample")
ax.legend(loc="upper right", fontsize=8)
save(f, "fig01-sample-staleness.png")

# --- Fig 2 : |a| vs throttle -----------------------------------------------
xs, ys, cs = [], [], []
for gg in ss:
    t, I = st(gg, 1); ta, A = st(gg, 2)
    if I is None or A is None: continue
    acc = np.sqrt(I["ax"]**2 + I["ay"]**2 + I["az"]**2)
    for k in range(int(min(t[-1], ta[-1]) * 4)):
        lo, hi = k * .25, (k + 1) * .25
        mi = (t >= lo) & (t < hi); ma = (ta >= lo) & (ta < hi)
        if mi.sum() < 20 or ma.sum() < 2: continue
        xs.append(A["thr"][ma].mean()); ys.append(acc[mi].mean())
        cs.append(RED if hslog.session_accel_unhealthy(gg) else BLUE)
f, ax = plt.subplots(figsize=(7, 4))
ax.scatter(xs, ys, c=cs, s=14, alpha=.75, edgecolors="none")
ax.axhline(9.80665, color=GRN, ls="--", lw=1.3, label="true gravity, 9.807 m s$^{-2}$")
ax.scatter([], [], c=RED, s=14, label="arm where accel-health latched")
ax.scatter([], [], c=BLUE, s=14, label="arm that did not latch")
ax.set_xlabel("commanded collective (throttle setpoint, 0–1)")
ax.set_ylabel("mean measured |accel|  (m s$^{-2}$)")
ax.set_title("Fig 2 — measured gravity collapses with throttle, on a stationary bench")
ax.legend(loc="lower left", fontsize=8)
save(f, "fig02-accel-vs-throttle.png")

# --- Fig 3 : not amplitude --------------------------------------------------
mm, ls, sd, rn = [], [], [], []
for i, gg in enumerate(ss):
    t, I = st(gg, 1); ta, A = st(gg, 2)
    if I is None or A is None: continue
    acc = np.sqrt(I["ax"]**2 + I["ay"]**2 + I["az"]**2)
    for k in range(int(min(t[-1], ta[-1]) * 8)):
        lo, hi = k * .125, (k + 1) * .125
        mi = (t >= lo) & (t < hi); ma = (ta >= lo) & (ta < hi)
        if mi.sum() < 15 or ma.sum() < 2: continue
        mm.append(np.mean([A["m%d" % j][ma].mean() for j in (1, 2, 3, 4)]))
        ls.append(9.80665 - acc[mi].mean()); sd.append(acc[mi].std()); rn.append(i + 1)
mm, ls, sd, rn = map(np.array, (mm, ls, sd, rn))
f, axs = plt.subplots(1, 2, figsize=(10, 4))
e = np.arange(0.16, 0.60, 0.02); c, v, se = [], [], []
for a_, b_ in zip(e, e[1:]):
    k = (mm >= a_) & (mm < b_)
    if k.sum() < 4: continue
    c.append((a_ + b_) / 2); v.append(ls[k].mean()); se.append(ls[k].std() / np.sqrt(k.sum()))
axs[0].errorbar(c, v, yerr=se, marker="o", ms=4, lw=1.4, color=BLUE, capsize=2,
                label="mean $\\pm$ s.e.")
axs[0].set_xlabel("mean motor command (0–1)")
axs[0].set_ylabel("gravity lost  $9.807-|a|$  (m s$^{-2}$)")
axs[0].set_title("Fig 3a — loss peaks mid-throttle, then falls\n(an amplitude effect could only rise)")
axs[0].legend(fontsize=8)
sel = (mm >= 0.30) & (mm < 0.40)
for r in np.unique(rn[sel]):
    k = sel & (rn == r)
    if k.sum() < 3: continue
    bad = hslog.session_accel_unhealthy(ss[r - 1])
    axs[1].scatter(sd[k].mean(), ls[k].mean(), s=45, color=RED if bad else BLUE)
    axs[1].annotate(str(r), (sd[k].mean(), ls[k].mean()), fontsize=7,
                    xytext=(4, 3), textcoords="offset points")
axs[1].scatter([], [], c=RED, s=45, label="accel-health latched")
axs[1].scatter([], [], c=BLUE, s=45, label="did not latch")
axs[1].set_xlabel("in-band vibration actually measured,  sd of $|a|$  (m s$^{-2}$)")
axs[1].set_ylabel("gravity lost  (m s$^{-2}$)")
axs[1].set_title("Fig 3b — same throttle, same vibration,\n4x different loss (labels = run)")
axs[1].set_xlim(0.9, 1.6); axs[1].legend(fontsize=8)
save(f, "fig03-loss-not-amplitude.png")

# --- Fig 4 : run 9 timeline -------------------------------------------------
g = ss[8]; t, I = st(g, 1); ta, A = st(g, 2); tv, V = st(g, 3)
acc = np.sqrt(I["ax"]**2 + I["ay"]**2 + I["az"]**2)
f, axs = plt.subplots(3, 1, figsize=(7.5, 6.6), sharex=True)
axs[0].plot(ta, A["thr"], lw=1.2, color=BLUE, label="commanded collective")
axs[0].plot(ta, np.maximum.reduce([A["m%d" % k] for k in (1, 2, 3, 4)]), lw=1,
            color=ORG, label="highest motor command")
axs[0].axhline(1.0, color=GRY, ls=":", lw=1, label="motor rail")
axs[0].set_ylabel("command (0–1)"); axs[0].legend(fontsize=8, loc="upper left")
axs[0].set_title("Fig 4 — run 9: throttle up $\\rightarrow$ gravity collapses $\\rightarrow$ bias clamps $\\rightarrow$ FAILSAFE")
axs[1].plot(t, acc, lw=.4, color=BLUE, label="measured $|a|$")
axs[1].axhline(9.80665, color=GRN, ls="--", lw=1.2, label="true gravity")
axs[1].set_ylabel("$|a|$  (m s$^{-2}$)"); axs[1].legend(fontsize=8, loc="upper right")
axs[2].plot(tv, V["abias"], lw=1.3, color=RED, label="estimator accel_bias")
axs[2].plot(tv, V["climb"], lw=1, color=ORG, label="estimated climb rate")
axs[2].axhline(-3.0, color=RED, ls=":", lw=1.1, label="bias clamp, VERT_ACCEL_BIAS_MAX")
axs[2].set_ylabel("m s$^{-2}$  /  m s$^{-1}$"); axs[2].set_xlabel("time since arm (s)")
axs[2].legend(fontsize=8, loc="lower right")
save(f, "fig04-run9-timeline.png")

# --- Fig 5 : D-term ---------------------------------------------------------
m = (t >= 3.00) & (t < 3.10); tt = t[m]
tk = np.arange(tt[0], tt[-1], 0.001); gk = I["gx"][m][np.searchsorted(tt, tk) - 1]
d = np.diff(gk) / 0.001
f, axs = plt.subplots(2, 1, figsize=(7.5, 4.8), sharex=True)
axs[0].step(tk * 1000, gk, where="post", lw=1.3, color=PUR, label="gx as the 1 kHz loop sees it")
axs[0].set_ylabel("roll rate  gx  (deg s$^{-1}$)"); axs[0].legend(fontsize=8)
axs[0].set_title("Fig 5 — a 1 kHz derivative of a 98 Hz signal: 90% zeros, 10% impulse")
axs[1].plot(tk[1:] * 1000, d, lw=1, color=RED, label="$D = \\Delta$err$\\,/\\,$INNER_LOOP_DT")
axs[1].set_ylabel("d(gx)/dt  (deg s$^{-2}$)"); axs[1].set_xlabel("time within run 9 (ms)")
axs[1].legend(fontsize=8)
save(f, "fig05-dterm-impulse-train.png")

# --- Fig 6 : chain check lag sweep -----------------------------------------
U, AW, CO = [], [], []
for gg in ss:
    ti, I2 = st(gg, 1); ta, A = st(gg, 2)
    if I2 is None or A is None: continue
    M = np.c_[[A["m%d" % j] for j in (1, 2, 3, 4)]].T
    uns = ((M > 0.1505) & (M < 0.9995)).all(1) & (A["thr"] > 0.05)
    if uns.sum() < 300: continue
    gy = np.c_[[np.interp(ta, ti, I2[k]) for k in ("gx", "gy", "gz")]].T
    dt = np.median(np.diff(ta))
    U.append(M @ MIX / 4.0); AW.append(np.gradient(gy, axis=0) / dt); CO.append(uns)
dt = 0.00303


def corr(a, b):
    a = a - a.mean(); b = b - b.mean()
    dd = np.sqrt((a * a).sum() * (b * b).sum())
    return float((a * b).sum() / dd) if dd > 0 else np.nan


lags = list(range(0, 61, 2))
cur = {k: [] for k in range(3)}
for lag in lags:
    for k in range(3):
        vv = []
        for u, aw, mk in zip(U, AW, CO):
            if lag: a, b, mm2 = u[:-lag, k], aw[lag:, k], mk[:-lag] & mk[lag:]
            else: a, b, mm2 = u[:, k], aw[:, k], mk
            if mm2.sum() > 200: vv.append(corr(a[mm2], b[mm2]))
        cur[k].append(np.median(vv) if vv else np.nan)
f, ax = plt.subplots(figsize=(7.2, 4))
x = np.array(lags) * dt * 1000
for k, (nm, col) in enumerate(zip(("roll", "pitch", "yaw"), (BLUE, ORG, RED))):
    ax.plot(x, cur[k], lw=1.6, color=col, label=nm)
ax.axhline(0, color=GRY, lw=1)
ax.annotate("roll peak +0.53\n48 ms", (48, 0.53), xytext=(60, 0.48), fontsize=8,
            arrowprops=dict(arrowstyle="->", color=GRY, lw=.9))
ax.annotate("half-period 109 ms\n$\\Rightarrow$ ~4.5 Hz loop oscillation", (109, -0.31),
            xytext=(115, -0.20), fontsize=8,
            arrowprops=dict(arrowstyle="->", color=GRY, lw=.9))
ax.set_xlabel("lag $\\tau$ applied to the angular acceleration (ms)")
ax.set_ylabel("median corr$\\,(u(t),\\ \\dot\\omega(t+\\tau))$")
ax.set_title("Fig 6 — does the commanded torque actually move the airframe?\n(unsaturated samples only; positive = same direction)")
ax.legend(fontsize=8, title="axis", title_fontsize=8)
save(f, "fig06-chain-lag-sweep.png")

# --- Fig 7 : motor asymmetry ------------------------------------------------
rows, labels = [], []
for i, gg in enumerate(ss):
    ti, I2 = st(gg, 1); ta, A = st(gg, 2)
    if I2 is None or A is None: continue
    M = np.c_[[A["m%d" % j] for j in (1, 2, 3, 4)]].T
    gy = np.c_[[np.interp(ta, ti, I2[k]) for k in ("gx", "gy", "gz")]].T
    ax3 = np.c_[[np.interp(ta, ti, I2[k]) for k in ("ax", "ay", "az")]].T
    n = np.linalg.norm(ax3, axis=1)
    tilt = np.degrees(np.arccos(np.clip(-ax3[:, 2] / np.maximum(n, 1e-6), -1, 1)))
    mk = (((M > 0.1505) & (M < 0.9995)).all(1) & (A["thr"] > 0.05)
          & (np.abs(gy) < 20).all(1) & (tilt < 15) & (n > 4))
    if mk.sum() < 60: continue
    rows.append(M[mk].mean(0)); labels.append(i + 1)
R = np.array(rows)
f, ax = plt.subplots(figsize=(7.6, 4))
w = 0.2; xx = np.arange(len(labels))
for j, col in enumerate((BLUE, ORG, RED, GRN)):
    ax.bar(xx + (j - 1.5) * w, R[:, j], w, color=col, label="m%d" % (j + 1))
ax.set_xticks(xx); ax.set_xticklabels(["run %d" % l for l in labels], fontsize=8)
ax.set_ylabel("mean motor command (0–1)")
ax.set_title("Fig 7 — motor 3 holds a standing deficit (quasi-static samples:\nunsaturated, $|\\omega|<20$ deg s$^{-1}$, tilt $<15$ deg)")
ax.legend(fontsize=8, ncol=4, title="motor", title_fontsize=8)
save(f, "fig07-motor-asymmetry.png")

# --- Fig 8 : ToF aiding -----------------------------------------------------
f, ax = plt.subplots(figsize=(7.2, 4.2))
for i, gg in enumerate(ss):
    tv, V = st(gg, 3); ta, A = st(gg, 2)
    if V is None or A is None or A["thr"].max() < 0.15 or len(tv) < 40: continue
    tof = (V["flags"].astype(int) & 1).mean()
    col = BLUE if tof > 0.4 else (ORG if tof > 0.05 else RED)
    ax.plot(tv, V["abias"], lw=1.2, color=col, alpha=.85)
    ax.annotate("%d" % (i + 1), (tv[-1], V["abias"][-1]), fontsize=7,
                xytext=(3, 0), textcoords="offset points", color=col)
ax.axhline(-3.0, color=GRY, ls=":", lw=1.1, label="clamp, VERT_ACCEL_BIAS_MAX")
ax.plot([], [], color=BLUE, lw=1.4, label="rangefinder live (>40% valid)")
ax.plot([], [], color=ORG, lw=1.4, label="marginal (13% valid, run 7)")
ax.plot([], [], color=RED, lw=1.4, label="rangefinder dead (0%)")
ax.set_xlabel("time since arm (s)"); ax.set_ylabel("estimator accel_bias  (m s$^{-2}$)")
ax.set_title("Fig 8 — ToF velocity aiding converges the bias 2–3x faster\n(labels = run number)")
ax.legend(fontsize=8, loc="lower left")
save(f, "fig08-tof-aiding.png")

# --- Fig 9 : oscillation spectra -------------------------------------------
f, ax = plt.subplots(figsize=(7.2, 4))
early = {2, 3, 5, 8, 9}
for i, gg in enumerate(ss):
    ta, A = st(gg, 2)
    if A is None or (A["thr"] > 0.05).sum() < 800: continue
    M = np.c_[[A["m%d" % j] for j in (1, 2, 3, 4)]].T
    fs = (len(ta) - 1) / (ta[-1] - ta[0]); u = (M @ MIX / 4.0)[:, 0]
    y = u - u.mean(); w2 = np.hanning(len(y))
    mag = np.abs(np.fft.rfft(y * w2)) * (2 / w2.sum())
    fr = np.fft.rfftfreq(len(y), 1 / fs); k = (fr > 0.5) & (fr < 30)
    ax.semilogy(fr[k], mag[k], lw=1.1,
                color=BLUE if (i + 1) in early else RED, alpha=.85)
ax.plot([], [], color=BLUE, lw=1.4, label="early arms (2,3,5,8,9): 4.6–7.6 Hz")
ax.plot([], [], color=RED, lw=1.4, label="later arms (10,14,15,16,17): 0.8–1.3 Hz")
ax.axvline(1.79, color=GRN, ls="--", lw=1.2, label="1.79 Hz limit cycle (2026-06)")
ax.set_xlabel("frequency (Hz)   —   act stream is not gyro-aliased, valid to ~160 Hz")
ax.set_ylabel("roll-demand amplitude (motor units)")
ax.set_title("Fig 9 — two oscillation regimes, neither the 2026-06 one")
ax.legend(fontsize=8)
save(f, "fig09-oscillation-spectra.png")
print("done")
