#!/usr/bin/env python3
"""Generate the figures for the pitch-indi campaign analysis into ./plots/.

Bespoke to this archive. Reuses the segmented loader/metrics so every figure is
computed inside a homogeneous flight regime (never blanket-averaged):
  _pipeline.load  -> time-aligned pipeline series + reconstructed IMU
  _segment.segment -> IDLE / SPOOL / AIRBORNE regimes + impact events
  analyze_campaign.window_metrics -> per-window pipeline metrics

Usage (from this dir):  python3 make_plots.py
"""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import _pipeline as P
import _segment as S
import analyze_campaign as A

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "plots")
os.makedirs(OUT, exist_ok=True)
plt.rcParams.update({"figure.dpi": 120, "font.size": 9, "axes.grid": True,
                     "grid.alpha": 0.3, "figure.autolayout": True})

# (key, controller-family, path) in campaign order
LOGS = [
    ("pitch-verify", "PID", "20260625-233852-pitch-verify/pitch-verify.bin"),
    ("telem30s",     "PID", "20260626-001343-telem30s/stream.bin"),
    ("indi-test",    "INDI k20", "20260626-010249-indi-test/stream.bin"),
    ("freeair",      "INDI k20", "20260626-011558-indi-freeair/stream.bin"),
    ("k20-openair",  "INDI k20", "20260626-013249-indi-k20-openair/indi-k20-openair.bin"),
    ("pid-openair",  "PID", "20260626-014848-pid-openair/pid-openair.bin"),
    ("k6-seed",      "INDI k6", "20260626-020449-indi-k6-seed/indi-k6-seed.bin"),
]
FAMCOL = {"PID": "#d62728", "INDI k20": "#1f77b4", "INDI k6": "#2ca02c"}

print("loading…")
DATA = {}
for k, fam, rel in LOGS:
    d = P.load(os.path.join(HERE, rel))
    segs, info = S.segment(d)
    DATA[k] = (fam, d, segs, info)


def active_windows(k, minlen=1.0):
    fam, d, segs, info = DATA[k]
    out = []
    for s in segs:
        if s["kind"] in ("AIRBORNE", "SPOOL") and s["dur"] >= minlen:
            out.append((s, A.window_metrics(d, s)))
    return out


# ---------- 00 pipeline diagram (matplotlib, no mermaid dependency) -------
def p00():
    import matplotlib.patches as mpatches
    fig, ax = plt.subplots(figsize=(9.2, 2.8))
    ax.set_xlim(0, 10); ax.set_ylim(0, 3); ax.axis("off")
    boxes = [
        (0.3, "IMU\nBMX160 @1kHz", "ok"),
        (2.0, "Attitude\nestimator", "ok"),
        (3.8, "Rate loop\nPID / INDI", "ok"),
        (5.7, "Mixer\nanti-sat + idle floor", "bad"),
        (7.7, "4 motors\n(0..1)", "bad"),
    ]
    col = {"ok": ("#e7f5e7", "#2ca02c"), "bad": ("#fde7e7", "#d62728")}
    cx = []
    for x, label, kind in boxes:
        fc, ec = col[kind]
        ax.add_patch(mpatches.FancyBboxPatch((x, 1.1), 1.5, 0.8,
                     boxstyle="round,pad=0.04", fc=fc, ec=ec, lw=1.6))
        ax.text(x + 0.75, 1.5, label, ha="center", va="center", fontsize=8)
        cx.append(x + 1.5)
    for i in range(len(boxes) - 1):
        ax.annotate("", xy=(boxes[i + 1][0], 1.5), xytext=(cx[i], 1.5),
                    arrowprops=dict(arrowstyle="->", lw=1.3, color="#444"))
    # setpoint into rate loop
    ax.text(3.8 + 0.75, 2.55, "RC / pilot setpoints", ha="center", fontsize=7.5, color="#555")
    ax.annotate("", xy=(3.8 + 0.75, 1.9), xytext=(3.8 + 0.75, 2.4),
                arrowprops=dict(arrowstyle="->", lw=1.1, color="#888"))
    # thrust feedback loop motors -> IMU
    ax.annotate("", xy=(0.3 + 0.75, 1.1), xytext=(7.7 + 0.75, 1.1),
                arrowprops=dict(arrowstyle="->", lw=1.0, color="#888",
                                connectionstyle="arc3,rad=0.25", ls="--"))
    ax.text(4.6, 0.35, "thrust → vibration / motion (closed loop)", ha="center",
            fontsize=7, color="#888")
    ax.text(0.3, 2.6, "green = verified clean", color="#2ca02c", fontsize=8)
    ax.text(5.7, 2.6, "red = where flight is lost", color="#d62728", fontsize=8)
    fig.savefig(os.path.join(OUT, "00_pipeline.png")); plt.close(fig)


# ---------- 01 regime budget: there was never sustained flight ----------
def p01():
    fig, ax = plt.subplots(figsize=(9, 3.8))
    keys = [k for k, *_ in LOGS]
    order = ["IDLE", "SPOOL", "AIRBORNE"]
    cols = {"IDLE": "#bbbbbb", "SPOOL": "#ff9e4a", "AIRBORNE": "#2ca02c"}
    bottoms = np.zeros(len(keys))
    for kind in order:
        vals = []
        for k in keys:
            fam, d, segs, info = DATA[k]
            span = d["ct"][-1, 0] - d["ct"][0, 0]
            tot = sum(s["dur"] for s in segs if s["kind"] == kind)
            vals.append(100 * tot / span)
        ax.bar(keys, vals, bottom=bottoms, color=cols[kind], label=kind)
        bottoms += np.array(vals)
    for i, k in enumerate(keys):
        fam, d, segs, info = DATA[k]
        span = d["ct"][-1, 0] - d["ct"][0, 0]
        air = sum(s["dur"] for s in segs if s["kind"] == "AIRBORNE")
        ax.text(i, 101, f"{air:.0f}s air", ha="center", va="bottom", fontsize=6.5)
    ax.set_ylabel("% of capture")
    ax.set_ylim(0, 108)
    ax.set_title("Flight-regime budget — every capture is 80–95% IDLE; AIRBORNE is "
                 "seconds of 1–3 s hops, never sustained")
    ax.legend(fontsize=7, ncol=3, loc="lower right")
    ax.tick_params(axis="x", labelsize=7, rotation=12)
    fig.savefig(os.path.join(OUT, "01_regime_budget.png")); plt.close(fig)


# ---------- 02 the closed limit cycle: rate / out / motor lock-step --------
def p02():
    fam, d, segs, info = DATA["telem30s"]
    ct, mot = d["ct"], d["mot"]
    seg = max((s for s in segs if s["kind"] == "SPOOL"), key=lambda s: s["dur"])
    sl = slice(seg["i0"], seg["i1"])
    t = ct[sl, 0]; t0 = t[0]
    pr = ct[sl, P.CT_I["prate"]]; pout = ct[sl, P.CT_I["pitch_out"]]
    mm = (mot[:, 0] >= seg["t0"]) & (mot[:, 0] <= seg["t1"])
    tm = mot[mm, 0] - t0; M = mot[mm, 1:5]
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 5.2), sharex=True)
    ax1.plot(t - t0, pr, color="#d62728", lw=0.8, label="pitch rate (°/s)")
    ax1b = ax1.twinx()
    ax1b.plot(t - t0, pout, color="#1f77b4", lw=0.8, alpha=0.8, label="pitch_out u")
    ax1b.axhline(0.99, color="#1f77b4", ls=":", lw=0.7); ax1b.axhline(-0.99, color="#1f77b4", ls=":", lw=0.7)
    ax1b.set_ylabel("pitch_out u", color="#1f77b4"); ax1b.set_ylim(-1.1, 1.1); ax1b.grid(False)
    ax1.set_ylabel("pitch rate (°/s)", color="#d62728")
    ax1.set_title("telem30s — closed limit cycle: pitch rate, controller output u, and motors all at ~2.0 Hz")
    l1, la1 = ax1.get_legend_handles_labels(); l2, la2 = ax1b.get_legend_handles_labels()
    ax1.legend(l1 + l2, la1 + la2, loc="upper right", fontsize=7, ncol=2)
    nm = ["M1", "M2", "M3", "M4"]; cols = ["#1f77b4", "#9467bd", "#d62728", "#ff7f0e"]
    for i in range(4):
        ax2.plot(tm, M[:, i], lw=0.7, color=cols[i], label=nm[i])
    ax2.axhline(0.005, color="k", ls="--", lw=0.8, alpha=0.7)
    ax2.text(tm[-1] * 0.55, 0.03, "MOTOR_IDLE_FLOOR = 0.005  (low motor pinned here ~95% of cycle)",
             fontsize=6.5, color="#b00")
    ax2.set_ylabel("motor cmd (0..1)"); ax2.set_xlabel("time (s)"); ax2.set_ylim(-0.02, 1.02)
    ax2.legend(fontsize=7, ncol=4, loc="upper right")
    fig.savefig(os.path.join(OUT, "02_limit_cycle.png")); plt.close(fig)


# ---------- 03 dominant frequency is owned by the controller ----------
def p03():
    fig, ax = plt.subplots(figsize=(9, 3.8))
    x = 0; ticks = []; tlabels = []
    for k, fam, _ in LOGS:
        ws = active_windows(k, 1.5)
        for s, m in ws:
            if not np.isfinite(m["pfreq"]):
                continue
            ax.bar(x, m["pfreq"], color=FAMCOL[fam], width=0.8)
            x += 1
        if x and (not ticks or ticks[-1] != x - 1):
            ticks.append(x - 1); tlabels.append(k)
        x += 0.6
    for fam, c in FAMCOL.items():
        ax.bar(np.nan, 0, color=c, label=fam)
    ax.axhspan(1.7, 2.1, color="#d62728", alpha=0.08)
    ax.axhspan(3.3, 3.9, color="#1f77b4", alpha=0.08)
    ax.text(0.2, 2.0, "PID 1.7–2.1 Hz (gain-invariant)", fontsize=7, color="#b00")
    ax.text(0.2, 3.7, "INDI k20 ≈ bandwidth (3.3–3.8 Hz)", fontsize=7, color="#1147a0")
    ax.set_ylabel("dominant pitch-rate freq (Hz)")
    ax.set_xticks(ticks); ax.set_xticklabels(tlabels, fontsize=7, rotation=12)
    ax.set_title("Limit-cycle frequency tracks the CONTROLLER, not the airframe "
                 "(each bar = one active window)")
    ax.legend(fontsize=7, loc="upper right")
    fig.savefig(os.path.join(OUT, "03_freq_by_controller.png")); plt.close(fig)


# ---------- 04 THE plot: saturation side flips with throttle ----------
def p04():
    fig, ax = plt.subplots(figsize=(9, 4.2))
    for k, fam, _ in LOGS:
        for s, m in active_windows(k, 1.0):
            if not np.isfinite(m["floor"]):
                continue
            ax.plot(m["thr"], m["floor"], "v", color="#ff7f0e", ms=7,
                    mec="k", mew=0.3)
            ax.plot(m["thr"], m["rail"], "^", color="#1f77b4", ms=7,
                    mec="k", mew=0.3)
    ax.plot(np.nan, np.nan, "v", color="#ff7f0e", mec="k", label="% motors at idle FLOOR (down-starved)")
    ax.plot(np.nan, np.nan, "^", color="#1f77b4", mec="k", label="% motors at RAIL (up-starved)")
    ax.axvline(0.5, color="k", ls=":", lw=0.8)
    ax.set_xlabel("window mean throttle"); ax.set_ylabel("% of samples saturated")
    ax.set_title("Throttle-headroom starvation: low throttle floors the low motor, high "
                 "throttle rails the high one\n— the saturating side FLIPS with throttle; "
                 "no throttle leaves room for the demanded differential")
    ax.legend(fontsize=7.5, loc="center right")
    ax.set_ylim(-3, 103)
    fig.savefig(os.path.join(OUT, "04_saturation_vs_throttle.png")); plt.close(fig)


# ---------- 05 motor differential demand vs available headroom ----------
def p05():
    fig, ax = plt.subplots(figsize=(9, 3.8))
    T = np.linspace(0.05, 0.95, 100)
    ax.fill_between(T, 0, np.minimum(T - 0.005, 1 - T), color="#2ca02c", alpha=0.15)
    ax.plot(T, T - 0.005, color="#ff7f0e", lw=1.5, label="down-authority  (T − floor)")
    ax.plot(T, 1 - T, color="#1f77b4", lw=1.5, label="up-authority  (1 − T)")
    ax.plot(T, np.minimum(T - 0.005, 1 - T), color="#2ca02c", lw=2.2,
            label="usable symmetric authority")
    # overlay demanded half-spread per window
    for k, fam, _ in LOGS:
        for s, m in active_windows(k, 1.0):
            if np.isfinite(m["mspread"]):
                ax.plot(m["thr"], m["mspread"] / 2, "o", color=FAMCOL[fam],
                        ms=5, mec="k", mew=0.3, alpha=0.8)
    ax.plot(np.nan, np.nan, "ko", label="realised half-differential (mSprd/2)")
    ax.set_xlabel("throttle"); ax.set_ylabel("motor authority / differential (0..1)")
    ax.set_title("The realised motor differential (dots) rides right at the usable\n"
                 "symmetric-authority ceiling (green) — the anti-sat clip is always active",
                 fontsize=9.5)
    ax.legend(fontsize=7, loc="upper right"); ax.set_xlim(0, 1); ax.set_ylim(0, 0.6)
    fig.savefig(os.path.join(OUT, "05_authority_headroom.png")); plt.close(fig)


# ---------- 06 sensor vibration by regime ----------
def p06():
    fig, ax = plt.subplots(figsize=(9, 3.8))
    keys = [k for k, *_ in LOGS]
    regimes = ["IDLE", "SPOOL", "AIRBORNE"]
    cols = {"IDLE": "#bbbbbb", "SPOOL": "#ff9e4a", "AIRBORNE": "#2ca02c"}
    w = 0.26
    xs = np.arange(len(keys))
    for j, reg in enumerate(regimes):
        vals = []
        for k in keys:
            fam, d, segs, info = DATA[k]
            acc = []
            for s in segs:
                if s["kind"] == reg and s["dur"] >= 1.5:
                    mi = (d["imu"][:, 0] >= s["t0"]) & (d["imu"][:, 0] <= s["t1"])
                    if mi.sum() > 4:
                        acc.append((np.linalg.norm(d["imu"][mi, 1:4], axis=1) / A.G).std())
            vals.append(np.mean(acc) if acc else 0)
        ax.bar(xs + (j - 1) * w, vals, w, color=cols[reg], label=reg)
    ax.set_ylabel("|accel| RMS (g, aliased)")
    ax.set_title("Accel vibration explodes the instant motors spin up (idle → active), "
                 "5–20× — degrades the attitude estimate")
    ax.set_xticks(xs); ax.set_xticklabels(keys, fontsize=7, rotation=12)
    ax.legend(fontsize=7)
    fig.savefig(os.path.join(OUT, "06_vibration_by_regime.png")); plt.close(fig)


# ---------- 07 crashes: impact events ----------
def p07():
    fig, ax = plt.subplots(figsize=(9, 3.8))
    for k, fam, _ in LOGS:
        fam, d, segs, info = DATA[k]
        if not info["events"]:
            continue
        g = [e["acc_max_g"] for e in info["events"]]
        gy = [e["gyro_max"] for e in info["events"]]
        ax.scatter(g, gy, s=28, color=FAMCOL[fam], alpha=0.7, edgecolor="k",
                   linewidth=0.3, label=None)
    ax.axvline(2.5, color="k", ls=":", lw=0.8); ax.axhline(300, color="k", ls=":", lw=0.8)
    ax.text(2.6, 50, "impact threshold 2.5 g", fontsize=7)
    ax.text(0.1, 320, "tumble threshold 300 °/s", fontsize=7)
    for fam, c in FAMCOL.items():
        ax.scatter(np.nan, np.nan, color=c, label=fam, edgecolor="k", linewidth=0.3)
    ax.set_xlabel("peak |accel| (g)"); ax.set_ylabel("peak |gyro| (°/s)")
    ax.set_title("Every hop ends in a crash: ground impacts to 12 g and tumbles to 950 °/s")
    ax.legend(fontsize=7, loc="upper right")
    fig.savefig(os.path.join(OUT, "07_impacts.png")); plt.close(fig)


# ---------- 08 sim vs real: same firmware, different plant ----------
SIM_LOGS = [
    ("sim 06-20", "20260620-053528/export-20260620-053528.bin"),
    ("sim 06-21", "20260621-053142/export-20260621-053142.bin"),
]


def _windows_for(path):
    d = P.load(path)
    segs, info = S.segment(d)
    span = d["ct"][-1, 0] - d["ct"][0, 0]
    air = sum(s["dur"] for s in segs if s["kind"] == "AIRBORNE")
    ws = [(s, A.window_metrics(d, s)) for s in segs
          if s["kind"] in ("AIRBORNE", "SPOOL") and s["dur"] >= 1.0]
    return 100 * air / span, ws


def p08():
    sim = {"air": [], "psat": [], "mspr": []}
    real = {"air": [], "psat": [], "mspr": []}
    for _, rel in SIM_LOGS:
        a, ws = _windows_for(os.path.join(os.path.dirname(HERE), rel))
        sim["air"].append(a)
        for s, m in ws:
            if np.isfinite(m["poutSat"]):
                sim["psat"].append(m["poutSat"]); sim["mspr"].append(m["mspread"])
    for k, fam, _ in LOGS:
        fam, d, segs, info = DATA[k]
        span = d["ct"][-1, 0] - d["ct"][0, 0]
        real["air"].append(100 * sum(s["dur"] for s in segs if s["kind"] == "AIRBORNE") / span)
        for s, m in active_windows(k, 1.0):
            if np.isfinite(m["poutSat"]):
                real["psat"].append(m["poutSat"]); real["mspr"].append(m["mspread"])
    fig, axs = plt.subplots(1, 3, figsize=(9.4, 3.4))
    # AIRBORNE %
    axs[0].bar(["SIM", "REAL"], [np.mean(sim["air"]), np.mean(real["air"])],
               color=["#2ca02c", "#d62728"])
    axs[0].set_ylabel("% of capture AIRBORNE"); axs[0].set_title("Sustained flight", fontsize=9)
    # pitch_out saturation
    axs[1].boxplot([sim["psat"], real["psat"]], labels=["SIM", "REAL"], widths=0.6)
    axs[1].set_ylabel("pitch_out saturation (%)"); axs[1].set_title("Controller railing", fontsize=9)
    # motor spread
    axs[2].boxplot([sim["mspr"], real["mspr"]], labels=["SIM", "REAL"], widths=0.6)
    axs[2].set_ylabel("motor differential (max−min)"); axs[2].set_title("Commanded differential", fontsize=9)
    fig.suptitle("Same firmware, different plant: SIM flies (no railing, tiny differential); "
                 "REAL saturates and limit-cycles", fontsize=9.5)
    fig.savefig(os.path.join(OUT, "08_sim_vs_real.png")); plt.close(fig)


for fn in (p00, p01, p02, p03, p04, p05, p06, p07, p08):
    try:
        fn(); print("ok", fn.__name__)
    except Exception as e:
        print("FAIL", fn.__name__, repr(e))
print("plots ->", OUT)
