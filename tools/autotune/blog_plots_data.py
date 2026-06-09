#!/usr/bin/env python3
"""Generate the data-driven figures for the PID-autotuning engineering blog.
Reads existing flight logs + autotune result JSON; no SITL launch. The SITL
step-response figures are produced separately by blog_plots_sitl.py."""
import json, os, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
LOGS = os.path.join(REPO, "logs")
OUT  = os.path.join(REPO, "docs", "blog", "pid-autotuning", "images")
os.makedirs(OUT, exist_ok=True)
sys.path.insert(0, os.path.join(REPO, "tools"))
import sim_log_plot as S

plt.rcParams.update({"figure.facecolor": "white", "axes.grid": True,
                     "grid.alpha": 0.3, "font.size": 10})

# ---- Fig: whole-system architecture (SITL stack) ------------------------------
def fig_system():
    from matplotlib.patches import FancyBboxPatch
    fig, ax = plt.subplots(figsize=(11, 5.6))
    ax.set_xlim(0, 100); ax.set_ylim(0, 56); ax.axis("off")

    def box(x, y, w, h, title, sub, fc):
        ax.add_patch(FancyBboxPatch((x, y), w, h,
                     boxstyle="round,pad=0.2,rounding_size=1.6",
                     linewidth=1.6, edgecolor="#2b3440", facecolor=fc))
        ax.text(x + w / 2, y + h - 3.2, title, ha="center", va="top",
                fontsize=10.5, fontweight="bold", zorder=5)
        ax.text(x + w / 2, y + h - 7.2, sub, ha="center", va="top",
                fontsize=8.3, zorder=5, color="#333")

    def arrow(x1, y1, x2, y2, text="", color="#2b3440", rad=0.0, dx=0, dy=1.6,
              fs=8.3, two=False):
        style = "<|-|>" if two else "-|>"
        ax.annotate("", xy=(x2, y2), xytext=(x1, y1),
                    arrowprops=dict(arrowstyle=style, color=color, lw=1.7,
                                    connectionstyle=f"arc3,rad={rad}"))
        if text:
            ax.text((x1 + x2) / 2 + dx, (y1 + y2) / 2 + dy, text, ha="center",
                    va="center", fontsize=fs, color=color)

    box(3,  8, 30, 17, "vsim_d  (physics daemon)",
        "rigid-body dynamics 8 kHz\nsynthetic IMU + mag 1 kHz", "#F6DCE0")
    box(40, 8, 32, 17, "Firmware",
        "same C as flight:\nMahony est · cascaded PID\nNavLink · VFS store", "#D9F2DD")
    box(40, 36, 32, 15, "Navigator GCS  (Qt6)",
        "tuning/telemetry UI · 3-D view\nlaunches the autotuner", "#D6E8FF")

    # vsim_d <-> firmware over FIFOs
    arrow(33, 19.5, 40, 19.5, "IMU (gyro/accel/mag)", color="#b0506a", dy=1.8)
    arrow(40, 13.5, 33, 13.5, "motor PWM", color="#b0506a", dy=-1.8)
    # pose -> GCS 3-D renderer
    arrow(20, 25, 41, 36, "pose 60–120 Hz\n→ 3-D view", color="#7a6fae",
          rad=0.18, dx=-3, dy=0)
    # NavLink telemetry/commands between firmware and GCS
    arrow(60, 25, 60, 36, "NavLink\ntelemetry ↑ / commands ↓", color="#3d7bd1",
          dx=11, dy=0, two=True)

    ax.text(50, 2.6,
            "Hardware: the same firmware runs on an STM32 (NavHAL); vsim_d is "
            "replaced by the real BMX160 IMU + ESCs, and the GCS connects over the "
            "radio link.",
            ha="center", va="center", fontsize=8.3, style="italic", color="#555")
    ax.set_title("Vayu SITL system: one firmware binary, simulated plant, live ground station",
                 fontsize=11)
    fig.tight_layout()
    png = os.path.join(OUT, "system.png"); fig.savefig(png, dpi=130); plt.close(fig)
    print("wrote", png)

# ---- Fig: cascaded control architecture (rendered, not ASCII) ------------------
def fig_architecture():
    from matplotlib.patches import FancyBboxPatch
    fig, ax = plt.subplots(figsize=(11, 4.4))
    ax.set_xlim(0, 100); ax.set_ylim(0, 44); ax.axis("off")

    def box(x, w, text, fc, y=26, h=11):
        ax.add_patch(FancyBboxPatch((x, y), w, h,
                     boxstyle="round,pad=0.2,rounding_size=1.4",
                     linewidth=1.5, edgecolor="#2b3440", facecolor=fc))
        ax.text(x + w / 2, y + h / 2, text, ha="center", va="center",
                fontsize=9.5, zorder=5)

    def arrow(x1, y1, x2, y2, text="", color="#2b3440", rad=0.0, ty=2.0):
        ax.annotate("", xy=(x2, y2), xytext=(x1, y1),
                    arrowprops=dict(arrowstyle="-|>", color=color, lw=1.7,
                                    connectionstyle=f"arc3,rad={rad}"))
        if text:
            ax.text((x1 + x2) / 2, (y1 + y2) / 2 + ty, text, ha="center",
                    fontsize=8.5, color=color)

    box(2,  13, "Stick\ncommand", "#EAEFF5")
    box(20, 16, "OUTER\nAngle PID", "#D6E8FF")
    box(43, 16, "INNER\nRate PID",  "#D9F2DD")
    box(66, 12, "Motor\nmix",       "#FBEED6")
    box(82, 15, "Motors +\nairframe", "#F6DCE0")
    box(20, 70, "Attitude estimate  —  Mahony filter (gyro + accel, mag-fused yaw)",
        "#EEE9F7", y=4, h=8)

    arrow(15, 31.5, 20, 31.5)
    arrow(36, 31.5, 43, 31.5, "rate\nsetpoint", ty=2.4)
    arrow(59, 31.5, 66, 31.5)
    arrow(78, 31.5, 82, 31.5)
    # plant output down to the estimator
    arrow(89, 26, 89, 12, color="#7a6fae")
    ax.text(90.5, 19, "IMU\n(gyro/accel/mag)", ha="left", va="center",
            fontsize=8, color="#7a6fae")
    # feedback up: attitude -> outer loop, body rate -> inner loop
    arrow(40, 12, 28, 26, "attitude θ", color="#3d7bd1", rad=-0.15, ty=-3)
    arrow(58, 12, 51, 26, "body rate ω", color="#3a9d4e", rad=0.15, ty=-3)

    ax.set_title("Cascaded attitude control: outer angle loop drives the inner rate loop",
                 fontsize=11)
    fig.tight_layout()
    png = os.path.join(OUT, "architecture.png"); fig.savefig(png, dpi=130); plt.close(fig)
    print("wrote", png)

# ---- Fig: real flight overview (use the long 322 s test with failsafe recovery)
def fig_flight():
    path = os.path.join(LOGS, "sim-2026-06-06_20-34-34.bin")
    b = S.load(path)
    png = os.path.join(OUT, "flight_overview.png")
    S.plot(b, png, False, "In-sim flight on the autotuned, mag-fused build — 46 s armed, single window, no failsafe")
    print("wrote", png)

# ---- Fig: throttle-coupled buzz onset (the limit cycle that the rig missed) ----
def fig_buzz_onset():
    b = S.load(os.path.join(LOGS, "sim-2026-06-06_06-03-31.bin"))
    pt = b.pid_t; thr = b.pid["thro_out"]; ro = b.pid["roll_out"]
    # 1 s buckets: mean throttle + roll-output chatter (mean |Δ| sample-to-sample)
    import collections
    bk = collections.defaultdict(lambda: {"thr": [], "ro": []})
    for i in range(len(pt)):
        s = int(pt[i]); bk[s]["thr"].append(thr[i]); bk[s]["ro"].append(ro[i])
    ts = sorted(k for k in bk if bk[k]["thr"])
    t0 = ts[0]
    T  = [k - t0 for k in ts]
    TH = [sum(bk[k]["thr"]) / len(bk[k]["thr"]) for k in ts]
    def chat(x): return sum(abs(x[i] - x[i-1]) for i in range(1, len(x))) / max(1, len(x))
    CH = [chat(bk[k]["ro"]) for k in ts]
    fig, ax1 = plt.subplots(figsize=(9, 4))
    ax1.plot(T, TH, color="#E5C07B", lw=2, label="throttle cmd (0..1)")
    ax1.axhline(0.33, color="#888", ls=":", lw=1)
    ax1.text(T[0], 0.34, "hover ≈ 0.33", color="#666", fontsize=8)
    ax1.set_xlabel("time since arm (s)"); ax1.set_ylabel("throttle"); ax1.set_ylim(0, 1)
    ax2 = ax1.twinx()
    ax2.plot(T, CH, color="#E06C75", lw=2, label="roll-output chatter")
    ax2.set_ylabel("roll PID output chatter  (mean |Δout|)", color="#E06C75")
    ax2.tick_params(axis="y", colors="#E06C75")
    ax1.set_title("Throttle-coupled limit cycle: chatter explodes the instant throttle reaches hover")
    fig.tight_layout(); png = os.path.join(OUT, "buzz_onset.png")
    fig.savefig(png, dpi=130); plt.close(fig); print("wrote", png)

# ---- Fig: autotune convergence (real structured run from the GCS) --------------
def fig_convergence():
    d = json.load(open(os.path.join(LOGS, "autotune_yaw.json")))
    r = d["results"][0]; base = d["baseline"]
    hist = r["history"]
    n   = [h[0] for h in hist]; cost = [h[1] for h in hist]; best = [h[2] for h in hist]
    fig, ax = plt.subplots(figsize=(9, 4))
    ax.axhline(base, color="#888", ls="--", lw=1.5, label=f"seed baseline ({base:.1f})")
    ax.plot(n, cost, "o-", color="#61AFEF", ms=4, lw=1, alpha=0.7, label="eval cost")
    ax.plot(n, best, color="#98C379", lw=2.5, label="best-so-far")
    ax.set_xlabel("evaluation #"); ax.set_ylabel("tracking cost (lower = better)")
    ax.set_title(f"Structured optimizer convergence — {r['optimizer']}, "
                 f"{base:.0f} → {r['best_cost']:.0f} ({100*(base-r['best_cost'])/base:.0f}% better)")
    ax.legend(); fig.tight_layout()
    png = os.path.join(OUT, "convergence.png"); fig.savefig(png, dpi=130); plt.close(fig)
    print("wrote", png)

# ---- Fig: optimizer comparison (from a --yaw --compare budget-20 run, logged) --
def fig_optimizers():
    # Final cost per optimizer (lower=better); from the documented --compare run.
    data = [("hybrid", 26.7), ("portfolio", 28.7), ("spsa", 29.3), ("fdgd", 34.0),
            ("random", 34.5), ("coordinate", 38.4), ("nelder-mead", 39.8)]
    names = [d[0] for d in data]; costs = [d[1] for d in data]
    colors = ["#98C379" if c <= 30 else ("#E5C07B" if c <= 35 else "#E06C75") for c in costs]
    fig, ax = plt.subplots(figsize=(9, 4))
    ax.bar(names, costs, color=colors)
    for i, c in enumerate(costs):
        ax.text(i, c + 0.3, f"{c:.1f}", ha="center", fontsize=9)
    ax.set_ylabel("best cost (lower = better)")
    ax.set_title("Optimizer shoot-out (--compare, budget 20, --yaw): explorers beat pure-local search")
    ax.set_ylim(0, 44); fig.tight_layout()
    png = os.path.join(OUT, "optimizers.png"); fig.savefig(png, dpi=130); plt.close(fig)
    print("wrote", png)

# ---- Fig: buzz penalty by gain set (validates the detector) --------------------
def fig_buzz_penalty():
    # Mean ± min/max over 4 sensor-noise seeds (the limit cycle is stochastic, so
    # a single rollout is noisy — see the spread on the rate-hot tunes).
    data = [  # label, mean, lo, hi
        ("0.0005 / kd 0\nangle_kp 4\n(autotuner pick)", 0.0, 0.0, 0.0),
        ("0.004 / kd 0\nangle_kp 3", 11.2, 7.6, 14.6),
        ("0.007 / kd 0\nangle_kp 2", 15.7, 14.1, 16.8),
        ("0.012 / kd 0.001\nangle_kp 2\n(manual, BUZZED)", 16.1, 7.5, 23.1),
    ]
    names = [d[0] for d in data]; pen = [d[1] for d in data]
    lo = [d[1] - d[2] for d in data]; hi = [d[3] - d[1] for d in data]
    colors = ["#98C379", "#B5C98A", "#E5C07B", "#E06C75"]
    fig, ax = plt.subplots(figsize=(9, 4))
    ax.bar(names, pen, color=colors, yerr=[lo, hi], capsize=5,
           error_kw=dict(ecolor="#555", lw=1.3))
    for i, p in enumerate(pen):
        ax.text(i, p + (hi[i] if hi[i] else 0) + 0.7, f"{p:.1f}", ha="center", fontsize=9)
    ax.set_ylabel("buzz penalty added to cost")
    ax.set_title("Throttle-sweep buzz detector: the buzz-free pick scores 0; any rate-hot tune pays\n(mean ± range over 4 seeds — the limit cycle is stochastic)")
    ax.set_ylim(0, 28); fig.tight_layout()
    png = os.path.join(OUT, "buzz_penalty.png"); fig.savefig(png, dpi=130); plt.close(fig)
    print("wrote", png)

def _white():
    # sim_log_plot.plot() globally switches rcParams to its dark report theme;
    # re-assert a clean white style for the analysis figures.
    plt.rcParams.update(plt.rcParamsDefault)
    plt.rcParams.update({"figure.facecolor": "white", "axes.grid": True,
                         "grid.alpha": 0.3, "font.size": 10})

if __name__ == "__main__":
    # Custom (white) figures first, flight overview (native dark report) last so
    # its global rcParams change can't bleed into the others.
    _white()
    fig_system()
    fig_architecture()
    fig_buzz_onset(); fig_convergence(); fig_optimizers(); fig_buzz_penalty()
    fig_flight()
    print("data figures done")
