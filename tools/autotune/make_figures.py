#!/usr/bin/env python3
"""Generate the case-study figures for sim/vsim/docs/reference/autotune-methodology.md
from real log data: optimizer convergence (autotune history JSON), cost noise +
stability knee (/tmp/case_study.json), and a real GCS flight log (.bin)."""
import json, os, sys, struct
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import protocol as P

OUT = os.path.join(os.path.dirname(__file__), "..", "..", "tools", "vsim", "docs", "reference", "figures")
os.makedirs(OUT, exist_ok=True)
GREEN, BLUE, RED, GRAY = "#2ca02c", "#1f77b4", "#d62728", "#888"


def fig_convergence(path="/tmp/autotune_yaw_compare2.json"):
    if not os.path.exists(path):
        return
    d = json.load(open(path))
    base = d["baseline"]
    plt.figure(figsize=(7.2, 4.0))
    for r in sorted(d["results"], key=lambda r: r["best_cost"]):
        hist = r.get("history", [])
        if not hist:
            continue
        xs = [h[0] for h in hist]
        best = [min(h[2], base * 1.3) for h in hist]   # clip divergence spikes
        lw = 2.4 if r["optimizer"] == "hybrid" else 1.2
        plt.plot(xs, best, lw=lw, label=f"{r['optimizer']} ({r['best_cost']:.0f})")
    plt.axhline(base, color=GRAY, ls=":", label=f"baseline {base:.0f}")
    plt.xlabel("evaluation"); plt.ylabel("best cost so far (lower=better)")
    plt.title("Optimizer convergence — 7 methods, equal budget (well-posed plant)")
    plt.legend(fontsize=7, ncol=2); plt.grid(alpha=0.3); plt.tight_layout()
    plt.savefig(os.path.join(OUT, "fig_convergence.png"), dpi=130); plt.close()
    print("wrote fig_convergence.png")


def fig_noise(path="/tmp/case_study.json"):
    if not os.path.exists(path):
        return
    n = json.load(open(path)).get("noise", [])
    if len(n) < 3:
        return
    DIV = 1e3
    stable = [v for v in n if v < DIV]
    ndiv = sum(1 for v in n if v >= DIV)
    plt.figure(figsize=(7.2, 3.8))
    xs = list(range(1, len(n) + 1))
    cols = [RED if v >= DIV else BLUE for v in n]
    plt.scatter(xs, n, s=50, c=cols, zorder=3)
    plt.axhline(DIV, color=RED, ls=":", lw=1.2)
    plt.text(0.6, DIV * 1.5, "divergence penalty (vehicle flipped)", color=RED, fontsize=8)
    if stable:
        import statistics as st
        plt.axhline(st.mean(stable), color=BLUE, lw=1.2,
                    label=f"stable runs ≈ {st.mean(stable):.0f}")
    plt.yscale("log"); plt.xlim(0.5, len(n) + 0.5)
    plt.xlabel("repeat # (IDENTICAL seed gains)"); plt.ylabel("rollout cost (log)")
    plt.title(f"Cost noise: same gains, {len(n)} runs — {ndiv}/{len(n)} DIVERGED "
              f"(seed is marginally stable)")
    plt.legend(fontsize=8, loc="center right"); plt.grid(alpha=0.3, which="both")
    plt.tight_layout()
    plt.savefig(os.path.join(OUT, "fig_noise.png"), dpi=130); plt.close()
    print("wrote fig_noise.png")


def fig_knee(path="/tmp/case_study.json"):
    if not os.path.exists(path):
        return
    k = json.load(open(path)).get("knee", [])
    if not k:
        return
    # Split armed-and-responsive (track > 1 deg) from no-response trials.
    arm_kp = [r["rate_kp"] for r in k if r["track"] > 1.0]
    arm_ch = [r["chatter"] for r in k if r["track"] > 1.0]
    bad_kp = [r["rate_kp"] for r in k if r["track"] <= 1.0]
    bad_ch = [r["chatter"] for r in k if r["track"] <= 1.0]
    plt.figure(figsize=(7.2, 4.0))
    plt.axhspan(0.12, 1.1, color=RED, alpha=0.07)
    plt.axhline(0.12, color=RED, ls=":", lw=0.9)
    if arm_kp:
        plt.scatter(arm_kp, arm_ch, s=70, color=RED, zorder=3,
                    label="responded → BUZZING (chatter ≫ calm)")
    if bad_kp:
        plt.scatter(bad_kp, bad_ch, s=70, marker="x", color=GRAY, zorder=3,
                    label="no response (failed to arm — marginal plant)")
    plt.text(min(r["rate_kp"] for r in k), 0.5, "buzz / limit-cycle band",
             color=RED, fontsize=8)
    plt.ylim(-0.05, 1.1); plt.xlabel("inner-loop rate_kp")
    plt.ylabel("hover output chatter")
    plt.title("No calm+responsive rate_kp on this vehicle: it either buzzes or stalls")
    plt.legend(fontsize=8, loc="center right"); plt.grid(alpha=0.3); plt.tight_layout()
    plt.savefig(os.path.join(OUT, "fig_knee.png"), dpi=130); plt.close()
    print("wrote fig_knee.png")


def fig_flight(path=None):
    # Optional flight overview. Point VAYU_FLIGHT_LOG at a capture, else look in
    # the repo's logs/; skip silently if nothing is there (no personal paths).
    _repo = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    path = path or os.environ.get(
        "VAYU_FLIGHT_LOG",
        os.path.join(_repo, "logs", "sim-2026-06-04_03-26-58.bin"))
    if not os.path.exists(path):
        return
    cts = P.NavlinkDecoder().feed(open(path, "rb").read())
    if len(cts) < 20:
        return
    t = [i / 18.0 for i in range(len(cts))]          # ~18 Hz control telemetry
    roll = [c["roll_angle_curr"] for c in cts]
    pitch = [c["pitch_angle_curr"] for c in cts]
    yaw = [c["yaw_angle_curr"] for c in cts]
    plt.figure(figsize=(7.2, 4.0))
    plt.axhspan(70, 200, color=RED, alpha=0.06); plt.axhspan(-200, -70, color=RED, alpha=0.06)
    plt.axhline(70, color=RED, ls=":", lw=0.8); plt.axhline(-70, color=RED, ls=":", lw=0.8)
    plt.plot(t, roll, color=RED, lw=1.0, label="roll")
    plt.plot(t, pitch, color=GREEN, lw=1.0, label="pitch")
    plt.plot(t, yaw, color=BLUE, lw=1.0, label="yaw")
    plt.ylim(-185, 185); plt.xlabel("time [s]"); plt.ylabel("attitude [deg]")
    plt.text(0.5, 150, "±70° bank-angle FAILSAFE cutoff", color=RED, fontsize=8)
    plt.title("Real GCS flight log: aggressive angle-mode flight → cutoff")
    plt.legend(fontsize=8, loc="lower right"); plt.grid(alpha=0.3); plt.tight_layout()
    plt.savefig(os.path.join(OUT, "fig_flight.png"), dpi=130); plt.close()
    print("wrote fig_flight.png")


if __name__ == "__main__":
    fig_convergence(); fig_noise(); fig_knee(); fig_flight()
    print("figures ->", os.path.abspath(OUT))
