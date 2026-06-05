"""Live matplotlib dashboard for the SITL PID autotuner.

Six panels, refreshed ~5 Hz from the Monitor + the SITL sample deque:
  - angle tracking  (roll/pitch/yaw setpoint vs measured, current rollout)
  - rate tracking   (roll/pitch/yaw rate setpoint vs measured)
  - controller outputs (roll/pitch/yaw + throttle)
  - cost convergence (per-eval cost + best-so-far + baseline, by optimizer)
  - gains / status text

Runs in the MAIN thread (matplotlib requirement); the optimization runs in a
worker. Closing the window sets monitor.stop so the worker unwinds.
"""

import os
import matplotlib
if not os.environ.get("MPLBACKEND"):       # allow override (e.g. Agg for tests)
    matplotlib.use("QtAgg")
import matplotlib.pyplot as plt          # noqa: E402

AXES = ["roll", "pitch", "yaw"]
COLORS = {"roll": "tab:red", "pitch": "tab:green", "yaw": "tab:blue"}
WINDOW_S = 5.0                            # live trace window


def _recent(samples, span):
    if not samples:
        return []
    t_end = samples[-1][0]
    return [(t - t_end, ct) for t, ct in samples if t >= t_end - span]


def run(monitor, interval_ms=200):
    fig = plt.figure(figsize=(15, 8))
    fig.canvas.manager.set_window_title("Vayu PID Autotuner — live")
    gs = fig.add_gridspec(2, 3, hspace=0.32, wspace=0.22)
    ax_ang = fig.add_subplot(gs[0, 0])
    ax_rate = fig.add_subplot(gs[0, 1])
    ax_out = fig.add_subplot(gs[0, 2])
    ax_cost = fig.add_subplot(gs[1, 0:2])
    ax_txt = fig.add_subplot(gs[1, 2]); ax_txt.axis("off")

    def on_close(_):
        monitor.stop.set()
    fig.canvas.mpl_connect("close_event", on_close)

    def draw(_frame):
        samples = monitor.stack.snapshot()
        st = monitor.state()
        win = _recent(samples, WINDOW_S)

        # --- angle tracking ---
        ax_ang.clear()
        ax_ang.set_title("attitude: setpoint (dashed) vs measured [deg]", fontsize=9)
        ax_ang.set_ylabel("deg")
        if win:
            ts = [t for t, _ in win]
            for a in AXES:
                ax_ang.plot(ts, [c[f"{a}_angle_sp"] for _, c in win],
                            "--", color=COLORS[a], lw=1.0, alpha=0.7)
                ax_ang.plot(ts, [c[f"{a}_angle_curr"] for _, c in win],
                            "-", color=COLORS[a], lw=1.5, label=a)
            ax_ang.legend(loc="upper left", fontsize=7, ncol=3)
        ax_ang.grid(alpha=0.25)

        # --- rate tracking ---
        ax_rate.clear()
        ax_rate.set_title("body rates: sp (dashed) vs measured [deg/s]", fontsize=9)
        ax_rate.set_ylabel("deg/s")
        if win:
            ts = [t for t, _ in win]
            for a in AXES:
                ax_rate.plot(ts, [c[f"{a}_rate_sp"] for _, c in win],
                             "--", color=COLORS[a], lw=1.0, alpha=0.7)
                ax_rate.plot(ts, [c[f"{a}_rate_curr"] for _, c in win],
                             "-", color=COLORS[a], lw=1.2)
        ax_rate.grid(alpha=0.25)

        # --- outputs ---
        ax_out.clear()
        ax_out.set_title("controller outputs", fontsize=9)
        ax_out.set_xlabel("t [s]")
        if win:
            ts = [t for t, _ in win]
            for a in AXES:
                ax_out.plot(ts, [c[f"{a}_out"] for _, c in win],
                            color=COLORS[a], lw=1.2, label=a)
            ax_out.plot(ts, [c["thro_out"] for _, c in win], color="black",
                        lw=1.0, label="thr")
            ax_out.legend(loc="upper left", fontsize=7, ncol=4)
        ax_out.set_ylim(-1.1, 1.1)
        ax_out.grid(alpha=0.25)

        # --- cost convergence ---
        ax_cost.clear()
        ax_cost.set_title("cost per evaluation (lower = better)", fontsize=9)
        ax_cost.set_xlabel("evaluation"); ax_cost.set_ylabel("cost")
        evals = st["evals"]
        if evals:
            opts = {}
            for n, c, o in evals:
                opts.setdefault(o, ([], []))
                opts[o][0].append(n); opts[o][1].append(c)
            finite = [c for _, c, _ in evals if c < 1e5]
            cap = (max(finite) * 1.15) if finite else 1.0
            for o, (xs, ys) in opts.items():
                ys_c = [min(y, cap) for y in ys]
                ax_cost.scatter(xs, ys_c, s=18, label=o, alpha=0.8)
            # best-so-far step
            best = []
            b = float("inf")
            for n, c, _ in evals:
                b = min(b, c); best.append(min(b, cap))
            ax_cost.plot([n for n, _, _ in evals], best, "k-", lw=1.2, label="best")
            if st["baseline"] is not None:
                ax_cost.axhline(min(st["baseline"], cap), color="gray",
                                ls=":", lw=1.0, label="baseline")
            ax_cost.set_ylim(0, cap)
            ax_cost.legend(loc="upper right", fontsize=7, ncol=2)
        ax_cost.grid(alpha=0.25)

        # --- status / gains ---
        ax_txt.clear(); ax_txt.axis("off")
        lines = []
        status = "DONE" if st["done"] else f"running: {st['optimizer']}"
        lines.append(f"status: {status}")
        lines.append(f"evals:  {len(evals)}")
        if st["baseline"] is not None:
            lines.append(f"baseline cost: {st['baseline']:.2f}")
        if st["best_cost"] < 1e5:
            imp = ((st["baseline"] - st["best_cost"]) / st["baseline"] * 100
                   if st["baseline"] else 0)
            lines.append(f"best cost: {st['best_cost']:.2f}  ({imp:+.0f}%)  [{st['best_opt']}]")
        lines.append("")
        names = st["names"]
        cur = st["current_x"]; best = st["best_x"]
        lines.append(f"{'param':<14}{'current':>10}{'best':>11}")
        for i, nm in enumerate(names):
            cv = f"{cur[i]:.4g}" if cur else "-"
            bv = f"{best[i]:.4g}" if best else "-"
            lines.append(f"{nm:<14}{cv:>10}{bv:>11}")
        ax_txt.text(0.0, 1.0, "\n".join(lines), va="top", ha="left",
                    family="monospace", fontsize=9, transform=ax_txt.transAxes)

    # keep a ref so the animation isn't garbage-collected
    from matplotlib.animation import FuncAnimation
    run._anim = FuncAnimation(fig, draw, interval=interval_ms, cache_frame_data=False)
    plt.show()
