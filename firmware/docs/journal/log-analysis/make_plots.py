#!/usr/bin/env python3
"""Generate analysis plots for a recorded NavLink v2 session.

Reuses the decoder in parse_log.py. Writes PNGs into <archive>/plots/.

Usage (from repo root):
    python3 docs/log-analysis/make_plots.py docs/log-analysis/20260617-124210
    # expects <dir>/export-*.bin inside it
"""
import importlib.util
import glob
import math
import os
import sys
import bisect
import statistics as st

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("pl", os.path.join(HERE, "parse_log.py"))
pl = importlib.util.module_from_spec(spec); spec.loader.exec_module(pl)
nl = pl.nl

plt.rcParams.update({"figure.dpi": 110, "font.size": 9, "axes.grid": True,
                     "grid.alpha": 0.3, "figure.autolayout": True})


def load(archive):
    bins = glob.glob(os.path.join(archive, "export-*.bin"))
    if not bins:
        raise SystemExit(f"no export-*.bin in {archive}")
    _, recs = pl.read_records(bins[0])
    col = pl.Collector().run(recs)
    T0 = min(t for lst in col.msgs.values() for (t, _, _) in lst)
    return col, T0, recs


def main():
    archive = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "20260617-124210")
    outdir = os.path.join(archive, "plots")
    os.makedirs(outdir, exist_ok=True)
    col, T0, recs = load(archive)
    rel = lambda t: (t - T0) / 1e6

    # armed intervals + helper
    hb = [(t, m.nav_state) for t, m, _ in col.msgs["Heartbeat"]]
    hbt = [t for t, _ in hb]
    nav = lambda t: hb[max(bisect.bisect_right(hbt, t) - 1, 0)][1]
    ARM = nl.NavState.ARMED.value
    armed_iv = []
    s = None
    for t, st_ in hb:
        if st_ == ARM and s is None: s = t
        elif st_ != ARM and s is not None: armed_iv.append((s, t)); s = None
    if s: armed_iv.append((s, hb[-1][0]))

    def shade_armed(ax):
        for a, b in armed_iv:
            ax.axvspan(rel(a), rel(b), color="green", alpha=0.07,
                       label="_armed")

    saved = []
    def save(fig, name):
        p = os.path.join(outdir, name)
        fig.savefig(p, bbox_inches="tight"); plt.close(fig); saved.append(name)

    R = 180 / math.pi
    att = [(t, m) for t, m, _ in col.msgs["AttitudeEuler"]]
    ct = col.msgs["ControlTrace"]
    ctT = [t for t, _, _ in ct]
    ctrl_at = lambda t: ct[min(bisect.bisect_left(ctT, t), len(ct) - 1)][1]

    # ---- 1. Session overview: nav_state + throttle ----
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(10, 4.5), sharex=True)
    names = {s.value: s.name for s in nl.NavState}
    a1.step([rel(t) for t in hbt], [m for _, m in hb], where="post", lw=1.2)
    a1.set_yticks(sorted(set(m for _, m in hb)))
    a1.set_yticklabels([names.get(v, v) for v in sorted(set(m for _, m in hb))])
    a1.set_title("Flight state and throttle"); a1.set_ylabel("nav_state")
    a2.plot([rel(t) for t, _, _ in ct], [m.thro_out for _, m, _ in ct], lw=0.5, color="C1")
    a2.axhline(0.30, color="r", ls="--", lw=0.8, label="PID full-authority (0.30)")
    a2.axhline(0.10, color="orange", ls=":", lw=0.8, label="min armed (0.10)")
    a2.set_ylabel("throttle"); a2.set_xlabel("t (s)"); a2.legend(loc="upper right", fontsize=7)
    shade_armed(a2)
    save(fig, "01_session_overview.png")

    # ---- 2. Attitude: fused roll/pitch/yaw over time ----
    fig, ax = plt.subplots(figsize=(10, 3.2))
    ax.plot([rel(t) for t, _ in att], [m.roll * R for _, m in att], lw=0.6, label="roll")
    ax.plot([rel(t) for t, _ in att], [m.pitch * R for _, m in att], lw=0.6, label="pitch")
    ax.plot([rel(t) for t, _ in att], [m.yaw * R for _, m in att], lw=0.4, label="yaw", alpha=0.6)
    ax.axhline(0, color="k", lw=0.5)
    ax.set_title("Fused attitude (EKF) — note sustained +pitch tilt")
    ax.set_ylabel("deg"); ax.set_xlabel("t (s)"); ax.legend(loc="upper right", fontsize=7)
    shade_armed(ax)
    save(fig, "02_attitude.png")

    # ---- 3. Throttle-only tilt: pitch_curr vs setpoint ----
    to = [(rel(t), m) for t, m, _ in ct
          if nav(t) == ARM and abs(m.roll_angle_sp) < 2 and abs(m.pitch_angle_sp) < 2 and m.thro_out > 0.1]
    fig, ax = plt.subplots(figsize=(10, 3.0))
    ax.plot([x for x, _ in to], [m.pitch_angle_curr for _, m in to], ".", ms=2, label="pitch actual")
    ax.plot([x for x, _ in to], [m.roll_angle_curr for _, m in to], ".", ms=2, label="roll actual", alpha=0.6)
    ax.axhline(0, color="r", lw=1, label="commanded (level)")
    ax.axhline(st.mean(m.pitch_angle_curr for _, m in to), color="C0", ls="--", lw=0.8,
               label=f"pitch mean {st.mean(m.pitch_angle_curr for _, m in to):+.0f}°")
    ax.set_title("Throttle-only, level command: frame sits tilted (real)")
    ax.set_ylabel("deg"); ax.set_xlabel("t (s)"); ax.legend(loc="upper right", fontsize=7)
    save(fig, "03_throttle_only_tilt.png")

    # ---- 4. Fusion vs accel tilt cross-check ----
    # level gravity unit from frames EKF believed level
    attt = [t for t, _ in att]
    fused_at = lambda t: att[min(bisect.bisect_left(attt, t), len(att) - 1)][1]
    ir = col.msgs["ImuRaw"]
    lvl = [m.acc for t, m, _ in ir if abs(fused_at(t).roll * R) < 5 and abs(fused_at(t).pitch * R) < 5]
    g = np.array([st.mean(a[i] for a in lvl) for i in range(3)]); g /= np.linalg.norm(g)
    xs, ftilt, atilt = [], [], []
    for t, m, _ in ir:
        if nav(t) != ARM: continue
        a = np.array(m.acc)
        if np.linalg.norm(a) < 1: continue
        a = a / np.linalg.norm(a)
        at = math.degrees(math.acos(max(-1, min(1, a.dot(g)))))
        f = fused_at(t)
        ftn = math.degrees(math.acos(max(-1, min(1, math.cos(f.roll) * math.cos(f.pitch)))))
        xs.append(rel(t)); atilt.append(at); ftilt.append(ftn)
    fig, ax = plt.subplots(figsize=(10, 3.0))
    ax.plot(xs, atilt, ".", ms=3, alpha=0.5, label="accel-derived tilt")
    ax.plot(xs, ftilt, ".", ms=3, alpha=0.7, label="fused (EKF) tilt")
    ax.set_title(f"Fusion tracks gravity: tilt is real (mean Δ={st.mean(f-a for f,a in zip(ftilt,atilt)):+.1f}°)")
    ax.set_ylabel("tilt from level (deg)"); ax.set_xlabel("t (s)"); ax.legend(fontsize=7)
    save(fig, "04_fusion_vs_accel_tilt.png")

    # ---- 5. Yaw spin episodes ----
    yt = [(rel(t), m.yaw_rate_curr, m.yaw_out) for t, m, _ in ct if nav(t) == ARM]
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(10, 4.2), sharex=True)
    a1.plot([x[0] for x in yt], [x[1] for x in yt], lw=0.5)
    a1.axhline(0, color="k", lw=0.5)
    a1.set_title("Yaw: disturbance rate (cmd=0 throughout) and controller output")
    a1.set_ylabel("yaw_rate_curr (°/s)")
    a2.plot([x[0] for x in yt], [x[2] for x in yt], lw=0.5, color="C3")
    a2.axhline(1, color="r", ls="--", lw=0.6); a2.axhline(-1, color="r", ls="--", lw=0.6)
    a2.set_ylabel("yaw_out (±1 sat)"); a2.set_xlabel("t (s)")
    a2.axvspan(343, 351, color="purple", alpha=0.15, label="343–351 s alt. spins")
    a2.legend(fontsize=7, loc="upper left")
    save(fig, "05_yaw_spins.png")

    # ---- 6. Rate-loop authority: output vs error (roll/pitch vs yaw) ----
    arm = [m for t, m, _ in ct if nav(t) == ARM]
    fig, ax = plt.subplots(figsize=(6.5, 4.2))
    ax.scatter([m.pitch_rate_sp - m.pitch_rate_curr for m in arm], [m.pitch_out for m in arm],
               s=2, alpha=0.3, label="pitch (Kp 5e-4)")
    ax.scatter([m.roll_rate_sp - m.roll_rate_curr for m in arm], [m.roll_out for m in arm],
               s=2, alpha=0.3, label="roll (Kp 5e-4)")
    ax.scatter([m.yaw_rate_sp - m.yaw_rate_curr for m in arm], [m.yaw_out for m in arm],
               s=2, alpha=0.3, color="C3", label="yaw (Kp 1.8e-2, 36×)")
    ax.set_title("Rate-loop authority: output vs rate error")
    ax.set_xlabel("rate error (°/s)"); ax.set_ylabel("controller out (±1)")
    ax.legend(fontsize=7); ax.axhline(0, color="k", lw=0.4); ax.axvline(0, color="k", lw=0.4)
    save(fig, "06_rate_authority.png")

    # ---- 11. Attitude-loop oscillation (episode 1 zoom) ----
    seg = [(rel(t), m) for t, m, _ in ct if 45 <= rel(t) <= 132]
    if seg:
        tx = [t for t, _ in seg]
        fig, (a1, a2, a3) = plt.subplots(3, 1, figsize=(10, 5.2), sharex=True)
        a1.plot(tx, [m.thro_out for _, m in seg], color="C1")
        a1.axhline(0.30, color="r", ls="--", lw=0.8, label="full authority 0.30")
        a1.set_ylabel("throttle"); a1.legend(fontsize=7, loc="upper right")
        a1.set_title("Attitude-loop oscillation (armed window 1): grows under sustained throttle (~0.30), "
                     "pilot pulls throttle near the end")
        a2.plot(tx, [m.roll_angle_curr for _, m in seg], label="roll", lw=0.8)
        a2.plot(tx, [m.pitch_angle_curr for _, m in seg], label="pitch", lw=0.8, alpha=0.8)
        a2.axhline(0, color="k", lw=0.5); a2.set_ylabel("angle (°)")
        a2.legend(fontsize=7, loc="upper right")
        a3.plot(tx, [m.roll_out for _, m in seg], color="C3", lw=0.8, label="roll_out")
        a3.axhline(0, color="k", lw=0.5)
        a3.set_ylabel("roll_out"); a3.set_xlabel("t (s)")
        a3.legend(fontsize=7, loc="upper right")
        a3.text(0.01, 0.05, "controller output anti-phase with angle (corr −0.82): closed-loop, not passive swing",
                transform=a3.transAxes, fontsize=7, color="#555")
        save(fig, "11_oscillation.png")

    # ---- 7. Motor balance (throttle-only) + over time ----
    mt = col.msgs["MotorTelemetry"]
    rows = [m.cmd[:4] for t, m, _ in mt if nav(t) == ARM
            and abs(ctrl_at(t).roll_angle_sp) < 2 and abs(ctrl_at(t).pitch_angle_sp) < 2
            and ctrl_at(t).thro_out > 0.1]
    means = [st.mean(r[i] for r in rows) for i in range(4)]
    labels = ["M1 FR\nCW", "M2 RR\nCCW", "M3 RL\nCW", "M4 FL\nCCW"]
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(10, 3.4))
    bars = a1.bar(labels, means, color=["C0", "C1", "C2", "C3"])
    avg = sum(means) / 4
    a1.axhline(avg, color="k", ls="--", lw=0.8, label=f"avg {avg:.2f}")
    for b, m in zip(bars, means):
        a1.text(b.get_x() + b.get_width() / 2, m, f"{100*(m-avg)/avg:+.0f}%", ha="center", va="bottom", fontsize=8)
    a1.set_title("Motor mean, throttle-only level cmd"); a1.set_ylabel("cmd (0..1)"); a1.legend(fontsize=7)
    for i in range(4):
        a2.plot([rel(t) for t, _, _ in mt], [m.cmd[i] for _, m, _ in mt], lw=0.3, label=f"M{i+1}")
    a2.set_title("Motor commands over session"); a2.set_ylabel("cmd"); a2.set_xlabel("t (s)")
    a2.legend(fontsize=7, ncol=4); shade_armed(a2)
    save(fig, "07_motors.png")

    # ---- 8. Sensor magnitudes + accel scale ----
    accm = [np.linalg.norm(m.acc) for _, m, _ in ir]
    magm = [np.linalg.norm(m.mag) for _, m, _ in ir]
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(10, 3.4))
    a1.hist(accm, bins=40, color="C0", alpha=0.8)
    a1.axvline(9.807, color="r", lw=1.2, label="g = 9.807")
    a1.axvline(st.mean(accm), color="k", ls="--", lw=0.9, label=f"mean {st.mean(accm):.2f}")
    a1.set_title("|accel| — scale error vs g"); a1.set_xlabel("m/s²"); a1.legend(fontsize=7)
    a2.hist(magm, bins=40, color="C2", alpha=0.8)
    a2.set_title("|mag| — should be constant if calibrated (it isn't)")
    a2.set_xlabel("µT"); a2.axvline(st.mean(magm), color="k", ls="--", lw=0.9,
                                    label=f"{min(magm):.0f}–{max(magm):.0f}")
    a2.legend(fontsize=7)
    save(fig, "08_sensor_magnitudes.png")

    # ---- 9. Mag sphere (uncalibrated) ----
    mx = [m.mag[0] for _, m, _ in ir]; my = [m.mag[1] for _, m, _ in ir]; mz = [m.mag[2] for _, m, _ in ir]
    fig, axs = plt.subplots(1, 3, figsize=(10, 3.4))
    for ax, (u, v, lu, lv) in zip(axs, [(mx, my, "x", "y"), (mx, mz, "x", "z"), (my, mz, "y", "z")]):
        ax.scatter(u, v, s=2, alpha=0.3)
        ax.axhline(0, color="k", lw=0.4); ax.axvline(0, color="k", lw=0.4)
        ax.set_aspect("equal"); ax.set_xlabel(f"mag {lu}"); ax.set_ylabel(f"mag {lv}")
    fig.suptitle("Magnetometer scatter — off-center, non-spherical = uncalibrated", y=1.02)
    save(fig, "09_mag_sphere.png")

    # ---- 10. Kernel: CPU load + FIFO drops ----
    pg = col.msgs["PerfGlobal"]; M = 1 << 32
    d = lambda a, b: (b - a) % M
    cx, cy = [], []
    for i in range(1, len(pg)):
        a, b = pg[i - 1][1], pg[i][1]
        dc = d(a.cpu_cycles_lo, b.cpu_cycles_lo); di = d(a.idle_cycles_lo, b.idle_cycles_lo)
        if dc > 0 and di <= dc:
            cx.append(rel(pg[i][0])); cy.append(100 * (1 - di / dc))
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(10, 3.4))
    a1.plot(cx, cy, lw=0.8); a1.set_ylim(0, 100)
    a1.set_title(f"CPU load (mean {st.mean(cy):.0f}%)"); a1.set_ylabel("%"); a1.set_xlabel("t (s)")
    pf = col.msgs["PerfFifo"]; lastf = pf[-1][1].seq
    fr = sorted([m for _, m, _ in pf if m.seq == lastf], key=lambda x: x.fifo_id)
    fnames = [nl.PerfFifoId(m.fifo_id).name if m.fifo_id in nl.PerfFifoId._value2member_map_ else str(m.fifo_id) for m in fr]
    drops = [m.drops for m in fr]
    cols = ["C3" if "TELEMETRY" in n else "C2" for n in fnames]
    a2.barh(fnames, drops, color=cols)
    a2.set_title("FIFO drops (red=telemetry, green=control)")
    a2.set_xlabel("drops (u16 saturates at 65535)")
    save(fig, "10_kernel.png")

    print(f"wrote {len(saved)} plots to {outdir}/:")
    for n in saved:
        print(" ", n)


if __name__ == "__main__":
    main()
