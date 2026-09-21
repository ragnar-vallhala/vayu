#!/usr/bin/env python3
# Copyright (C) 2026 NAVRobotec Pvt Ltd
# Author: Ragnar Vallhala
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
sim_log_plot.py - PID tuning analysis report from a vayu sim log.

Picks the newest .bin in <repo>/logs/ (or a path you pass) and produces
a single multi-panel matplotlib figure focused on PID tuning:

  Row 1   state timeline (background colored by state for every plot)
  Row 2   pilot throttle + 4 motor outputs, with saturation marks
  Row 3   angle setpoint vs current   - roll | pitch | yaw
  Row 4   rate  setpoint vs current   - roll | pitch | yaw  (degrees/s)
  Row 5   PID outputs                 - roll | pitch | yaw  (-1..+1)
  Row 6   loop dt + gyro magnitude

The figure is written next to the log as <stem>.pdf (and shown if --show),
and a textual summary of the armed-window metrics is printed to stdout:

  - time spent armed / failsafe
  - motor saturation rate (any motor pinned at 0 or 1)
  - RMS + peak attitude error per axis
  - RMS + peak rate error per axis
  - peak gyro per axis
  - mean outer / inner loop dt

Usage:
  ./tools/analysis/sim_log_plot.py                   # newest logs/sim-*.bin
  ./tools/analysis/sim_log_plot.py <log.bin>
  ./tools/analysis/sim_log_plot.py --logs-dir DIR    # different logs root
  ./tools/analysis/sim_log_plot.py --show            # open the figure interactively
"""

from __future__ import annotations

import argparse
import math
import os
import struct
import sys
from collections import defaultdict
from dataclasses import dataclass, field

# Share the parser + constants with sim_log_to_csv. Same dir, ordinary
# import: parse_frames yields (idx, type, t_ms, dev_id, payload).
HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
from sim_log_to_csv import (    # noqa: E402
    parse_frames,
    fp16_to_f32,
    PACKET_IMU_DATA_FULL, PACKET_IMU_COMPRESSED, PACKET_ATTITUDE,
    PACKET_RC_CHANNELS, PACKET_MOTOR_TELEMETRY, PACKET_SYSTEM_STATUS,
    ORIGIN_SYS_STATE, ORIGIN_PID_ERROR, STATE_NAMES,
)

try:
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    from matplotlib.gridspec import GridSpec
except ImportError as e:
    sys.stderr.write(
        "sim_log_plot.py needs matplotlib. Install:\n"
        "  pip install matplotlib\n")
    raise

# Where the PID telemetry's 18 floats live in the SYSTEM_STATUS payload.
PID_FIELDS = [
    "roll_angle_sp",  "pitch_angle_sp",  "yaw_angle_sp",
    "roll_angle_cur", "pitch_angle_cur", "yaw_angle_cur",
    "roll_rate_sp",   "pitch_rate_sp",   "yaw_rate_sp",
    "roll_rate_cur",  "pitch_rate_cur",  "yaw_rate_cur",
    "roll_out",       "pitch_out",       "yaw_out",
    "thro_out",
    "outer_dt",       "inner_dt",
]

# Background tint per state. Anything we don't list defaults to no shade.
STATE_TINTS = {
    "STANDBY":      (0.20, 0.22, 0.28, 0.25),
    "PREARM":       (0.30, 0.30, 0.10, 0.25),
    "ARMED":        (0.10, 0.35, 0.10, 0.25),
    "IN_AIR":       (0.05, 0.45, 0.10, 0.30),
    "FAILSAFE":     (0.50, 0.10, 0.10, 0.30),
    "TERMINATED":   (0.10, 0.10, 0.10, 0.35),
}


# --- data containers ------------------------------------------------------

@dataclass
class Series:
    """One axis of a uniformly-named per-channel time series."""
    t: list[float] = field(default_factory=list)        # seconds
    v: list[float] = field(default_factory=list)

    def add(self, t: float, v: float) -> None:
        self.t.append(t); self.v.append(v)


@dataclass
class LogBundle:
    state_t: list[float] = field(default_factory=list)
    state_n: list[str]   = field(default_factory=list)

    # IMU (reconstructed)
    imu_t:   list[float] = field(default_factory=list)
    imu_acc: list[tuple[float,float,float]] = field(default_factory=list)
    imu_gyr: list[tuple[float,float,float]] = field(default_factory=list)

    # Attitude packets (mahony output)
    att_t:   list[float] = field(default_factory=list)
    att_rpy: list[tuple[float,float,float]] = field(default_factory=list)

    # Motor packets
    mot_t:  list[float] = field(default_factory=list)
    mot_m:  list[tuple[float,float,float,float]] = field(default_factory=list)

    # PID telemetry (18 floats per packet, sampled at ~control rate)
    pid_t: list[float] = field(default_factory=list)
    pid:   dict[str, list[float]] = field(default_factory=lambda:
              {k: [] for k in PID_FIELDS})

    def duration_s(self) -> float:
        ends = [s[-1] for s in (self.state_t, self.imu_t, self.att_t,
                                self.mot_t, self.pid_t) if s]
        starts = [s[0] for s in (self.state_t, self.imu_t, self.att_t,
                                 self.mot_t, self.pid_t) if s]
        if not ends or not starts:
            return 0.0
        return max(ends) - min(starts)


def load(path: str) -> LogBundle:
    """Parse a sim log into a structured bundle."""
    raw = open(path, "rb").read()
    b = LogBundle()
    imu_state = None     # absolute 10-float vector from latest FULL keyframe
    for _idx, ptype, t_ms, _dev, payload in parse_frames(raw):
        t_s = t_ms / 1000.0
        if ptype == PACKET_IMU_DATA_FULL and len(payload) == 40:
            imu_state = list(struct.unpack("<10f", payload))
            b.imu_t.append(t_s)
            b.imu_acc.append((imu_state[0], imu_state[1], imu_state[2]))
            b.imu_gyr.append((imu_state[3], imu_state[4], imu_state[5]))
        elif ptype == PACKET_IMU_COMPRESSED and len(payload) == 20:
            if imu_state is None:
                continue
            ds = struct.unpack("<10H", payload)
            for i, d in enumerate(ds):
                imu_state[i] += fp16_to_f32(d)
            b.imu_t.append(t_s)
            b.imu_acc.append((imu_state[0], imu_state[1], imu_state[2]))
            b.imu_gyr.append((imu_state[3], imu_state[4], imu_state[5]))
        elif ptype == PACKET_ATTITUDE and len(payload) == 12:
            r, p, y = struct.unpack("<3f", payload)
            b.att_t.append(t_s); b.att_rpy.append((r, p, y))
        elif ptype == PACKET_MOTOR_TELEMETRY and len(payload) == 16:
            m = struct.unpack("<4f", payload)
            b.mot_t.append(t_s); b.mot_m.append(m)
        elif ptype == PACKET_SYSTEM_STATUS:
            if len(payload) == 6 and payload[0] == ORIGIN_SYS_STATE:
                s = int(struct.unpack("<f", payload[2:])[0])
                b.state_t.append(t_s)
                b.state_n.append(STATE_NAMES.get(s, f"0x{s:X}"))
            elif len(payload) == 74 and payload[0] == ORIGIN_PID_ERROR:
                vals = struct.unpack("<18f", payload[2:])
                b.pid_t.append(t_s)
                for k, v in zip(PID_FIELDS, vals):
                    b.pid[k].append(v)
    return b


# --- analysis helpers -----------------------------------------------------

def state_ranges(b: LogBundle) -> list[tuple[float, float, str]]:
    """[(t0, t1, name), ...] runs of constant state."""
    out: list[tuple[float, float, str]] = []
    if not b.state_t:
        return out
    cur_t = b.state_t[0]
    cur_n = b.state_n[0]
    for t, n in zip(b.state_t[1:], b.state_n[1:]):
        if n != cur_n:
            out.append((cur_t, t, cur_n))
            cur_t, cur_n = t, n
    # Close the last range at the end of any timeline we have.
    last = max((s[-1] for s in (b.state_t, b.imu_t, b.att_t, b.mot_t,
                                b.pid_t) if s), default=cur_t)
    out.append((cur_t, last, cur_n))
    return out


def armed_windows(ranges) -> list[tuple[float, float]]:
    return [(t0, t1) for t0, t1, n in ranges if n == "ARMED"]


def in_any(t: float, windows) -> bool:
    for t0, t1 in windows:
        if t0 <= t <= t1:
            return True
    return False


def shade_states(ax, ranges):
    for t0, t1, n in ranges:
        c = STATE_TINTS.get(n)
        if c is None: continue
        ax.axvspan(t0, t1, facecolor=c[:3], alpha=c[3], zorder=0)


def rms(xs):
    if not xs: return float("nan")
    return math.sqrt(sum(x*x for x in xs) / len(xs))


def saturation_fraction(motor_m: list[tuple], thresh_low=0.005, thresh_hi=0.995) -> float:
    """Fraction of motor frames where ANY motor is pinned at 0 or 1."""
    if not motor_m: return 0.0
    n_sat = sum(1 for m in motor_m
                if min(m) <= thresh_low or max(m) >= thresh_hi)
    return n_sat / len(motor_m)


# --- plotting -------------------------------------------------------------

def plot(b: LogBundle, out_path: str | None, show: bool, title: str) -> None:
    fig = plt.figure(figsize=(16, 18))
    fig.patch.set_facecolor("#1B1F23")
    plt.rcParams.update({
        "axes.facecolor":  "#21252B",
        "axes.edgecolor":  "#3E4452",
        "axes.labelcolor": "#DCDFE4",
        "xtick.color":     "#ABB2BF",
        "ytick.color":     "#ABB2BF",
        "text.color":      "#DCDFE4",
        "grid.color":      "#3E4452",
        "grid.alpha":      0.4,
        "axes.titleweight": "bold",
        "font.size":       9,
    })

    gs = GridSpec(6, 3, figure=fig,
                  height_ratios=[0.6, 1.4, 1.6, 1.6, 1.6, 1.2],
                  hspace=0.45, wspace=0.22,
                  left=0.06, right=0.98, top=0.95, bottom=0.04)

    ranges = state_ranges(b)
    armed = armed_windows(ranges)

    # ---- Row 1: state timeline ----
    ax_state = fig.add_subplot(gs[0, :])
    ax_state.set_title(title, color="#61AFEF", fontsize=12, loc="left")
    ax_state.set_yticks([]); ax_state.set_ylim(0, 1)
    seen = []
    for t0, t1, n in ranges:
        c = STATE_TINTS.get(n, (0.5, 0.5, 0.5, 0.4))
        ax_state.axvspan(t0, t1, facecolor=c[:3], alpha=c[3])
        ax_state.text((t0 + t1) / 2, 0.5, n, ha="center", va="center",
                      fontsize=8, color="#DCDFE4")
        if n not in seen: seen.append(n)
    ax_state.set_xlabel("t (s)")
    ax_state.set_xlim(left=min(b.state_t, default=0))

    # ---- Row 2: throttle + motors ----
    ax_thr = fig.add_subplot(gs[1, :], sharex=ax_state)
    shade_states(ax_thr, ranges)
    if b.pid_t:
        ax_thr.plot(b.pid_t, b.pid["thro_out"], color="#E5C07B",
                    linewidth=1.4, label="throttle cmd")
    colors = ["#E06C75", "#98C379", "#56B6C2", "#C678DD"]
    if b.mot_t:
        m1, m2, m3, m4 = zip(*b.mot_m)
        for i, (m, c) in enumerate(zip([m1, m2, m3, m4], colors), start=1):
            ax_thr.plot(b.mot_t, m, color=c, linewidth=0.8, alpha=0.85,
                        label=f"M{i}")
        # Saturation markers along the bottom strip.
        sat_t = [t for t, m in zip(b.mot_t, b.mot_m)
                 if min(m) <= 0.005 or max(m) >= 0.995]
        if sat_t:
            ax_thr.scatter(sat_t, [-0.04] * len(sat_t), s=6, marker="|",
                           color="#E06C75", zorder=5,
                           label=f"motor saturation ({len(sat_t)})")
    ax_thr.set_ylabel("throttle / motor (0..1)")
    ax_thr.set_ylim(-0.08, 1.05)
    ax_thr.grid(True)
    ax_thr.legend(loc="upper left", ncols=6, fontsize=7,
                  facecolor="#21252B", edgecolor="#3E4452")

    axis_color = ["#E06C75", "#98C379", "#61AFEF"]   # roll/pitch/yaw

    # ---- Row 3: angle SP vs current ----
    angle_curs = ["roll_angle_cur", "pitch_angle_cur", "yaw_angle_cur"]
    angle_sps  = ["roll_angle_sp",  "pitch_angle_sp",  "yaw_angle_sp"]
    angle_lbls = ["roll", "pitch", "yaw"]
    for col, (curk, spk, lbl, c) in enumerate(zip(angle_curs, angle_sps,
                                                  angle_lbls, axis_color)):
        ax = fig.add_subplot(gs[2, col], sharex=ax_state)
        shade_states(ax, ranges)
        if b.pid_t:
            ax.plot(b.pid_t, b.pid[curk], color=c, linewidth=1.2, label="actual")
            ax.plot(b.pid_t, b.pid[spk], color=c, linewidth=0.9, alpha=0.55,
                    linestyle="--", label="setpoint")
        ax.set_title(f"{lbl} angle (deg)")
        ax.grid(True)
        if col == 0:
            ax.legend(loc="upper right", fontsize=7,
                      facecolor="#21252B", edgecolor="#3E4452")

    # ---- Row 4: rate SP vs current ----
    rate_curs = ["roll_rate_cur", "pitch_rate_cur", "yaw_rate_cur"]
    rate_sps  = ["roll_rate_sp",  "pitch_rate_sp",  "yaw_rate_sp"]
    for col, (curk, spk, lbl, c) in enumerate(zip(rate_curs, rate_sps,
                                                  angle_lbls, axis_color)):
        ax = fig.add_subplot(gs[3, col], sharex=ax_state)
        shade_states(ax, ranges)
        if b.pid_t:
            ax.plot(b.pid_t, b.pid[curk], color=c, linewidth=1.2, label="gyro")
            ax.plot(b.pid_t, b.pid[spk], color=c, linewidth=0.9, alpha=0.55,
                    linestyle="--", label="rate sp")
        ax.set_title(f"{lbl} rate (deg/s)")
        ax.grid(True)
        if col == 0:
            ax.legend(loc="upper right", fontsize=7,
                      facecolor="#21252B", edgecolor="#3E4452")

    # ---- Row 5: PID outputs ----
    out_ks = ["roll_out", "pitch_out", "yaw_out"]
    for col, (k, lbl, c) in enumerate(zip(out_ks, angle_lbls, axis_color)):
        ax = fig.add_subplot(gs[4, col], sharex=ax_state)
        shade_states(ax, ranges)
        if b.pid_t:
            ax.plot(b.pid_t, b.pid[k], color=c, linewidth=1.0)
            # +/-1 are the PID OUT_MIN / OUT_MAX rails defined in variables.h.
            ax.axhline(+1.0, color="#E06C75", linewidth=0.6, alpha=0.6, linestyle=":")
            ax.axhline(-1.0, color="#E06C75", linewidth=0.6, alpha=0.6, linestyle=":")
        ax.set_title(f"{lbl} PID output")
        ax.set_ylim(-1.15, 1.15)
        ax.grid(True)

    # ---- Row 6: dt + gyro magnitudes ----
    ax_dt = fig.add_subplot(gs[5, 0], sharex=ax_state)
    shade_states(ax_dt, ranges)
    if b.pid_t:
        # dt in ms is more readable than seconds.
        ax_dt.plot(b.pid_t, [d * 1000.0 for d in b.pid["outer_dt"]],
                   color="#C678DD", linewidth=0.9, label="outer dt (ms)")
        ax_dt.plot(b.pid_t, [d * 1000.0 for d in b.pid["inner_dt"]],
                   color="#56B6C2", linewidth=0.9, label="inner dt (ms)")
    ax_dt.set_title("loop dt")
    ax_dt.set_ylabel("ms")
    ax_dt.grid(True)
    ax_dt.legend(loc="upper right", fontsize=7,
                 facecolor="#21252B", edgecolor="#3E4452")

    ax_g = fig.add_subplot(gs[5, 1:], sharex=ax_state)
    shade_states(ax_g, ranges)
    if b.imu_t:
        gx, gy, gz = zip(*b.imu_gyr)
        ax_g.plot(b.imu_t, gx, color=axis_color[0], linewidth=0.7, label="gx")
        ax_g.plot(b.imu_t, gy, color=axis_color[1], linewidth=0.7, label="gy")
        ax_g.plot(b.imu_t, gz, color=axis_color[2], linewidth=0.7, label="gz")
    ax_g.set_title("gyro (deg/s, raw from BMX160 stream)")
    ax_g.grid(True)
    ax_g.legend(loc="upper right", fontsize=7,
                facecolor="#21252B", edgecolor="#3E4452")
    ax_g.set_xlabel("t (s)")

    if out_path:
        fig.savefig(out_path, dpi=120, facecolor=fig.get_facecolor())
        print(f"  wrote {out_path}")
    if show:
        plt.show()
    plt.close(fig)


# --- text summary ---------------------------------------------------------

def summary(b: LogBundle) -> None:
    ranges = state_ranges(b)
    armed = armed_windows(ranges)
    total_armed = sum(t1 - t0 for t0, t1 in armed)
    print("=" * 70)
    print(f"Duration:   {b.duration_s():.2f} s")
    print(f"States:     " + " -> ".join(
        f"{n}@{t0:.2f}s" for t0, _, n in ranges))
    print(f"Armed for:  {total_armed:.2f} s  (across {len(armed)} window(s))")

    if not b.pid_t or not armed:
        print("(no PID telemetry during armed window - nothing to tune)")
        return

    # Per-axis error stats inside the armed window.
    print()
    print("PID tracking quality (armed window only):")
    print(f"  {'axis':<6} {'angle RMS':>10} {'angle peak':>12} "
          f"{'rate RMS':>10} {'rate peak':>12}")
    for axis in ("roll", "pitch", "yaw"):
        ang_err = []
        rate_err = []
        for i, t in enumerate(b.pid_t):
            if not in_any(t, armed): continue
            ang_err.append(b.pid[f"{axis}_angle_cur"][i] -
                           b.pid[f"{axis}_angle_sp"][i])
            rate_err.append(b.pid[f"{axis}_rate_cur"][i] -
                            b.pid[f"{axis}_rate_sp"][i])
        if not ang_err: continue
        print(f"  {axis:<6} {rms(ang_err):>10.2f} {max(abs(x) for x in ang_err):>12.2f} "
              f"{rms(rate_err):>10.2f} {max(abs(x) for x in rate_err):>12.2f}")

    # Motor saturation rate inside armed.
    armed_m = [m for t, m in zip(b.mot_t, b.mot_m) if in_any(t, armed)]
    sat = saturation_fraction(armed_m)
    print()
    print(f"Motor saturation while armed: {sat * 100:.1f}% "
          f"({sum(1 for m in armed_m if min(m) <= 0.005 or max(m) >= 0.995)}"
          f" / {len(armed_m)} frames)")

    # Loop timing.
    outer = [d for t, d in zip(b.pid_t, b.pid["outer_dt"]) if in_any(t, armed)]
    inner = [d for t, d in zip(b.pid_t, b.pid["inner_dt"]) if in_any(t, armed)]
    if outer:
        print(f"Loop dt (ms): outer mean={1000*sum(outer)/len(outer):.2f}  "
              f"min={1000*min(outer):.2f}  max={1000*max(outer):.2f}")
    if inner:
        print(f"              inner mean={1000*sum(inner)/len(inner):.2f}  "
              f"min={1000*min(inner):.2f}  max={1000*max(inner):.2f}")

    # PID output saturation (hitting +-1)
    print()
    print("PID output rail-hit rate (|out| > 0.99) while armed:")
    for axis in ("roll", "pitch", "yaw"):
        n_hit = sum(1 for i, t in enumerate(b.pid_t)
                    if in_any(t, armed) and abs(b.pid[f"{axis}_out"][i]) > 0.99)
        n_tot = sum(1 for t in b.pid_t if in_any(t, armed))
        if n_tot:
            print(f"  {axis:<6} {n_hit / n_tot * 100:5.1f}%   "
                  f"({n_hit} / {n_tot})")

    # Quick tuning hints based on what we just measured.
    print()
    print("Tuning hints:")
    if sat > 0.10:
        print(f"  - {sat*100:.0f}% of armed time has a motor pinned at 0 or 1.")
        print("    Rate-loop gains are likely too high for this airframe's")
        print("    inertia. Drop DEAFULT_*_ANGLE_RATE_KP by 1.5-2x and rerun.")
    elif sat > 0.02:
        print(f"  - {sat*100:.0f}% motor saturation: rate-loop is hitting")
        print("    limits occasionally; small Kp reduction would help.")
    else:
        print("  - motor saturation is low, rate-loop has headroom.")

    # Cheap oscillation detector: sign changes in roll/pitch angle error
    # during armed.
    for axis in ("roll", "pitch"):
        err = [b.pid[f"{axis}_angle_cur"][i] - b.pid[f"{axis}_angle_sp"][i]
               for i, t in enumerate(b.pid_t) if in_any(t, armed)]
        if len(err) < 50: continue
        zc = sum(1 for a, b_ in zip(err[:-1], err[1:]) if a * b_ < 0)
        dur = sum(1 for t in b.pid_t if in_any(t, armed)) * \
              (1.0 / max(1, len([t for t in b.pid_t if in_any(t, armed)])))
        # Approximate Hz: zero-crossings per armed second / 2.
        if total_armed > 0:
            hz = zc / (2.0 * total_armed)
            if hz > 8:
                print(f"  - {axis} angle error crosses zero ~{hz:.1f} times/sec:")
                print("    oscillation in the angle loop. Reduce angle Kp")
                print("    or add D term (currently 0).")


# --- entry ----------------------------------------------------------------

def newest_log(logs_dir: str) -> str | None:
    if not os.path.isdir(logs_dir): return None
    cands = [os.path.join(logs_dir, f) for f in os.listdir(logs_dir)
             if f.startswith("sim-") and f.endswith(".bin")]
    if not cands: return None
    return max(cands, key=os.path.getmtime)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", nargs="?", help="Path to a sim-*.bin")
    ap.add_argument("--logs-dir", default=None,
                    help="Where to look for the newest log "
                         "(default: <repo>/logs)")
    ap.add_argument("--show", action="store_true",
                    help="Open the figure in an interactive window")
    ap.add_argument("--no-pdf", action="store_true",
                    help="Skip writing the PDF alongside the log")
    args = ap.parse_args(argv)

    if args.log:
        log_path = args.log
    else:
        logs_dir = args.logs_dir
        if logs_dir is None:
            # Default = sibling of this script.
            logs_dir = os.path.abspath(os.path.join(HERE, "..", "..", "logs"))
        log_path = newest_log(logs_dir)
        if log_path is None:
            print(f"no sim-*.bin in {logs_dir}", file=sys.stderr)
            return 1

    print(f"analysing: {log_path} ({os.path.getsize(log_path)} bytes)")
    b = load(log_path)
    summary(b)

    out_pdf = None
    if not args.no_pdf:
        out_pdf = os.path.splitext(log_path)[0] + ".pdf"
    title = f"{os.path.basename(log_path)}  -  PID tuning report"
    plot(b, out_pdf, args.show, title)
    return 0


if __name__ == "__main__":
    sys.exit(main())
