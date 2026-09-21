#!/usr/bin/env python3
"""On-hardware system-ID plant fit + analytic gain design.

Port of SysId.cpp from the vayu-navigator repo to the capture-CSV workflow. Reads a
captured run (control effort u = rate-PID output, measured rate omega), fits the
rate-loop plant  omega/u = K / (s (tau s + 1))  by the same differenced-ARX
method, then loop-shapes the rate+angle gains. Optionally applies them to the FC
via CMD_SET_PID.

  # validate the fit math with no hardware (synthetic known plant):
  python3 tools/sysid/sysid_fit.py --selftest

  # fit a captured run (CSV columns: t_s, u/control/rate_sp, gyro_dps):
  python3 tools/sysid/sysid_fit.py --csv /tmp/sysid_dump.csv --axis roll

  # ... and push the designed gains to the FC:
  python3 tools/sysid/sysid_fit.py --csv /tmp/sysid_dump.csv --axis roll --apply

  # identify the INDI effectiveness b (=plant K) for rate_indi.h, accumulate
  # all three axes into one JSON (run per axis):
  python3 tools/sysid/sysid_fit.py --csv roll.csv  --axis roll  --indi-json indi_params.json
  python3 tools/sysid/sysid_fit.py --csv pitch.csv --axis pitch --indi-json indi_params.json

NOTE: the FC captures the rate-PID OUTPUT u in the first column (sysid_capture()
is fed the PID `outputs`, scaled x1000; the host writes it as the `u` column).
That control effort is exactly the plant-fit input, so K is meaningful — provided
the run was ARMED so the gyro actually responded. --selftest validates the math
regardless. (For a fit that is far more robust to gyro noise, see sysid_fit_oe.py.)
"""
import argparse
import csv
import json
import math
import os
import sys

AXES = {"roll": 0, "pitch": 1, "yaw": 2}


class Plant:
    def __init__(self):
        self.ok = False
        self.K = 0.0      # DC gain u -> angular accel  [(rate/s)/u]
        self.tau = 0.0    # actuator/filter lag [s]
        self.a = 0.0      # ARX pole (discrete)
        self.b = 0.0      # ARX input gain (discrete)
        self.r2 = 0.0     # one-step accel prediction R^2
        self.n = 0

    def actuator_bw_hz(self):
        return 1.0 / self.tau / (2.0 * math.pi) if (self.ok and self.tau > 1e-9) else 0.0


def smooth(x, w):
    """Centered moving average (odd window). Light pre-filter for omega: the
    differencing in the fit amplifies gyro noise, so a few-sample smooth cuts the
    noise-induced bias on tau a lot (real hardware is noisier than SITL)."""
    if w <= 1:
        return x
    h = w // 2
    out = []
    for i in range(len(x)):
        a = max(0, i - h)
        b = min(len(x), i + h + 1)
        out.append(sum(x[a:b]) / (b - a))
    return out


def identify_plant(u, omega, dt):
    """Differenced-ARX fit (mirrors SysId.cpp identifyPlant)."""
    p = Plant()
    n = len(omega)
    if n < 50 or len(u) != n or dt <= 1e-9:
        return p
    m = n - 1
    accel = [(omega[i + 1] - omega[i]) / dt for i in range(m)]

    # accel[k] = a*accel[k-1] + b*u[k]
    s11 = s12 = s22 = sy1 = sy2 = 0.0
    cnt = 0
    for k in range(1, m):
        x1, x2, y = accel[k - 1], u[k], accel[k]
        s11 += x1 * x1; s12 += x1 * x2; s22 += x2 * x2
        sy1 += y * x1;  sy2 += y * x2
        cnt += 1
    det = s11 * s22 - s12 * s12
    if cnt < 20 or abs(det) < 1e-30:
        return p
    a = (sy1 * s22 - sy2 * s12) / det
    b = (-sy1 * s12 + sy2 * s11) / det
    if not (0.0 < a < 1.0):
        return p  # no stable first-order lag

    mean = sum(accel[1:m]) / (m - 1)
    ss_res = ss_tot = 0.0
    for k in range(1, m):
        pred = a * accel[k - 1] + b * u[k]
        ss_res += (accel[k] - pred) ** 2
        ss_tot += (accel[k] - mean) ** 2

    p.ok = True
    p.a = a
    p.b = b
    p.tau = -dt / math.log(a)
    p.K = b / (1.0 - a)
    p.r2 = (1.0 - ss_res / ss_tot) if ss_tot > 1e-30 else 0.0
    p.n = cnt
    return p


# Absolute ceiling on the rate-loop crossover. The kp_max clamp bounds rate_kp,
# but for a high plant-gain K it leaves wc = kp_max*K large, so the inner loop
# ends up faster than the un-modeled delays (ESC lag, gyro filter, ~1 ms loop
# discretization — none captured by the single-pole fit) can tolerate, and it
# limit-cycles. On 2026-06-25 pitch (K=1381) got wc=16.6 rad/s (2.64 Hz) and
# oscillated at ~2.6 Hz, while roll (K=563, wc=6.8 rad/s = 1.08 Hz) was rock
# stable on the same kp. Cap wc at the proven-stable roll bandwidth so a high-K
# axis is tuned to the same inner-loop speed, not a faster one. Tunable: --wc-max.
WC_MAX = 7.0


def choose_crossover(p, bw_frac=0.33, kp_max=0.012, wc_max=WC_MAX):
    if not p.ok or p.tau <= 1e-9 or p.K <= 0.0:
        return 0.0
    wc = bw_frac / p.tau
    if wc / p.K > kp_max:
        wc = kp_max * p.K
    return min(wc, wc_max)


# Empirical ceiling on the outer (angle) proportional gain. The analytic design
# sets angle_kp = 0.25*wc, but for a high-gain plant choose_crossover() clamps
# rate_kp (good) while leaving wc = kp_max*K large — so angle_kp scales up with
# the plant gain K and the outer loop out-runs what the real inner loop can
# follow. On 2026-06-25 a pitch fit (K=1381) designed angle_kp=4.14 and the FC
# went into a 1.5 Hz cascade limit cycle (railed motor output -> burned a motor;
# see firmware/docs/journal/log-analysis/20260625-231706-pitch-osc/). Proven-stable values
# on this airframe: roll 1.69, free-flight 1.0. Cap at 2.0 so the design can
# never again hand back an outer gain that re-excites the cascade; the operator
# can still soften further. Tunable via --angle-kp-max.
ANGLE_KP_MAX = 2.0


def design_gains(p, wc, angle_kp_max=ANGLE_KP_MAX):
    g = {"rate_kp": 0.0, "rate_ki": 0.0, "rate_kd": 0.0, "angle_kp": 0.0,
         "wc": 0.0, "angle_kp_capped": False}
    if not p.ok or p.K <= 0.0 or wc <= 0.0:
        return g
    g["wc"] = wc
    g["rate_kp"] = wc / p.K
    g["rate_kd"] = g["rate_kp"] * p.tau
    g["rate_ki"] = 0.1 * wc * g["rate_kp"]
    raw_angle_kp = 0.25 * wc
    g["angle_kp"] = min(raw_angle_kp, angle_kp_max)
    g["angle_kp_capped"] = raw_angle_kp > angle_kp_max
    g["angle_kp_raw"] = raw_angle_kp
    return g


# ----------------------------------------------------------------------------
# INDI effectiveness identification.
#
# The INDI rate loop (firmware/src/control/rate_indi.c) needs ONE number per
# axis: b = control effectiveness = d(omega_dot)/d(u), the angular accel per
# unit command. The plant fit already produces it — p.K is the DC gain
# u -> angular accel, i.e. b == K (same deg/s^2-per-unit-u units rate_indi.h
# expects). So "identifying b" is just surfacing K in INDI-ready form; the
# differenced-ARX fit and its --selftest already validate the number.
#
# Alongside b we suggest the two shared INDI tunables:
#   k      outer bandwidth gain [1/s]: omega_dot_des = k*(rate_sp - rate). Set
#          to the same loop crossover the PID design targets (choose_crossover),
#          so INDI and PID aim at one closed-loop bandwidth (wc-capped for high-K
#          axes, same guard as the PID path).
#   lpf_rc synchronized filter time constant [s]: anchor near the actuator lag
#          tau so the gyro-derivative and command feedback share one delay (the
#          core INDI requirement), clamped to a sane fast range.
# ----------------------------------------------------------------------------
def design_indi(p, wc, lpf_lo=0.003, lpf_hi=0.008):
    d = {"ok": False, "b": 0.0, "k": 0.0, "lpf_rc": 0.0}
    if not p.ok or p.K <= 0.0:
        return d
    d["ok"] = True
    d["b"] = p.K                                   # effectiveness == plant DC gain
    d["k"] = wc                                    # outer bandwidth = PID crossover
    d["lpf_rc"] = max(lpf_lo, min(p.tau, lpf_hi))  # sync filter ~ actuator lag
    return d


# Per-axis -> the rate_indi.h #define that carries b (k/lpf are shared defines).
INDI_B_DEFINE = {"roll": "DEAFULT_ROLL_INDI_B", "pitch": "DEAFULT_PITCH_INDI_B",
                 "yaw": "DEAFULT_YAW_INDI_B"}


def report_indi(p, d, axis_name):
    if not d["ok"]:
        print("[indi] no effectiveness — plant fit failed (see [fit] note).")
        return False
    print(f"[indi] axis={axis_name}  b={d['b']:.4g} deg/s^2 per unit u  (= plant K)")
    print(f"       k={d['k']:.3f} 1/s   lpf_rc={d['lpf_rc']*1000:.1f} ms   "
          f"(R^2={p.r2:.3f})")
    bdef = INDI_B_DEFINE.get(axis_name, f"DEAFULT_{axis_name.upper()}_INDI_B")
    print(f"  rate_indi.h:  #define {bdef:<22s} {d['b']:.1f}f")
    print(f"     (shared)   #define DEAFULT_RATE_INDI_K    {d['k']:.1f}f")
    print(f"     (shared)   #define DEAFULT_RATE_INDI_LPF  {d['lpf_rc']:.4f}f")
    if p.r2 < 0.5:
        print("  WARNING: low R^2 — b is unreliable; re-capture before trusting it.")
    return True


def write_indi_json(path, axis_name, p, d):
    """Merge this axis's identified b/k/lpf into a per-axis INDI params JSON
    (created if absent), so all three axes accumulate into one file across runs."""
    doc = {}
    if os.path.exists(path):
        try:
            with open(path) as f:
                doc = json.load(f)
        except (OSError, ValueError):
            doc = {}
    doc.setdefault("_comment",
                   "INDI per-axis effectiveness b (=sysid plant K) + suggested "
                   "k/lpf. Paste b into firmware/include/control/rate_indi.h "
                   "(DEAFULT_*_INDI_B); k/lpf are shared DEAFULT_RATE_INDI_*.")
    doc.setdefault("axes", {})
    doc["axes"][axis_name] = {
        "b": round(d["b"], 3), "k": round(d["k"], 3),
        "lpf_rc": round(d["lpf_rc"], 4),
        "from": {"K": round(p.K, 3), "tau_ms": round(p.tau * 1000, 3),
                 "r2": round(p.r2, 3)},
    }
    with open(path, "w") as f:
        json.dump(doc, f, indent=2)
        f.write("\n")
    print(f"[indi] wrote {axis_name} -> {path}")


def load_csv(path):
    """Return (t, u, omega). Accepts t_s + a control column (u/control/rate_sp)
    + a rate column (gyro_dps/rate/omega)."""
    with open(path) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return [], [], []
    cols = rows[0].keys()
    ucol = next((c for c in ("u", "control", "u_native", "rate_sp_dps") if c in cols), None)
    ycol = next((c for c in ("gyro_dps", "omega", "rate", "rate_curr_dps") if c in cols), None)
    tcol = next((c for c in ("t_s", "t") if c in cols), None)
    if not ucol or not ycol:
        raise SystemExit(f"CSV missing control/rate columns; have {list(cols)}")
    t = [float(r[tcol]) for r in rows] if tcol else list(range(len(rows)))
    u = [float(r[ucol]) for r in rows]
    y = [float(r[ycol]) for r in rows]
    return t, u, y


def report(p, g, axis_name):
    if not p.ok:
        print("[fit] PLANT FIT FAILED (data too short, or no stable first-order "
              "lag — check that the capture column is the PID OUTPUT u, not the "
              "setpoint, and that the gyro actually responded i.e. ARMED).")
        return False
    print(f"[fit] axis={axis_name}  n={p.n}  R^2={p.r2:.3f}")
    print(f"  plant:  K={p.K:.4g} (rate/s)/u   tau={p.tau*1000:.2f} ms   "
          f"actuator BW={p.actuator_bw_hz():.1f} Hz")
    print(f"  design: wc={g['wc']:.3f} rad/s ({g['wc']/(2*math.pi):.2f} Hz)")
    print(f"  rate_kp={g['rate_kp']:.5f}  rate_ki={g['rate_ki']:.5f}  "
          f"rate_kd={g['rate_kd']:.5f}  angle_kp={g['angle_kp']:.4f}")
    if g.get("angle_kp_capped"):
        print(f"  NOTE: angle_kp capped {g['angle_kp_raw']:.4f} -> {g['angle_kp']:.4f} "
              f"(outer-loop ceiling; raw 0.25*wc would re-excite the ~1.5 Hz "
              f"cascade — see the 2026-06-25 pitch-osc analysis).")
    if p.r2 < 0.5:
        print("  WARNING: low R^2 — fit is poor; do not apply these gains.")
    return True


def selftest(show_indi=False):
    """Simulate a known plant driven by the same chirp the FC injects, fit it,
    and check the recovery is within the method's inherent accuracy. (This is a
    faithful port of SysId.cpp's differenced-ARX fit; with a chirp the lagged
    regressor is correlated with the input, so K/tau carry a known ~10-30% bias —
    the loop-shaping is designed to be robust to it.)"""
    import random
    random.seed(1)
    K_true, tau_true, dt = 800.0, 0.020, 1.0 / 500.0
    n = 1500
    u, omega = [], []
    accel = w = 0.0
    ph = 0.0
    for i in range(n):
        f = 0.5 + (12.0 - 0.5) * (i / n)            # 0.5->12 Hz chirp
        ph += 2 * math.pi * f * dt
        ui = 0.3 * math.sin(ph)
        accel += dt / tau_true * (K_true * ui - accel)  # tau*d(accel)/dt+accel=K*u
        w += dt * accel
        omega.append(w + 0.02 * random.gauss(0, 1))     # realistic gyro noise
        u.append(ui)
    p = identify_plant(u, smooth(omega, 5), dt)
    wc = choose_crossover(p)
    g = design_gains(p, wc)
    print(f"[selftest] true plant: K={K_true} tau={tau_true*1000:.1f}ms")
    ok = report(p, g, "synthetic")
    if ok and show_indi:
        report_indi(p, design_indi(p, wc), "synthetic")
    if ok:
        eK = abs(p.K - K_true) / K_true * 100
        eT = abs(p.tau - tau_true) / tau_true * 100
        good = eK < 25 and eT < 40
        print(f"[selftest] recovery error: K {eK:.1f}%  tau {eT:.1f}%  => "
              f"{'PASS (within method bias)' if good else 'CHECK'}")
        print("[selftest] note: gains are loop-shaped below the actuator BW, so a "
              "~20% plant error gives a slightly conservative tune, not instability.")
        return 0 if good else 1
    return 1


def apply_gains(port, axis, g):
    """Push rate + angle gains to the FC via CMD_SET_PID."""
    HERE = os.path.dirname(os.path.abspath(__file__))
    ROOT = os.path.dirname(os.path.dirname(HERE))
    sys.path.insert(0, os.path.join(ROOT, "navlink", "sim"))
    sys.path.insert(0, os.path.join(ROOT, "navlink", "generated", "python"))
    import socket, time
    import frame, navlink_msgs as nl
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    s.bind(("0.0.0.0", port)); s.settimeout(0.1)
    peer = None; t0 = time.time(); lh = 0
    while not peer and time.time() - t0 < 4:
        if time.time() - lh > 0.4:
            s.sendto(b"GCS-HELLO", ("255.255.255.255", port)); lh = time.time()
        try:
            d, a = s.recvfrom(2048)
            if d != b"GCS-HELLO":
                peer = a
        except socket.timeout:
            pass
    if not peer:
        print("[apply] no FC seen"); return 1
    seq = [0]
    def send(m, pl):
        seq[0] = (seq[0] + 1) & 0xFF
        s.sendto(frame.encode(m, pl, seq=seq[0]), peer)
    send(nl.TimeSync.MSGID, nl.TimeSync(role=0, seq=1, t1_gcs_tx=int(time.time()*1e6)).pack())
    time.sleep(0.05)
    # controller 1=rate, 0=angle ; axis 0/1/2
    send(nl.CmdSetPid.MSGID, nl.CmdSetPid(target_sys=42, target_comp=1, req_seq=seq[0],
        controller=1, axis=axis, kp=g["rate_kp"], ki=g["rate_ki"], kd=g["rate_kd"], kff=0.0).pack())
    time.sleep(0.05)
    send(nl.CmdSetPid.MSGID, nl.CmdSetPid(target_sys=42, target_comp=1, req_seq=seq[0],
        controller=0, axis=axis, kp=g["angle_kp"], ki=0.0, kd=0.0, kff=0.0).pack())
    print(f"[apply] sent rate+angle PID for axis {axis} to {peer[0]}")
    time.sleep(0.3)
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--csv", help="captured run CSV")
    ap.add_argument("--axis", choices=list(AXES), default="roll")
    ap.add_argument("--rate-hz", type=float, default=500.0, help="capture rate")
    ap.add_argument("--bw-frac", type=float, default=0.33)
    ap.add_argument("--wc-max", type=float, default=WC_MAX,
                    help="ceiling on the rate-loop crossover rad/s (default "
                         f"{WC_MAX}); guards a high-K plant from a too-fast inner "
                         "loop (2026-06-25 pitch limit cycle).")
    ap.add_argument("--angle-kp-max", type=float, default=ANGLE_KP_MAX,
                    help="ceiling on the designed outer angle_kp (default "
                         f"{ANGLE_KP_MAX}); guards against the high-K cascade "
                         "(2026-06-25 pitch). Lower for a softer outer loop.")
    ap.add_argument("--smooth", type=int, default=5,
                    help="omega pre-smoothing window (odd; cuts gyro-noise bias)")
    ap.add_argument("--apply", action="store_true", help="push gains via CMD_SET_PID")
    ap.add_argument("--port", type=int, default=14555)
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--indi", action="store_true",
                    help="also identify the INDI effectiveness b (=plant K) and "
                         "suggest k/lpf for firmware/include/control/rate_indi.h")
    ap.add_argument("--indi-json", metavar="PATH",
                    help="merge the identified INDI b/k/lpf for this axis into a "
                         "per-axis JSON (implies --indi); accumulates all 3 axes")
    args = ap.parse_args()
    if args.indi_json:
        args.indi = True

    if args.selftest:
        return selftest(args.indi)
    if not args.csv:
        ap.error("provide --csv (or --selftest)")

    t, u, omega = load_csv(args.csv)
    dt = 1.0 / args.rate_hz
    p = identify_plant(u, smooth(omega, args.smooth), dt)
    wc = choose_crossover(p, args.bw_frac, wc_max=args.wc_max)
    g = design_gains(p, wc, args.angle_kp_max)
    ok = report(p, g, args.axis)
    if ok and args.indi:
        ind = design_indi(p, wc)
        report_indi(p, ind, args.axis)
        if args.indi_json and ind["ok"]:
            write_indi_json(args.indi_json, args.axis, p, ind)
    if ok and args.apply:
        if p.r2 < 0.5:
            print("[apply] refusing: R^2 too low.")
            return 1
        return apply_gains(args.port, AXES[args.axis], g)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
