#!/usr/bin/env python3
"""SITL PID autotuner (MVP).

Launches a headless physics-in-the-loop stack (vsim_d + vayu_sitl) in test-rig
mode (translation pinned, rotation free), excites attitude step doublets, reads
the firmware's control-telemetry (setpoint vs measured) and searches the PID
gains that minimize a tracking cost.

The plant is the cascaded angle->rate loop. By default we tune, symmetric
across roll & pitch:  rate Kp, rate Ki, rate Kd, angle Kp  (4 params).
With --yaw, yaw gets its own 4 params (rate Kp/Ki/Kd + angle Kp), excited on a
third doublet and folded into the cost.

Examples:
  python3 autotune.py --optimizer spsa --budget 60 --apply
  python3 autotune.py --compare --budget 50
  python3 autotune.py --yaw --optimizer portfolio --budget 80
"""

import argparse
import json
import math
import os
import random
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import protocol as P            # noqa: E402
from sitl import SitlStack      # noqa: E402
import optimizers as OPT        # noqa: E402

BIG = 1.0e6

# Base param space (roll/pitch symmetric): (name, lo, hi, seed) — VAYU_SIM seeds.
# Bounds reflect the measured plant: the INNER rate loop limit-cycles (buzzes)
# above rate_kp ~= 0.01, and any real rate_kd differentiates the noisy/deadbanded
# gyro into chatter — so cap both low. RESPONSIVENESS comes from the OUTER
# angle_kp, which doesn't buzz even at high values, so give it a wide range.
_BASE = [
    ("rate_kp", 0.0005, 0.012, 0.007),    # capped below the buzz knee (~0.01)
    ("rate_ki", 0.0,    0.01,  0.002),
    ("rate_kd", 0.0,    0.001, 0.0005),   # capped: D on noisy gyro buzzes
    ("angle_kp", 0.05,  4.0,   2.0),      # the responsiveness lever (safe)
    ("gyro_lpf", 0.0,   0.012, 0.0),      # gyro LPF [s]; lag here destabilizes
]
# Extra params when --yaw. Tuned in angle mode like roll/pitch, but the bounds
# are CAPPED conservatively: in sim the mag-less Mahony yaw drifts, so a high
# yaw_angle_kp (or integral) chases the phantom heading and spins out over a
# session. Keeping the gains modest keeps the drift-chase bounded and stable.
_YAW = [
    ("yaw_rate_kp", 0.001, 0.025, 0.01),
    ("yaw_rate_ki", 0.0,   0.008, 0.002),
    ("yaw_rate_kd", 0.0,   0.002, 0.0005),
    ("yaw_angle_kp", 0.05, 2.0,   1.2),
    ("yaw_gyro_lpf", 0.0,  0.012, 0.0),
]


class Space:
    def __init__(self, tune_yaw):
        self.tune_yaw = tune_yaw
        self.params = _BASE + (_YAW if tune_yaw else [])
        self.names = [p[0] for p in self.params]
        self.bounds = [(p[1], p[2]) for p in self.params]
        self.seed = [p[3] for p in self.params]

    def fmt(self, x):
        return ", ".join(f"{n}={v:.5g}" for n, v in zip(self.names, x))


def apply_gains(stack, x, tune_yaw):
    """Push x to the rate (roll,pitch[,yaw]) + angle controllers and the gyro
    LPF. Param order: [rate_kp,rate_ki,rate_kd,angle_kp,gyro_lpf] then, with
    --yaw, [yaw_rate_kp,yaw_rate_ki,yaw_rate_kd,yaw_angle_kp,yaw_gyro_lpf]."""
    rate_kp, rate_ki, rate_kd, angle_kp, gyro_lpf = x[0:5]
    for axis in (0, 1):                       # 0=roll, 1=pitch
        stack.set_pid(1, axis, rate_kp, rate_ki, rate_kd, 0.0)   # rate loop
        time.sleep(0.02)
        stack.set_pid(0, axis, angle_kp, 0.0, 0.0, 0.0)          # angle loop (P)
        time.sleep(0.02)
        stack.set_gyro_lpf(axis, gyro_lpf)                       # input filter
        time.sleep(0.02)
    if tune_yaw:
        ykp, yki, ykd, yakp, ylpf = x[5:10]
        stack.set_pid(1, 2, ykp, yki, ykd, 0.0)                  # yaw rate
        time.sleep(0.02)
        stack.set_pid(0, 2, yakp, 0.0, 0.0, 0.0)                 # yaw angle (capped)
        time.sleep(0.02)
        stack.set_gyro_lpf(2, ylpf)
        time.sleep(0.02)
    time.sleep(0.05)


def _axis_cost(samples, axis):
    """Cost for one axis from its (setpoint, measured, output) trace.

    Tracking (IAE) + overshoot + a strong CHATTER term: the mean
    sample-to-sample swing of the controller output. Undamped high-gain tunes
    track fine but buzz the motors (output slamming +/-) — chatter catches that
    so the search prefers smooth, damped gains over twitchy ones."""
    sp_k, cur_k, out_k = f"{axis}_angle_sp", f"{axis}_angle_curr", f"{axis}_out"
    pts = [(s[1][sp_k], s[1][cur_k], s[1][out_k]) for s in samples]
    if len(pts) < 5:
        return BIG
    cur = [c for _, c, _ in pts]
    if any((not math.isfinite(c)) or abs(c) > 80.0 for c in cur):
        return BIG                            # diverged / failsafe
    iae = sum(abs(sp - c) for sp, c, _ in pts) / len(pts)     # mean |err| [deg]
    target = max(1.0, max(abs(sp) for sp, _, _ in pts))
    overshoot = max(0.0, max(abs(c) for c in cur) - target) / target
    outs = [o for _, _, o in pts]
    chatter = sum(abs(outs[i] - outs[i - 1]) for i in range(1, len(outs))) / len(outs)
    # Dead-zone penalty: mild output activity is free (a responsive loop needs
    # some), but buzz above ~0.04 is punished hard. This lets the search push
    # gains up for tracking until they START to chatter, then stops — finding
    # the responsiveness/noise knee instead of either extreme.
    chatter_pen = 25.0 * max(0.0, chatter - 0.04)
    return iae + 3.0 * overshoot + chatter_pen


def rollout(stack, x, tune_yaw, step_us=1800, hold=1.0, ret=0.7, settle=0.6, hover=1500):
    """One excitation rollout; returns the scalar cost for gains x."""
    apply_gains(stack, x, tune_yaw)
    stack.reset()
    stack.set_testrig(True)
    stack.set_rc(roll=1500, pitch=1500, yaw=1500, ch6=1000)   # ch6 low = angle mode
    stack.arm()
    stack.set_rc(thr=hover)
    time.sleep(settle)

    # Angle-mode step doublet + angle-tracking cost, per axis. Yaw stays in
    # angle mode too (stable per-rollout); its bounds are capped so the search
    # can't push it into the drift-chasing spin region.
    axes = ["roll", "pitch"] + (["yaw"] if tune_yaw else [])
    cost = 0.0
    for axis in axes:
        stack.clear_samples()
        stack.set_rc(**{axis: step_us})
        time.sleep(hold)
        stack.set_rc(**{axis: 1500})
        time.sleep(ret)                       # settle window — chatter shows here
        cost += _axis_cost(stack.snapshot(), axis)

    stack.disarm()
    return cost


def make_evaluate(stack, space, repeats, verbose, monitor=None):
    prog = {"n": 0, "best": float("inf")}

    def evaluate(x):
        if monitor is not None:
            if monitor.stop.is_set():
                raise OPT.BudgetExhausted()
            monitor.set_current(x)
        vals = [rollout(stack, x, space.tune_yaw) for _ in range(repeats)]
        c = sum(vals) / len(vals)
        if monitor is not None:
            monitor.add_eval(c)
        # Machine-parseable progress line (consumed by the GCS autotune chart).
        prog["n"] += 1
        prog["best"] = min(prog["best"], c)
        print(f"#EVAL {prog['n']} {c:.4f} {prog['best']:.4f}", flush=True)
        if verbose:
            print(f"    eval cost={c:8.3f}  [{space.fmt(x)}]")
        return c
    return evaluate


def run_one(stack, space, name, budget, repeats, seed, verbose, monitor=None):
    if monitor is not None:
        monitor.set_optimizer(name)
    rng = random.Random(seed)
    ev = OPT.Evaluator(make_evaluate(stack, space, repeats, verbose, monitor), budget)
    fn = OPT.REGISTRY[name]
    t0 = time.monotonic()
    try:
        fn(ev, space.seed, space.bounds, rng)
    except OPT.BudgetExhausted:
        pass
    return {"optimizer": name, "best_cost": ev.best, "best_x": ev.best_x,
            "evals": ev.n, "seconds": round(time.monotonic() - t0, 1),
            "history": ev.history}


def optimize(stack, space, args, monitor=None):
    """Run baseline + the selected optimizer(s); return (baseline, results)."""
    base = make_evaluate(stack, space, args.repeats, False, monitor)(space.seed)
    if monitor is not None:
        monitor.set_baseline(base)
    print(f"baseline cost @ seed gains: {base:.3f}\n")

    # Fail fast: if even the seed gains diverge, the plant is unstable (almost
    # always a motor-layout mismatch — the vehicle's motor numbering/positions
    # don't match the firmware's fixed X-quad mixer, so a control axis is
    # sign-inverted and the craft settles inverted). No point tuning.
    if base > 1000.0:
        print("ABORT: baseline diverges — the vehicle is UNSTABLE with the "
              "firmware mixer before any tuning.")
        print("  Likely cause: motor layout mismatch. The firmware mixer is a "
              "fixed X-quad (M1=front-right,+Y / M2=back-right / M3=back-left / "
              "M4=front-left). Reorder the vehicle's motors to match, then retry.")
        return base, []

    methods = list(OPT.REGISTRY) if args.compare else [args.optimizer]
    results = []
    for m in methods:
        if monitor is not None and monitor.stop.is_set():
            break
        print(f"=== optimizer: {m} (budget {args.budget}) ===")
        r = run_one(stack, space, m, args.budget, args.repeats, args.seed,
                    args.verbose, monitor)
        print(f"    best cost {r['best_cost']:.3f} in {r['evals']} evals "
              f"({r['seconds']}s): {space.fmt(r['best_x'])}\n")
        results.append(r)
    results.sort(key=lambda r: r["best_cost"])
    return base, results


def report(space, base, results, args, stack):
    winner = results[0]
    print("================ summary ================")
    print(f"baseline: {base:.3f}")
    for r in results:
        print(f"  {r['optimizer']:<12} {r['best_cost']:8.3f}  ({r['evals']} evals)")
    gain = (1 - winner["best_cost"] / base) * 100 if base > 0 else 0
    print(f"\nBEST: {winner['optimizer']}  cost {winner['best_cost']:.3f} "
          f"({gain:.0f}% better than seed)")
    print(f"gains: {space.fmt(winner['best_x'])}")
    with open(args.out, "w") as f:
        json.dump({"baseline": base, "params": space.names, "results": results}, f, indent=2)
    print(f"wrote {args.out}")
    if args.apply and winner["best_x"]:
        # Safety: the SITL cost is noisy, so the search can land worse than the
        # seed. Never persist a tune that didn't actually beat the baseline.
        if winner["best_cost"] < base:
            print("applying + persisting best gains to firmware ...")
            apply_gains(stack, winner["best_x"], space.tune_yaw)
            time.sleep(0.3)
        else:
            print("NOT applying: best tune did not beat the seed baseline "
                  f"({winner['best_cost']:.2f} >= {base:.2f}) — seed gains kept. "
                  "Try a larger budget or --repeats to denoise.")


def main():
    ap = argparse.ArgumentParser(description="SITL PID autotuner (MVP)")
    ap.add_argument("--repo-root", default=os.path.join(os.path.dirname(__file__), "..", ".."))
    ap.add_argument("--optimizer", default="spsa", choices=list(OPT.REGISTRY))
    ap.add_argument("--compare", action="store_true",
                    help="run every optimizer on equal budgets and compare")
    ap.add_argument("--yaw", action="store_true",
                    help="also tune yaw (adds 4 params + a yaw doublet)")
    ap.add_argument("--budget", type=int, default=50, help="rollouts per optimizer")
    ap.add_argument("--repeats", type=int, default=1, help="rollouts averaged per eval")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--apply", action="store_true",
                    help="push + persist the best gains to the firmware at the end")
    ap.add_argument("--geometry", default=None,
                    help="vehicle geometry JSON (mass/inertia/motors) to tune against")
    ap.add_argument("--plot", action="store_true", help="live matplotlib dashboard")
    ap.add_argument("--out", default="autotune_result.json")
    ap.add_argument("--quiet", action="store_true", help="silence sim subprocess output")
    ap.add_argument("--verbose", action="store_true", help="print every eval")
    args = ap.parse_args()

    space = Space(args.yaw)
    print(f"tuning {len(space.params)} params{' (incl. yaw)' if args.yaw else ''}")
    print(f"seed gains: {space.fmt(space.seed)}")
    geom = None
    if args.geometry:
        with open(args.geometry) as f:
            geom = json.load(f)
        print(f"vehicle geometry: mass={geom['mass']:.3f} kg, {len(geom['motors'])} motors "
              f"(from {os.path.basename(args.geometry)})")
    stack = SitlStack(args.repo_root, quiet=not args.verbose or args.quiet, geometry=geom)
    print("launching SITL stack (vsim_d + vayu_sitl, test-rig mode) ...")
    stack.start()
    try:
        if args.plot:
            import threading
            from monitor import Monitor
            import plotter
            mon = Monitor(stack, space.names)
            box = {}

            def worker():
                try:
                    base, results = optimize(stack, space, args, mon)
                    box["results"] = results
                    # Persist + print as soon as tuning finishes, so the
                    # artifact exists while the window stays open on "DONE".
                    if results:
                        report(space, base, results, args, stack)
                except OPT.BudgetExhausted:
                    pass
                finally:
                    mon.finish()
            th = threading.Thread(target=worker, daemon=True)
            th.start()
            plotter.run(mon)          # blocks in the main thread until window closed
            mon.stop.set()
            th.join(timeout=10)
        else:
            base, results = optimize(stack, space, args)
            if results:
                report(space, base, results, args, stack)
    finally:
        stack.stop()


if __name__ == "__main__":
    main()
