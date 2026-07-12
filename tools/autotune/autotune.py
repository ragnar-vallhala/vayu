#!/usr/bin/env python3
"""SITL PID autotuner (MVP).

Launches a headless physics-in-the-loop stack (vayu_sitl_rtos driver) in test-rig
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

# Default base seed for the deterministic sensor-noise reset (matches the engine's
# built-in default 0xC0FFEE). Repeat i of an eval uses sim_seed+i, so eval(x) is
# reproducible AND --repeats still averages distinct noise realizations.
DEFAULT_SIM_SEED = 0xC0FFEE


class _NoData(Exception):
    """Telemetry starvation: too few samples to score an axis. A harness hiccup
    (pty/scheduling), NOT a physics divergence — retried, never scored as 1e6.
    Conflating the two is what made the cost look bimodal (see autotune-
    methodology.md sec. 8.1)."""


class _RolloutFailed(Exception):
    """A rollout produced no scorable result after retries — arm never
    confirmed, or telemetry starved every attempt. A harness failure, dropped
    from the eval average rather than penalized as a divergence."""

# Base param space (roll/pitch symmetric): (name, lo, hi, seed) — VAYU_SIM seeds.
# Bounds reflect the measured plant: the INNER rate loop limit-cycles (buzzes)
# above rate_kp ~= 0.01, and any real rate_kd differentiates the noisy/deadbanded
# gyro into chatter — so cap both low. RESPONSIVENESS comes from the OUTER
# angle_kp, which doesn't buzz even at high values, so give it a wide range.
_BASE = [
    ("rate_kp", 0.0005,  0.012, 0.007),   # capped below the buzz knee (~0.01)
    ("rate_ki", 0.0,     0.01,  0.002),
    # FLOOR > 0: the soft rig idealizes the attitude estimate, so an UNDAMPED
    # tune (kd=0) scores fine on the rig yet topples the instant a free-flight
    # disturbance hits (freeflight_check failed at 178deg tilt with kd=0). The
    # structured sweep starts each gain at its lower bound, so a zero floor let
    # it lock kd=0; a small floor guarantees minimum damping while staying well
    # under the buzz cap. Upper bound unchanged (D on the noisy gyro buzzes).
    ("rate_kd", 0.0003,  0.001, 0.0005),
    ("angle_kp", 0.05,   3.0,   2.0),     # responsiveness lever; ceiling trimmed
                                          # (very high angle_kp + light damping
                                          # over-tunes the rig -> free-flight flip)
    ("gyro_lpf", 0.0,    0.012, 0.0),     # gyro LPF [s]; lag here destabilizes
]
# Extra params when --yaw: the yaw RATE loop only. Yaw is rate-controlled in
# BOTH flight modes (a centered stick holds the current heading; the mag-fused
# Mahony estimate no longer drifts), so there is NO yaw angle loop to tune — we
# excite a yaw-RATE doublet and score rate tracking. The yaw axis has lower
# control authority than roll/pitch, so it tolerates a wider rate_kp before it
# chatters; rate_kd is still capped (D on the noisy gyro buzzes on any axis).
_YAW = [
    ("yaw_rate_kp", 0.002, 0.05,  0.01),
    ("yaw_rate_ki", 0.0,   0.012, 0.002),
    ("yaw_rate_kd", 0.0,   0.002, 0.0005),
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
        # Yaw RATE loop only (yaw is rate-controlled; the angle loop is unused).
        ykp, yki, ykd, ylpf = x[5:9]
        stack.set_pid(1, 2, ykp, yki, ykd, 0.0)                  # yaw rate loop
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
        # Too few samples to score => the telemetry starved this window, not a
        # flip. Signal a retry instead of returning the 1e6 divergence penalty.
        raise _NoData(f"{axis}: only {len(pts)} samples in window")
    cur = [c for _, c, _ in pts]
    if any((not math.isfinite(c)) or abs(c) > 80.0 for c in cur):
        return BIG                            # genuine divergence / failsafe
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


def _yaw_rate_cost(samples):
    """Cost for the yaw RATE loop from its (rate_sp, rate_curr, out) trace.

    Yaw is rate-controlled (no angle hold), so we score how well the measured
    body rate tracks the commanded rate. The raw error is in deg/s (~0..150),
    so it's normalized to a FRACTION of the commanded rate and rescaled to the
    angle-cost magnitude (~degrees) — that way the yaw term sums sensibly with
    the roll/pitch angle costs instead of dominating or vanishing."""
    pts = [(s[1]["yaw_rate_sp"], s[1]["yaw_rate_curr"], s[1]["yaw_out"]) for s in samples]
    if len(pts) < 5:
        raise _NoData(f"yaw-rate: only {len(pts)} samples in window")
    cur = [c for _, c, _ in pts]
    if any((not math.isfinite(c)) or abs(c) > 2000.0 for c in cur):
        return BIG                            # gyro saturating / divergence
    iae = sum(abs(sp - c) for sp, c, _ in pts) / len(pts)        # mean |err| deg/s
    target = max(20.0, max(abs(sp) for sp, _, _ in pts))         # commanded rate
    iae_n = (iae / target) * 21.0             # fraction-of-command -> angle scale
    overshoot = max(0.0, max(abs(c) for c in cur) - target) / target
    outs = [o for _, _, o in pts]
    chatter = sum(abs(outs[i] - outs[i - 1]) for i in range(1, len(outs))) / len(outs)
    chatter_pen = 25.0 * max(0.0, chatter - 0.04)
    return iae_n + 3.0 * overshoot + chatter_pen


# Rig rollouts pin translation, so the reset height is dynamically irrelevant to
# the attitude search — EXCEPT that spawning a few cm above ground (the old
# -0.05 m) lets the ground-contact righting force (ground_right_gain) kick the
# craft into a spin it never recovers from, diverging every rollout (worse on a
# low-inertia airframe). Spawn the rig WELL above ground so ground contact never
# triggers and the search is immune to the world's ground gains. The free-flight
# check stays near ground on purpose (it must lift off).
RIG_RESET_Z = -2.0   # NED: 2 m up

# Body rate (deg/s) above which the craft is judged to be in a pre-excitation
# transient (a vsim reset/arm glitch), not normal hover. Normal post-settle hover
# is a few deg/s; the glitch spins it at thousands. Sits well between.
PRE_EXCITE_SPIN_MAX = 300.0


def _excite_once(stack, tune_yaw, step_us, hold, ret, settle, hover, seed):
    """A single excitation attempt. Returns the scalar cost, or raises
    _RolloutFailed (arm never confirmed) / _NoData (telemetry starved) so the
    caller can retry instead of recording a phantom divergence."""
    stack.reset(pos=(0, 0, RIG_RESET_Z), seed=seed)    # deterministic noise; off the ground
    stack.set_testrig(True, pos=(0, 0, RIG_RESET_Z),
                      tether_k=stack.tether_k)          # soft rig if tether_k>0
    stack.set_rc(roll=1500, pitch=1500, yaw=1500, ch6=1000)   # ch6 low = angle mode
    # Let the attitude estimator re-level after the reset before arming. A prior
    # rollout's divergence leaves the estimator 'degraded', which blocks arming;
    # waiting for level clears that instead of failing a perfectly good gain set.
    stack.clear_samples()
    stack.wait_level()
    if not stack.arm():                       # CONFIRM armed before exciting
        raise _RolloutFailed("arm not confirmed")
    stack.set_rc(thr=hover)
    time.sleep(settle)

    # Reject a PRE-EXCITATION transient. vsim intermittently injects a large body
    # rate at the reset/arm/hover transition (rates in the thousands of deg/s —
    # physically impossible from motor torque), which then runs away regardless
    # of the gains and would score as a phantom 1e6. It's rare per rollout, but
    # at repeats>1 nearly every eval catches one and the averaged cost pins at
    # kBig, so the search never converges. If the craft is already spinning
    # BEFORE we excite, it's a harness/physics glitch — retry, don't blame gains.
    stack.clear_samples()
    time.sleep(0.1)
    pre = stack.snapshot()
    if pre:
        spin = max(max(abs(d["roll_rate_curr"]), abs(d["pitch_rate_curr"]))
                   for _, d in pre)
        if spin > PRE_EXCITE_SPIN_MAX:
            raise _RolloutFailed(f"pre-excitation transient spin {spin:.0f} deg/s")

    # Step doublet per axis. Roll/pitch are angle-controlled, so the stick is an
    # ANGLE command scored by angle tracking. Yaw is rate-controlled, so the same
    # stick is a RATE command scored by rate tracking (_yaw_rate_cost).
    axes = ["roll", "pitch"] + (["yaw"] if tune_yaw else [])
    cost = 0.0
    # Track the peak attitude/pose across all axis windows (the firmware's
    # estimated pose the controller sees — what diverges). Stashed on `stack`
    # so the eval logger can surface it; mirrors the C++ AutotuneWorker log.
    mx_roll = mx_pitch = mx_yawrate = 0.0
    for axis in axes:
        stack.clear_samples()
        stack.set_rc(**{axis: step_us})
        time.sleep(hold)
        stack.set_rc(**{axis: 1500})
        time.sleep(ret)                       # settle window — chatter shows here
        snap = stack.snapshot()
        for _, d in snap:
            mx_roll = max(mx_roll, abs(d["roll_angle_curr"]))
            mx_pitch = max(mx_pitch, abs(d["pitch_angle_curr"]))
            mx_yawrate = max(mx_yawrate, abs(d["yaw_rate_curr"]))
        cost += _yaw_rate_cost(snap) if axis == "yaw" else _axis_cost(snap, axis)

    stack._last_pose_peak = (mx_roll, mx_pitch, mx_yawrate)
    stack.disarm()
    return cost


def rollout(stack, x, tune_yaw, step_us=1800, hold=1.0, ret=0.7, settle=0.6,
            hover=1500, seed=0, max_retries=2):
    """One excitation rollout; returns the scalar cost for gains x.

    Applies the gains once, then retries the excitation if arming never
    confirmed or the telemetry starved — both harness hiccups, not gain
    quality. Raises _RolloutFailed if every attempt fails (caller drops it)."""
    apply_gains(stack, x, tune_yaw)
    last = None
    for _ in range(max_retries + 1):
        try:
            return _excite_once(stack, tune_yaw, step_us, hold, ret, settle, hover, seed)
        except (_NoData, _RolloutFailed) as e:
            last = e
            # Strong recovery: disarm, re-level, and wait for the estimator to
            # clear 'degraded' (one valid accel sample once the craft is still).
            # On a marginal plant a divergent rollout degrades the estimator,
            # which flips STANDBY->FAILSAFE and blocks the next arm.
            stack.disarm()
            stack.reset(pos=(0, 0, RIG_RESET_Z), seed=seed)   # off the ground (see RIG_RESET_Z)
            stack.clear_samples()
            stack.wait_level(timeout=3.0)
            time.sleep(0.2)
    raise _RolloutFailed(f"no scorable rollout in {max_retries + 1} attempts ({last})")


def freeflight_check(stack, x, tune_yaw, hover=1600, max_tilt=60.0,
                     sim_seed=DEFAULT_SIM_SEED):
    """Validate gains x in FREE flight (test-rig OFF): lift off, then PERTURB each
    axis with a stick step and confirm it RECOVERS to level instead of diverging.

    The rig idealizes the attitude estimate, so a marginal tune (e.g. high angle_kp
    with no rate_kd) can score well there AND survive a clean hover, yet diverge in
    real flight the moment a disturbance kicks it — the accel follows the tilted
    thrust, the estimate lags, the low-margin loop runs away. A clean-hover check
    misses that; a step-and-recover does not. Returns (ok, max_tilt_deg)."""
    apply_gains(stack, x, tune_yaw)
    stack.set_testrig(False)                          # real free flight
    stack.reset(seed=sim_seed, pos=(0, 0, -0.02))     # just above ground
    stack.clear_samples()
    stack.wait_level()
    if not stack.arm():
        return (False, None)                          # couldn't arm -> fail
    stack.set_rc(roll=1500, pitch=1500, yaw=1500)
    stack.set_rc(thr=hover)                            # throttle past hover -> lift off
    time.sleep(1.0)                                    # settle in a hover
    # Disturb each axis and let it return; a marginal loop diverges here.
    for axis in ("roll", "pitch"):
        stack.set_rc(**{axis: 1700})                  # modest step
        time.sleep(0.6)
        stack.set_rc(**{axis: 1500})                  # release -> must recover
        time.sleep(0.9)
    samples = stack.snapshot()
    stack.disarm()
    pts = [(s[1]["roll_angle_curr"], s[1]["pitch_angle_curr"]) for s in samples
           if math.isfinite(s[1]["roll_angle_curr"]) and math.isfinite(s[1]["pitch_angle_curr"])]
    if len(pts) < 10:
        return (False, None)                          # no/garbage telemetry
    tilts = [max(abs(r), abs(p)) for r, p in pts]
    mx = max(tilts)
    # Must stay bounded AND end near level (recovered, not parked at an angle).
    settled = max(max(abs(r), abs(p)) for r, p in pts[-8:]) < 20.0
    return (mx < max_tilt and settled, mx)


# --- throttle-coupled limit-cycle (buzz) detection -----------------------------
# Motor torque authority scales with thrust, so a rate loop that is stable at
# idle can limit-cycle at hover+. The rig DOESN'T pin this away: motors still
# spin per throttle, and a given PID differential makes more torque at higher
# rpm, so the effective loop gain climbs with throttle in the rig too. What the
# rig DOES remove is translation drift -- and that matters, because in FREE
# flight a soft (low-gain) tune sloshes around as it drifts, swamping any
# amplitude metric with low-frequency motion that looks just like a limit cycle.
# So buzz is measured IN THE RIG (no drift), holding centered sticks while
# sweeping throttle, via the high-frequency CHATTER of the rate-loop output
# (mean |Δout| sample-to-sample). A clean tune barely moves the output (≈0); a
# buzzing one slams it ± every sample. Chatter is intrinsically high-pass, so it
# catches the fast limit cycle and ignores any slow trend.
BUZZ_THRESH  = 0.02    # output chatter below this -> no penalty (quiet hold)
BUZZ_SCALE   = 150.0   # cost per unit of output chatter above the threshold
BUZZ_DIVERGE = 100.0   # oscillated past ±80° even pinned: clearly reject

def _chatter(xs):
    if len(xs) < 2:
        return 0.0
    return sum(abs(xs[i] - xs[i - 1]) for i in range(1, len(xs))) / len(xs)

def throttle_buzz(stack, x, tune_yaw, sim_seed=DEFAULT_SIM_SEED):
    """Rig throttle sweep; returns a penalty for throttle-coupled limit cycling.
    Holds centered sticks (setpoint 0°) while stepping throttle hover→above, and
    measures the chatter of the roll/pitch rate-loop output. A clean tune holds
    the output near-still at every throttle; a buzzy one chatters, worse as
    throttle (authority) rises. Returns 0 for a quiet tune, a scaled penalty for
    a chattery one, BUZZ_DIVERGE if it oscillates past ±80°."""
    apply_gains(stack, x, tune_yaw)
    stack.reset(pos=(0, 0, RIG_RESET_Z), seed=sim_seed)   # off the ground (see RIG_RESET_Z)
    stack.set_testrig(True, pos=(0, 0, RIG_RESET_Z),
                      tether_k=0.0)                   # hard pin: no drift to swamp it
    stack.set_rc(roll=1500, pitch=1500, yaw=1500, ch6=1000)   # centered, angle mode
    stack.clear_samples()
    stack.wait_level()
    if not stack.arm():
        raise _RolloutFailed("buzz check: arm not confirmed")
    worst = 0.0
    for thr in (1500, 1650, 1750):                    # hover and above
        stack.set_rc(thr=thr)
        time.sleep(0.5)                               # reach the new thrust
        stack.clear_samples()
        time.sleep(0.8)                               # measurement window
        s = stack.snapshot()
        ang = [max(abs(d[1]["roll_angle_curr"]), abs(d[1]["pitch_angle_curr"]))
               for d in s if math.isfinite(d[1]["roll_angle_curr"])]
        if ang and max(ang) > 80.0:                   # oscillated off even pinned
            stack.disarm()
            return BUZZ_DIVERGE
        ro = [d[1]["roll_out"] for d in s if math.isfinite(d[1]["roll_out"])]
        po = [d[1]["pitch_out"] for d in s if math.isfinite(d[1]["pitch_out"])]
        if len(ro) >= 5:
            worst = max(worst, _chatter(ro), _chatter(po))
    stack.disarm()
    return BUZZ_SCALE * max(0.0, worst - BUZZ_THRESH)


def make_evaluate(stack, space, repeats, verbose, monitor=None, sim_seed=DEFAULT_SIM_SEED,
                  step_us=1800, buzz_check=True):
    prog = {"n": 0, "best": float("inf")}

    def evaluate(x):
        if monitor is not None:
            if monitor.stop.is_set():
                raise OPT.BudgetExhausted()
            monitor.set_current(x)
        # Repeat i uses sim_seed+i: eval(x) is reproducible, and repeats average
        # distinct noise realizations rather than re-rolling the same one.
        vals = []
        for i in range(repeats):
            try:
                vals.append(rollout(stack, x, space.tune_yaw, seed=sim_seed + i,
                                    step_us=step_us))
            except _RolloutFailed as e:
                print(f"#WARN dropped rollout (harness, not gains): {e}", flush=True)
        if vals:
            c = sum(vals) / len(vals)
        else:
            # Every attempt was a harness failure — distinct from a divergence,
            # but unscorable; fall back to the penalty and flag it loudly.
            c = BIG
            print("#WARN no scorable rollout this eval (all attempts failed)", flush=True)
        # Free-flight throttle-sweep buzz penalty (once per eval, not per repeat):
        # the rig tracking cost above cannot see throttle-coupled limit cycling,
        # so add it explicitly to steer the search away from gains that buzz at
        # hover+. Skipped when the rig rollout already failed (c == BIG).
        if buzz_check and c < BIG:
            try:
                bz = throttle_buzz(stack, x, space.tune_yaw, sim_seed)
            except (_NoData, _RolloutFailed):
                bz = 0.0                  # inconclusive free-flight; don't penalize
            if bz > 0.0:
                print(f"#BUZZ eval penalty={bz:.2f}", flush=True)
            c += bz
        if monitor is not None:
            monitor.add_eval(c)
        # Machine-parseable progress line (consumed by the GCS autotune chart).
        prog["n"] += 1
        prog["best"] = min(prog["best"], c)
        print(f"#EVAL {prog['n']} {c:.4f} {prog['best']:.4f}", flush=True)
        if verbose:
            mr, mp, my = getattr(stack, "_last_pose_peak", (0.0, 0.0, 0.0))
            div = "  *** DIVERGED (>80°)" if c >= BIG else ""
            print(f"    eval cost={c:8.3f}  [{space.fmt(x)}]")
            print(f"    pose: max|roll|={mr:.1f}° max|pitch|={mp:.1f}° "
                  f"peak|yawRate|={my:.1f}°/s{div}", flush=True)
        return c
    return evaluate


def run_one(stack, space, name, budget, repeats, seed, verbose, monitor=None,
            sim_seed=DEFAULT_SIM_SEED, step_us=1800, buzz_check=True):
    if monitor is not None:
        monitor.set_optimizer(name)
    rng = random.Random(seed)
    ev = OPT.Evaluator(
        make_evaluate(stack, space, repeats, verbose, monitor, sim_seed, step_us,
                      buzz_check), budget)
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
    base = make_evaluate(stack, space, args.repeats, False, monitor, args.sim_seed,
                         args.step_us, args.buzz_check)(space.seed)
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
                    args.verbose, monitor, args.sim_seed, args.step_us, args.buzz_check)
        print(f"    best cost {r['best_cost']:.3f} in {r['evals']} evals "
              f"({r['seconds']}s): {space.fmt(r['best_x'])}\n")
        results.append(r)
    results.sort(key=lambda r: r["best_cost"])
    return base, results


def report(space, base, results, args, stack, allow_apply=True):
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

    cancelled = args.apply and not allow_apply

    # Free-flight validation: the rig idealizes the attitude estimate, so a tune
    # can score well there yet topple at lift-off. Validate candidates best-first
    # and keep the first that actually flies; never persist one that doesn't.
    chosen = winner
    if args.validate and not cancelled and winner["best_x"]:
        print("\n--- free-flight validation (lift off, hold hover) ---")
        chosen = None
        for r in [r for r in results if r["best_x"]][:3]:
            ok, mx = freeflight_check(stack, r["best_x"], space.tune_yaw,
                                      sim_seed=args.sim_seed)
            tag = (f"max tilt {mx:.0f}°" if mx is not None else "no lift-off / arm")
            print(f"  {r['optimizer']:<12} cost {r['best_cost']:6.2f} -> "
                  f"{'PASS' if ok else 'FAIL'}  ({tag})")
            if ok and chosen is None:
                chosen = r
        if chosen is None:
            print("⚠ ALL tunes FAILED free-flight — they topple at lift-off "
                  "(the rig over-tuned). Re-run with --rig-tether (soft rig) so the "
                  "search is estimator-aware, or add rate_kd / lower angle_kp. "
                  "NOT applying.")

    if cancelled:
        print("NOT applying: run was cancelled (plot window closed) — gains not "
              "persisted to firmware.")
    elif args.apply and chosen and chosen["best_x"]:
        # Safety: the SITL cost is noisy, so the search can land worse than the
        # seed. Never persist a tune that didn't actually beat the baseline.
        if chosen["best_cost"] < base:
            print(f"applying + persisting {chosen['optimizer']} gains to firmware ...")
            apply_gains(stack, chosen["best_x"], space.tune_yaw)
            time.sleep(0.3)
        else:
            print("NOT applying: best tune did not beat the seed baseline "
                  f"({chosen['best_cost']:.2f} >= {base:.2f}) — seed gains kept. "
                  "Try a larger budget or --repeats to denoise.")


def main():
    ap = argparse.ArgumentParser(description="SITL PID autotuner (MVP)")
    ap.add_argument("--repo-root", default=os.path.join(os.path.dirname(__file__), "..", ".."))
    ap.add_argument("--optimizer", default="spsa", choices=list(OPT.REGISTRY))
    ap.add_argument("--compare", action="store_true",
                    help="run every optimizer on equal budgets and compare")
    ap.add_argument("--yaw", action="store_true",
                    help="also tune the yaw RATE loop (adds yaw_rate kp/ki/kd + "
                         "yaw gyro LPF, and a yaw-rate doublet scored by rate "
                         "tracking). Yaw is rate-controlled, so there's no yaw "
                         "angle gain to tune.")
    ap.add_argument("--budget", type=int, default=50, help="rollouts per optimizer")
    ap.add_argument("--repeats", type=int, default=1, help="rollouts averaged per eval")
    ap.add_argument("--rig-tether", type=float, default=30.0,
                    help="soft-rig spring stiffness [1/s^2]: the body translates "
                         "during the doublet so the accel sees free-flight thrust-"
                         "tilt (estimator-aware tuning). 0 = legacy hard pin.")
    ap.add_argument("--no-validate", action="store_true",
                    help="skip the post-search free-flight lift-off validation")
    ap.add_argument("--no-buzz-check", action="store_true",
                    help="skip the per-eval free-flight throttle-sweep buzz "
                         "penalty (the rig can't see throttle-coupled limit "
                         "cycling; this adds it to the cost). Faster but lets a "
                         "tune that buzzes at hover+ win the search.")
    ap.add_argument("--step-us", type=int, default=1800,
                    help="excitation amplitude: the doublet stick pulse [µs] "
                         "(1500=center, 2000=full, ~21° at 1800). Lower it (e.g. "
                         "1650) to soften the doublet on twitchy airframes so the "
                         "marginal seed doesn't flip — see methodology §8.5.")
    ap.add_argument("--seed", type=int, default=1, help="optimizer RNG seed")
    ap.add_argument("--sim-seed", type=int, default=DEFAULT_SIM_SEED,
                    help="base seed for the engine's sensor-noise reset; repeat i "
                         "uses sim_seed+i. Makes eval(x) reproducible. 0 = "
                         "free-running noise (legacy).")
    ap.add_argument("--apply", action="store_true",
                    help="push + persist the best gains to the firmware at the end")
    ap.add_argument("--mixer-geometry", default=None,
                    help="un-rotated motor layout for the FIRMWARE mixer signs "
                         "(GeometryEditor mixerConfig); physics uses --geometry. "
                         "Defaults to --geometry if unset.")
    ap.add_argument("--geometry", default=None,
                    help="vehicle geometry JSON (mass/inertia/motors) to tune against")
    ap.add_argument("--world", default=None,
                    help="world JSON (gravity/linear_drag/angular_drag/...) so the "
                         "tuner flies the SAME plant as the GCS — esp. angular_drag, "
                         "which a no-rate_kd tune depends on for damping")
    ap.add_argument("--fifo-suffix", default=None,
                    help="VSIM_FIFO_SUFFIX for the SITL stack; lets the GCS attach "
                         "its renderer to /tmp/vsim_pose<suffix> and watch the tune")
    ap.add_argument("--plot", action="store_true", help="live matplotlib dashboard")
    ap.add_argument("--out", default="autotune_result.json")
    ap.add_argument("--quiet", action="store_true", help="silence sim subprocess output")
    ap.add_argument("--verbose", action="store_true", help="print every eval")
    args = ap.parse_args()

    args.step_us = max(1520, min(1980, args.step_us))   # keep within stick range
    args.validate = not args.no_validate
    args.buzz_check = not args.no_buzz_check
    space = Space(args.yaw)
    print(f"tuning {len(space.params)} params{' (incl. yaw)' if args.yaw else ''}")
    print(f"seed gains: {space.fmt(space.seed)}")
    print(f"excitation: doublet stick {args.step_us} µs "
          f"(~{(args.step_us - 1500) / 500 * 100:.0f}% throw)")
    geom = None
    if args.geometry:
        with open(args.geometry) as f:
            geom = json.load(f)
        inr = geom.get("inertia", [0] * 9)
        print(f"vehicle geometry: mass={geom['mass']:.3f} kg, "
              f"inertia diag=[{inr[0]:.5g}, {inr[4]:.5g}, {inr[8]:.5g}] kg·m², "
              f"{len(geom['motors'])} motors (from {os.path.basename(args.geometry)})")
    mixgeom = None
    if args.mixer_geometry:
        with open(args.mixer_geometry) as f:
            mixgeom = json.load(f)
        print(f"mixer geometry (un-rotated firmware mix signs) from "
              f"{os.path.basename(args.mixer_geometry)}")
    world = None
    if args.world:
        with open(args.world) as f:
            world = json.load(f)
        print(f"world: gravity={world.get('gravity', 9.81):.2f}, "
              f"linear_drag={world.get('linear_drag', 0.10):.3f}, "
              f"angular_drag={world.get('angular_drag', 0.005):.4f} "
              f"(from {os.path.basename(args.world)})")
    stack = SitlStack(args.repo_root, suffix=args.fifo_suffix,
                      quiet=not args.verbose or args.quiet, geometry=geom,
                      mixer_geometry=mixgeom, world=world)
    stack.tether_k = args.rig_tether
    print(f"rig: {'soft tether k=%.0f' % args.rig_tether if args.rig_tether > 0 else 'hard pin'}"
          f"  ·  free-flight validation: {'on' if args.validate else 'off'}")
    print("launching SITL stack (vayu_sitl_rtos driver, test-rig mode) ...")
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
                    # If the window was closed mid-run (mon.stop set), treat it
                    # as a cancel and DON'T apply/persist the gains.
                    if results:
                        report(space, base, results, args, stack,
                               allow_apply=not mon.stop.is_set())
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
