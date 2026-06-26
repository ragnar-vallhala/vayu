#!/usr/bin/env python3
"""On-hardware system-ID plant fit — output-error (simulation-error) method.

A more accurate drop-in alternative to sysid_fit.py's differenced-ARX fit. Same
plant model  omega/u = K / (s (tau s + 1))  and the same loop-shaping/clamps
(reused from sysid_fit.py); only the *identification* changes.

Why this is more accurate than the ARX fit in sysid_fit.py
----------------------------------------------------------
sysid_fit.identify_plant() minimises ONE-STEP-AHEAD error on acceleration:

    accel[k] = a*accel[k-1] + b*u[k]      with  accel = d(omega)/dt

That estimator carries three structural biases on real, noisy gyro data:

  1. It DIFFERENTIATES the gyro. Forward-differencing amplifies HF noise, and
     that noise lands in the REGRESSOR accel[k-1] too — an errors-in-variables
     problem that biases the pole `a` toward 0, i.e. tau too small / actuator BW
     too high. Pre-smoothing softens but cannot remove it.
  2. One-sample input misalignment: the regressor is b*u[k] but the exact ZOH
     model wants the u that drove this step; under a fast chirp u changes a lot
     per sample, so K and tau pick up the documented ~10-30% frequency-dependent
     bias.
  3. Its R^2 is a one-step accel-prediction R^2, dominated by the autoregressive
     term — it stays high even when K/tau are 20-30% off, so it barely validates.

This file fits in OUTPUT-ERROR form instead: it SIMULATES the plant forward from
u with an exact zero-order-hold discretisation and minimises the error against
the *measured rate omega directly* — no differentiation at all. Output-error is
unbiased under output (gyro) noise, the dominant noise source here.

For the cleanest fit, pair this with `sysid_excite.py --inject u`, which perturbs
the rate-PID OUTPUT u directly (near-open-loop) instead of the setpoint, so u is
not shaped by / correlated with the closed loop.

For a fixed time constant tau the simulated rate is LINEAR in K and in the
unknown initial states / offsets, so we use SEPARABLE least squares:

    omega_model[k] = K   * omega_forced[k]   (response to u, zero IC, exact ZOH)
                   + c_a * omega_homog[k]     (decay from unknown initial accel)
                   + c_0                       (unknown rate offset / gyro bias)
                   + c_d * (k*dt)              (slow gyro-bias drift)

Outer loop: a 1-D search over the single nonlinear parameter tau (equivalently
the discrete pole a = exp(-dt/tau)). Inner loop: exact linear least squares for
[K, c_a, c_0, c_d]. This handles the plant's free integrator, the initial
conditions and gyro drift cleanly, recovers K and tau with far less bias, and
reports an HONEST variance-accounted-for (VAF) on omega itself.

Usage mirrors sysid_fit.py (pure-python, no numpy):
  python3 tools/sysid/sysid_fit_oe.py --selftest          # head-to-head vs ARX
  python3 tools/sysid/sysid_fit_oe.py --csv run.csv --axis roll
  python3 tools/sysid/sysid_fit_oe.py --csv run.csv --axis roll --apply

NOTE: the FC capture's control column IS the rate-PID OUTPUT u (sysid_capture()
is fed the PID `outputs`; the SYSID_SAMPLE `u` field carries it x1000) — i.e. the
plant-fit input, not the setpoint. K is meaningful provided the run was ARMED.
--selftest validates the math regardless.
"""
import argparse
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
# Reuse the model container, loop-shaping, clamps, CSV loader and apply path so
# the two fitters stay bit-for-bit consistent everywhere except the estimator.
from sysid_fit import (  # noqa: E402
    Plant, smooth, identify_plant, choose_crossover, design_gains,
    load_csv, apply_gains, report, AXES, WC_MAX, ANGLE_KP_MAX,
)


def _solve_sym(M, rhs):
    """Solve M x = rhs (small dense, Gaussian elimination + partial pivot)."""
    n = len(rhs)
    A = [M[i][:] + [rhs[i]] for i in range(n)]
    for col in range(n):
        piv = max(range(col, n), key=lambda r: abs(A[r][col]))
        if abs(A[piv][col]) < 1e-300:
            return None
        A[col], A[piv] = A[piv], A[col]
        pv = A[col][col]
        for r in range(n):
            if r == col:
                continue
            f = A[r][col] / pv
            if f == 0.0:
                continue
            for c in range(col, n + 1):
                A[r][c] -= f * A[col][c]
    return [A[r][n] / A[r][r] for r in range(n)]


def _basis(u, dt, a):
    """Exact-ZOH rate trajectories for a fixed pole a (= exp(-dt/tau)):
      of[k] = forced rate response to u with K=1 and zero initial state
      oa[k] = homogeneous rate response to a unit initial acceleration
    Both use the closed-form integral of a first-order-lag acceleration over one
    sample, so there is NO sub-stepping and NO differentiation."""
    n = len(u)
    tau = -dt / math.log(a)
    g = tau * (1.0 - a)            # integral of a decaying accel over one step
    of = [0.0] * n
    oa = [0.0] * n
    af = 0.0                       # forced accel state (K=1)
    aa = 1.0                       # homogeneous accel state (accel0 = 1)
    for k in range(n - 1):
        of[k + 1] = of[k] + u[k] * dt + (af - u[k]) * g
        af = a * af + (1.0 - a) * u[k]
        oa[k + 1] = oa[k] + aa * g
        aa = a * aa
    return of, oa


def _fit_fixed(u, omega, dt, a):
    """Separable inner LS: for a fixed pole a, best [K, c_a, c_0, c_d]. Returns
    (coef, rss)."""
    n = len(u)
    of, oa = _basis(u, dt, a)
    cols = [of, oa, [1.0] * n, [k * dt for k in range(n)]]
    nc = len(cols)
    M = [[0.0] * nc for _ in range(nc)]
    rhs = [0.0] * nc
    for i in range(nc):
        ci = cols[i]
        for j in range(i, nc):
            cj = cols[j]
            s = 0.0
            for k in range(n):
                s += ci[k] * cj[k]
            M[i][j] = M[j][i] = s
        s = 0.0
        for k in range(n):
            s += ci[k] * omega[k]
        rhs[i] = s
    coef = _solve_sym(M, rhs)
    if coef is None:
        return None, float("inf")
    rss = 0.0
    for k in range(n):
        m = coef[0] * of[k] + coef[1] * oa[k] + coef[2] + coef[3] * (k * dt)
        d = m - omega[k]
        rss += d * d
    return coef, rss


def identify_plant_oe(u, omega, dt, tau_lo=3e-4, tau_hi=0.5, grid=100, refine=40):
    """Output-error plant fit. 1-D search over tau (log-spaced coarse grid +
    golden-section refine), exact separable LS for K/IC/offset/drift inside."""
    p = Plant()
    n = len(omega)
    if n < 50 or len(u) != n or dt <= 1e-9:
        return p

    def rss_of(tau):
        a = math.exp(-dt / tau)
        if not (0.0 < a < 1.0):
            return float("inf"), None
        coef, rss = _fit_fixed(u, omega, dt, a)
        return rss, coef

    taus = [tau_lo * (tau_hi / tau_lo) ** (i / (grid - 1)) for i in range(grid)]
    best_i, best_rss = 0, float("inf")
    for i, tau in enumerate(taus):
        r, _ = rss_of(tau)
        if r < best_rss:
            best_rss, best_i = r, i

    # golden-section in log(tau) over the bracket around the grid minimum
    llo = math.log(taus[max(0, best_i - 1)])
    lhi = math.log(taus[min(grid - 1, best_i + 1)])
    gr = (math.sqrt(5.0) - 1.0) / 2.0
    c = lhi - gr * (lhi - llo)
    d = llo + gr * (lhi - llo)
    fc = rss_of(math.exp(c))[0]
    fd = rss_of(math.exp(d))[0]
    for _ in range(refine):
        if fc < fd:
            lhi, d, fd = d, c, fc
            c = lhi - gr * (lhi - llo)
            fc = rss_of(math.exp(c))[0]
        else:
            llo, c, fc = c, d, fd
            d = llo + gr * (lhi - llo)
            fd = rss_of(math.exp(d))[0]
    tau = math.exp((llo + lhi) / 2.0)
    rss, coef = rss_of(tau)
    if coef is None or coef[0] <= 0.0:
        return p  # no stable plant with positive DC gain

    a = math.exp(-dt / tau)
    mean = sum(omega) / n
    ss_tot = sum((y - mean) ** 2 for y in omega)
    p.ok = True
    p.a = a
    p.tau = tau
    p.K = coef[0]
    p.b = p.K * (1.0 - a)
    p.r2 = (1.0 - rss / ss_tot) if ss_tot > 1e-30 else 0.0
    p.n = n
    return p


def report_oe(p, g, axis_name):
    if not p.ok:
        print("[oe-fit] PLANT FIT FAILED (data too short, or no stable plant with "
              "positive DC gain — check the capture column is the PID OUTPUT u, "
              "not the setpoint, and that the gyro actually responded i.e. ARMED).")
        return False
    print(f"[oe-fit] axis={axis_name}  n={p.n}  omega-VAF(R^2)={p.r2:.3f}")
    print(f"  plant:  K={p.K:.4g} (rate/s)/u   tau={p.tau*1000:.2f} ms   "
          f"actuator BW={p.actuator_bw_hz():.1f} Hz")
    print(f"  design: wc={g['wc']:.3f} rad/s ({g['wc']/(2*math.pi):.2f} Hz)")
    print(f"  rate_kp={g['rate_kp']:.5f}  rate_ki={g['rate_ki']:.5f}  "
          f"rate_kd={g['rate_kd']:.5f}  angle_kp={g['angle_kp']:.4f}")
    if g.get("angle_kp_capped"):
        print(f"  NOTE: angle_kp capped {g['angle_kp_raw']:.4f} -> {g['angle_kp']:.4f} "
              f"(outer-loop ceiling; see the 2026-06-25 pitch-osc analysis).")
    if p.r2 < 0.8:
        print("  WARNING: omega VAF < 0.8 — poor excitation or model mismatch; "
              "do not apply these gains.")
    return True


def _synth(K_true, tau_true, dt, n, noise, seed=1):
    """True continuous first-order-lag plant driven by a ZOH chirp (u held for one
    control tick), integrated with fine sub-steps so neither estimator gets a
    discretisation home-advantage. Returns (u, omega_with_gyro_noise)."""
    import random
    random.seed(seed)
    sub = 40
    h = dt / sub
    u, omega = [], []
    accel = w = ph = 0.0
    for i in range(n):
        f = 0.5 + (12.0 - 0.5) * (i / n)        # 0.5 -> 12 Hz chirp
        ph += 2 * math.pi * f * dt
        ui = 0.3 * math.sin(ph)
        for _ in range(sub):                    # tau*accel_dot + accel = K*u
            accel += h * (K_true * ui - accel) / tau_true
            w += h * accel
        omega.append(w + noise * random.gauss(0, 1))
        u.append(ui)
    return u, omega


def selftest():
    K_true, tau_true, dt = 800.0, 0.020, 1.0 / 500.0
    n = 1500
    u, omega = _synth(K_true, tau_true, dt, n, noise=0.05)

    p_oe = identify_plant_oe(u, omega, dt)
    g_oe = design_gains(p_oe, choose_crossover(p_oe))
    p_arx = identify_plant(u, smooth(omega, 5), dt)
    g_arx = design_gains(p_arx, choose_crossover(p_arx))

    print(f"[selftest] true plant: K={K_true} tau={tau_true*1000:.1f}ms  "
          f"(continuous plant, ZOH chirp, gyro noise)\n")
    print("-- output-error (this file) --")
    report_oe(p_oe, g_oe, "synthetic")
    print("\n-- differenced-ARX (sysid_fit.py) --")
    report(p_arx, g_arx, "synthetic")

    def err(p):
        return (abs(p.K - K_true) / K_true * 100,
                abs(p.tau - tau_true) / tau_true * 100) if p.ok else None
    eo, ea = err(p_oe), err(p_arx)
    print()
    if ea:
        print(f"[selftest] ARX recovery error:  K {ea[0]:5.1f}%   tau {ea[1]:5.1f}%")
    if eo:
        print(f"[selftest] OE  recovery error:  K {eo[0]:5.1f}%   tau {eo[1]:5.1f}%")
    good = bool(eo) and eo[0] < 12 and eo[1] < 20
    print(f"[selftest] => {'PASS' if good else 'CHECK'} "
          f"(output-error within tight tolerance, unbiased under gyro noise)")
    return 0 if good else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--csv", help="captured run CSV")
    ap.add_argument("--axis", choices=list(AXES), default="roll")
    ap.add_argument("--rate-hz", type=float, default=500.0, help="capture rate")
    ap.add_argument("--bw-frac", type=float, default=0.33)
    ap.add_argument("--wc-max", type=float, default=WC_MAX,
                    help=f"ceiling on the rate-loop crossover rad/s (default {WC_MAX})")
    ap.add_argument("--angle-kp-max", type=float, default=ANGLE_KP_MAX,
                    help=f"ceiling on the designed outer angle_kp (default {ANGLE_KP_MAX})")
    ap.add_argument("--tau-lo", type=float, default=3e-4, help="tau search lower bound [s]")
    ap.add_argument("--tau-hi", type=float, default=0.5, help="tau search upper bound [s]")
    ap.add_argument("--apply", action="store_true", help="push gains via CMD_SET_PID")
    ap.add_argument("--port", type=int, default=14555)
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()

    if args.selftest:
        return selftest()
    if not args.csv:
        ap.error("provide --csv (or --selftest)")

    t, u, omega = load_csv(args.csv)
    dt = 1.0 / args.rate_hz
    p = identify_plant_oe(u, omega, dt, tau_lo=args.tau_lo, tau_hi=args.tau_hi)
    g = design_gains(p, choose_crossover(p, args.bw_frac, wc_max=args.wc_max),
                     args.angle_kp_max)
    ok = report_oe(p, g, args.axis)
    if ok and args.apply:
        if p.r2 < 0.8:
            print("[apply] refusing: omega VAF too low.")
            return 1
        return apply_gains(args.port, AXES[args.axis], g)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
