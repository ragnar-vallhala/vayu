#!/usr/bin/env python3
"""Verification for the SITL-flakiness fixes (1-4).

Runs the SAME (seed) gains many times and reports the cost spread under two
regimes:
  A) sim_seed=0   -> legacy free-running sensor noise (pre-fix behavior)
  B) sim_seed=fixed -> deterministic per-rollout noise reset (fix 1)

Expectation: B's spread collapses vs A. Also exercises fix 4 (arm() now returns
a confirmed bool) and fix 2/3 implicitly (no phantom 1e6 from starved windows).
"""
import os
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import autotune as AT          # noqa: E402
from sitl import SitlStack     # noqa: E402

N = 6
ROOT = os.path.join(os.path.dirname(__file__), "..", "..")


def run_series(stack, space, seed_base, label):
    costs, fails, arm_fail = [], 0, 0
    for i in range(N):
        # arm-confirm visibility: peek before the rollout drives it
        try:
            c = AT.rollout(stack, space.seed, space.tune_yaw,
                           seed=(seed_base + i if seed_base else 0))
            costs.append(c)
        except AT._RolloutFailed as e:
            fails += 1
            if "arm" in str(e):
                arm_fail += 1
            print(f"  [{label} {i}] FAILED: {e}")
            continue
        print(f"  [{label} {i}] cost = {c:.4f}")
    return costs, fails, arm_fail


def summary(label, costs, fails, arm_fail):
    diverged = sum(1 for c in costs if c >= AT.BIG)
    finite = [c for c in costs if c < AT.BIG]
    print(f"\n== {label} ==")
    print(f"  rollouts: {len(costs)} scored, {fails} dropped ({arm_fail} arm-fail), "
          f"{diverged} diverged (1e6)")
    if finite:
        mean = statistics.mean(finite)
        std = statistics.pstdev(finite) if len(finite) > 1 else 0.0
        print(f"  finite costs: n={len(finite)} mean={mean:.3f} std={std:.3f} "
              f"min={min(finite):.3f} max={max(finite):.3f} "
              f"spread={max(finite) - min(finite):.3f}")
    return finite


def main():
    space = AT.Space(tune_yaw=False)
    print(f"seed gains: {space.fmt(space.seed)}")
    stack = SitlStack(ROOT, quiet=True)
    print("launching SITL stack ...")
    stack.start()
    try:
        # Arm-confirmation smoke (fix 4): does sys-state telemetry decode?
        stack.reset(seed=AT.DEFAULT_SIM_SEED)
        ok = stack.arm()
        print(f"arm confirmed via telemetry: {ok} (last_state={stack._last_state})")
        stack.disarm()

        print("\n--- A: legacy free-running noise (sim_seed=0) ---")
        a, af, aaf = run_series(stack, space, 0, "A")
        print("\n--- B: deterministic reset (fixed sim_seed) ---")
        b, bf, baf = run_series(stack, space, AT.DEFAULT_SIM_SEED, "B")

        fa = summary("A: free-running", a, af, aaf)
        fb = summary("B: deterministic", b, bf, baf)

        print("\n================ VERDICT ================")
        if fa and fb:
            sa = statistics.pstdev(fa) if len(fa) > 1 else 0.0
            sb = statistics.pstdev(fb) if len(fb) > 1 else 0.0
            print(f"finite-cost std:  A={sa:.3f}  ->  B={sb:.3f}")
            if sb <= sa:
                print("PASS: deterministic reset did not increase spread "
                      f"({'collapsed' if sb < sa * 0.5 else 'reduced/equal'}).")
            else:
                print("WARN: B spread > A — investigate (timing jitter dominates?).")
    finally:
        stack.stop()


if __name__ == "__main__":
    main()
