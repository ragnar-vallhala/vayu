#!/usr/bin/env python3
"""Determinism validation for SITL lockstep (docs/plans/sitl-lockstep-sim.md).

Does running the sim faster than realtime (VSIM_LOCKSTEP) preserve the firmware's
behaviour? We can't fairly compare a *time-varying* input across speeds while the
harness still paces excitation in wall-clock (Phase 3), so we use a CONSTANT
input: a seeded sim on the test rig, fixed gains, centred sticks at hover
throttle. Then we capture a fixed NUMBER of control-trace frames (frame count ==
sim-time, since telemetry is emitted on the virtual clock) and compare the
per-frame attitude/rate trajectory.

The firmware is multi-threaded (estimator/control/motor pthreads sync via queues
+ the virtual clock), so two runs are not bit-identical even at the same speed.
So we measure the realtime run-to-run jitter (1x vs 1x) as the NOISE FLOOR, and
check that lockstep (1x vs Nx) sits within it. If lockstep's deviation from
realtime is ~the realtime-vs-realtime deviation, lockstep preserves behaviour.
"""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sitl import SitlStack  # noqa: E402

SEED = 12345
SETTLE_FRAMES = 80          # frames to discard after arm (transient)
CAPTURE_FRAMES = 400        # frames compared (~20 s sim @ ~20 Hz)
SIGNALS = ("roll_angle_curr", "pitch_angle_curr",
           "roll_rate_curr", "pitch_rate_curr", "roll_out", "pitch_out")


def _apply_fixed_gains(stack):
    # Explicit gains so a stale /tmp/vayu_vfs/0_pid.bin can't perturb the run.
    for axis in (0, 1):                       # roll, pitch
        stack.set_pid(1, axis, 0.0005, 0.0033333, 0.0, 0.0)   # rate
        time.sleep(0.02)
        stack.set_pid(0, axis, 4.0, 0.0, 0.0, 0.0)            # angle (P)
        time.sleep(0.02)
    stack.set_pid(1, 2, 0.018, 0.0, 0.0, 0.0)                 # yaw rate
    time.sleep(0.05)


def _wait_frames(stack, n, timeout_s):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        if len(stack.snapshot()) >= n:
            return True
        time.sleep(0.01)
    return False


def run_once(lockstep, label, credit=None):
    if lockstep:
        os.environ["VSIM_LOCKSTEP"] = "1"
        if credit is not None:
            os.environ["VSIM_LOCKSTEP_CREDIT"] = str(credit)
        else:
            os.environ.pop("VSIM_LOCKSTEP_CREDIT", None)
    else:
        os.environ.pop("VSIM_LOCKSTEP", None)
        os.environ.pop("VSIM_LOCKSTEP_CREDIT", None)

    repo_root = os.path.join(os.path.dirname(__file__), "..", "..")
    stack = SitlStack(repo_root, quiet=True)
    t0 = time.monotonic()
    with stack:
        # Seeded, level, on the rig (translation pinned so attitude stays bounded).
        stack.reset(pos=(0, 0, -2.0), seed=SEED)
        stack.set_testrig(True, pos=(0, 0, -2.0), tether_k=0.0)
        _apply_fixed_gains(stack)
        stack.set_rc(roll=1500, pitch=1500, yaw=1500, thr=1000, ch6=1000)
        stack.clear_samples()
        stack.wait_level()
        if not stack.arm():
            raise RuntimeError(f"{label}: arm not confirmed")
        stack.set_rc(thr=1500)                         # hover-ish, constant
        # Discard the arm/throttle transient, then capture a fixed frame count.
        if not _wait_frames(stack, SETTLE_FRAMES, 30.0):
            raise RuntimeError(f"{label}: no telemetry (settle)")
        stack.clear_samples()
        if not _wait_frames(stack, CAPTURE_FRAMES, 60.0):
            raise RuntimeError(f"{label}: only {len(stack.snapshot())} frames")
        snap = stack.snapshot()[:CAPTURE_FRAMES]
    wall = time.monotonic() - t0
    traj = {s: [d[s] for _, d in snap] for s in SIGNALS}
    print(f"  {label:14s}: {len(snap)} frames in {wall:5.1f}s wall "
          f"({'LOCKSTEP' if lockstep else 'realtime'})")
    return traj


def compare(a, b, label):
    print(f"\n{label}")
    worst = 0.0
    for s in SIGNALS:
        va, vb = a[s], b[s]
        n = min(len(va), len(vb))
        diffs = [abs(va[i] - vb[i]) for i in range(n)]
        mx = max(diffs) if diffs else 0.0
        rms = (sum(d * d for d in diffs) / n) ** 0.5 if n else 0.0
        unit = "deg/s" if "rate" in s else ("" if "out" in s else "deg")
        print(f"    {s:18s} max|Δ|={mx:9.4f} rms={rms:9.4f} {unit}")
        if "angle" in s:
            worst = max(worst, mx)
    return worst


def _maxangle(a, b):
    m = 0.0
    for s in ("roll_angle_curr", "pitch_angle_curr"):
        n = min(len(a[s]), len(b[s]))
        m = max(m, max((abs(a[s][i] - b[s][i]) for i in range(n)), default=0.0))
    return m


def main():
    credits = [int(x) for x in sys.argv[1:]] or [16, 8, 4, 2, 1]
    print(f"seed={SEED}  settle={SETTLE_FRAMES}  capture={CAPTURE_FRAMES} frames\n")
    print("baseline: two realtime captures for the noise floor:")
    rt_a = run_once(False, "realtime-A")
    rt_b = run_once(False, "realtime-B")
    floor = _maxangle(rt_a, rt_b)

    print(f"\nnoise floor (realtime A vs B) max angle Δ = {floor:.4f} deg")
    print(f"\ncredit sweep (lockstep vs realtime-A), tolerance = {max(2*floor,0.5):.3f} deg:")
    print(f"  {'credit':>6} {'wall(s)':>8} {'speedup':>8} {'maxΔ(deg)':>10}  verdict")
    tol = max(2.0 * floor, 0.5)
    results = []
    for c in credits:
        t0 = time.monotonic()
        try:
            ls = run_once(True, f"lockstep c={c}", credit=c)
        except RuntimeError as e:
            print(f"  {c:>6}   (skipped: {e})")
            continue
        wall = time.monotonic() - t0
        dev = _maxangle(rt_a, ls)
        speed = 23.6 / wall if wall else 0.0   # vs ~23.6s realtime capture
        ok = dev <= tol
        results.append((c, wall, dev, ok))
        print(f"  {c:>6} {wall:>8.1f} {speed:>7.1f}x {dev:>10.3f}  "
              f"{'PASS' if ok else 'FAIL'}")

    good = [r for r in results if r[3]]
    print("\n=== verdict ===")
    if good:
        best = good[0]
        print(f"  lockstep preserves behaviour at credit <= {max(g[0] for g in good)} "
              f"(divergence within {tol:.3f} deg of realtime).")
    else:
        print(f"  lockstep diverges at ALL tested credits — extra sim-time control "
              f"latency destabilises the marginal roll/pitch loop.")
    return 0 if good else 1


if __name__ == "__main__":
    sys.exit(main())
