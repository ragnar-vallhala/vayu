#!/usr/bin/env python3
"""Fast, deterministic autotune eval backend on `vayu_sitl_rtos`.

Runs the real-vaios in-process SITL (Phase 4, docs/plans/sitl-lockstep-sim.md)
for one gain set and returns the rate-loop tracking metric from a seeded
arm+doublet rollout. Each eval is ~0.04 s wall (~70x realtime) and
**bit-deterministic** — same (seed, gains) -> identical corr — so a gain sweep
that took minutes on the realtime FIFO harness takes ~1 s here and is repeatable.

Build the backend first:
    cmake -S sim/host -B build_sitl_rtos -DVAYU_SITL_RTOS_BUILD=ON
    cmake --build build_sitl_rtos --target vayu_sitl_rtos -j$(nproc)

Use as a library:  from rtos_eval import rollout; rollout(rate_kp=0.002)
Or run the sweep:   python3 tools/autotune/rtos_eval.py
"""

import os
import re
import subprocess

_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
BIN = os.environ.get("VAYU_SITL_RTOS_BIN",
                     os.path.join(_ROOT, "build_sitl_rtos", "vayu_sitl_rtos"))
_TUNE = re.compile(
    r"#RTOS-TUNE seed=(\d+) roll_corr=(\S+) pitch_corr=(\S+) yaw_corr=(\S+) "
    r"rate_kp=(\S+) angle_kp=(\S+) wall=(\S+) speedup=(\S+)")

_ENV_KEYS = {
    "rate_kp": "VAYU_RATE_KP", "rate_ki": "VAYU_RATE_KI", "rate_kd": "VAYU_RATE_KD",
    "angle_kp": "VAYU_ANGLE_KP", "yaw_rate_kp": "VAYU_YAW_RATE_KP",
}


def rollout(seed=12345, **gains):
    """Run one arm+doublet rollout for the given gains; return a dict with
    roll/pitch/yaw rate-tracking corr + speedup. Unspecified gains use the
    firmware defaults / persisted pid.bin."""
    env = dict(os.environ, VAYU_RTOS_SCENARIO="doublet", VAYU_RTOS_SEED=str(seed),
               VSIM_FIFO_SUFFIX=f"_rtoseval{os.getpid()}")
    for k, v in gains.items():
        if v is not None:
            env[_ENV_KEYS[k]] = repr(float(v))
    p = subprocess.run([BIN], env=env, capture_output=True, text=True, timeout=60)
    m = _TUNE.search(p.stdout)
    if not m:
        raise RuntimeError(f"no #RTOS-TUNE line from {BIN}\n{p.stderr[-600:]}")
    return {"seed": int(m.group(1)), "roll": float(m.group(2)),
            "pitch": float(m.group(3)), "yaw": float(m.group(4)),
            "rate_kp": float(m.group(5)), "angle_kp": float(m.group(6)),
            "speedup": float(m.group(8))}


def main():
    if not os.path.exists(BIN):
        print(f"build the backend first: {BIN} not found "
              f"(-DVAYU_SITL_RTOS_BUILD=ON)")
        return 2
    print(f"backend: {BIN}\nrate_kp sweep (roll/pitch/yaw rate-tracking corr, "
          f"deterministic in-process rollout):\n")
    print(f"  {'rate_kp':>9} {'roll':>6} {'pitch':>6} {'yaw':>6} {'speed':>7}")
    for kp in (0.0005, 0.001, 0.002, 0.004, 0.008):
        r = rollout(rate_kp=kp)
        print(f"  {kp:>9.4f} {r['roll']:>6.2f} {r['pitch']:>6.2f} {r['yaw']:>6.2f} "
              f"{r['speedup']:>6.0f}x")
    print("\n(each row is a full seeded rollout; re-running gives identical corr.)")
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
