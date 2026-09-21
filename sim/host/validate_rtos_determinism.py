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
"""Determinism validator for the real-vaios in-process SITL (vayu_sitl_rtos).

vayu_sitl_rtos runs the real RTOS scheduler single-threaded with physics linked
in-process (Phase 4, firmware/docs/plans/sitl-lockstep-sim.md), so a run is a pure
function of its seed. This checks that:

  1. same seed   -> bit-identical fingerprints   (determinism), and
  2. different seed -> different fingerprints     (the seed actually drives it,
     i.e. we're not trivially reproducing a constant),

across two firmware-derived fingerprints emitted on the binary's
`#RTOS-RESULT ...` line: imu_fp (the seeded sensor input) and att_fp (the
estimator's attitude trajectory). It also reports the wall-clock speedup.

Build first:
  cmake -S sim/host -B build_sitl_rtos -DVAYU_SITL_RTOS_BUILD=ON
  cmake --build build_sitl_rtos --target vayu_sitl_rtos -j$(nproc)
Run:
  python3 sim/host/validate_rtos_determinism.py
"""

import os
import re
import subprocess
import sys

BIN = os.environ.get(
    "VAYU_SITL_RTOS_BIN",
    os.path.join(os.path.dirname(os.path.abspath(__file__)),
                 "..", "..", "build_sitl_rtos", "vayu_sitl_rtos"))
SAMPLES = os.environ.get("VAYU_RTOS_SAMPLES", "5000")
_RESULT = re.compile(
    r"#RTOS-RESULT seed=(\d+) samples=(\d+) imu_fp=(\S+) att_fp=(\S+) "
    r"wall=(\S+) speedup=(\S+)")


def run(seed):
    """Run the binary once with `seed`; return (imu_fp, att_fp, speedup)."""
    env = dict(os.environ, VAYU_RTOS_SEED=str(seed), VAYU_RTOS_SAMPLES=SAMPLES,
               VSIM_FIFO_SUFFIX=f"_detv{seed}")
    p = subprocess.run([BIN], env=env, capture_output=True, text=True,
                       timeout=120)
    m = _RESULT.search(p.stdout)
    if not m:
        sys.stderr.write(p.stdout + p.stderr)
        raise RuntimeError(f"no #RTOS-RESULT line (seed={seed}); did it build?")
    return m.group(3), m.group(4), float(m.group(6))


def main():
    if not os.path.exists(BIN):
        print(f"FAIL: {BIN} not found — build it first (-DVAYU_SITL_RTOS_BUILD=ON)")
        return 2
    print(f"binary: {BIN}\nsamples/run: {SAMPLES}\n")

    print("1) same seed twice (determinism):")
    a_imu, a_att, a_x = run(12345)
    b_imu, b_att, b_x = run(12345)
    print(f"   run A: imu_fp={a_imu} att_fp={a_att} ({a_x:.1f}x)")
    print(f"   run B: imu_fp={b_imu} att_fp={b_att} ({b_x:.1f}x)")
    same = (a_imu == b_imu) and (a_att == b_att)
    print(f"   -> {'IDENTICAL (deterministic)' if same else 'DIFFER (NON-deterministic!)'}")

    print("\n2) different seed (seed must matter):")
    c_imu, c_att, _ = run(99999)
    print(f"   run C (seed 99999): imu_fp={c_imu} att_fp={c_att}")
    differs = (c_att != a_att)
    print(f"   -> {'DIFFERENT from seed 12345 (good)' if differs else 'SAME as seed 12345 (suspicious!)'}")

    ok = same and differs
    print(f"\n=== verdict: {'PASS' if ok else 'FAIL'} — "
          f"vayu_sitl_rtos is {'deterministic' if same else 'NON-deterministic'}"
          f"{' and seed-sensitive' if differs else ''}; ~{a_x:.0f}x realtime ===")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
