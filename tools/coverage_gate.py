#!/usr/bin/env python3
"""Per-component host-coverage regression gate — the "ratchet".

Reads a `gcovr --json-summary` report and fails (exit 1) if any component's
line coverage has dropped below its committed floor. Floors sit at/just under
the current measured value, so coverage can only go up: any regression breaks
CI. When you genuinely raise a component, bump its floor here in the same PR so
the new bar is enforced going forward.

Per-component (not one global %) because a flight controller's risk is not
uniform. The estimator (`est`) and calibration math (`calib`) are well tested
and must not be allowed to rot; `control`/`actuator` are the P1 targets and
start low on purpose. A single global number would let a regression in `est`
hide behind an improvement in, say, parser scaffolding.

This is tier 1 (host) of the two-tier coverage plan
(`firmware/docs/plans/on-hardware-test-and-coverage.md`); driver/bus/RTOS code that can
only run on the metal is covered by tier 2 (on-target gcov) and is expected to
stay low here — hence the low `sensor`/`actuator` floors.

Usage:
    cmake -S sim/host -B build_cov -DVAYU_COVERAGE=ON
    cmake --build build_cov --target coverage-gate          # runs this
  or by hand:
    gcovr --root <repo> --filter '<repo>/firmware/src/' --json-summary -o cov.json
    python3 tools/coverage_gate.py firmware cov.json

Floors are line-% per top-level component dir under `firmware/src/` (e.g.
`firmware/src/control/...` -> `control`). A component present in the report but
absent from the floor table defaults to 0 (reported, never gates) so a new
directory cannot silently fail the build.

@implements CONV-05
"""
from __future__ import annotations

import collections
import json
import sys

# Baseline: the sim/host integration suite (15 ctest cases) MERGED with the
# firmware host UNIT tests (firmware/tests/host: pid/mixer/maths/fft) — gcovr
# unions both over firmware/src/. The coverage-gate target (sim/host/
# CMakeLists.txt) builds + runs both before gcovr. Floors are rounded down from
# the measured value to a small jitter band; raise them as coverage improves.
FLOORS = {
    "firmware": {
        "calib": 92.0,      # ellipsoid + engine math — keep high
        "est": 77.0,        # ekf / fusion / vertical — protected high-water
        "logger": 90.0,     # tiny (11 lines); band tolerates one new line
        "storage": 69.0,    # fs owner / xfer state machine
        "comm": 56.0,     # + rc_buffer unit test (the SPSC rings were 0%)
        "sys": 50.0,        # + float32_to_float16 unit test (math_utils)
        "sensor": 23.0,     # driver code — bulk needs tier-2 (on-target gcov);
                            # imu_buffer's rings ARE host-testable and now are
        "maths": 92.0,      # maths_interface + fft + biquad unit tests (firmware/tests/host)
        "dsp": 92.0,        # notch front-end + bank unit-tested; gyro_notch.c glue
                            # is SITL-driven (test_phase3_comm), bar its v_malloc-
                            # failure aborts (not host-coverable) -> measured ~93%
        "control": 50.0,    # pid + mixer + sysid unit-tested; angle/rate are tasks
                            # (integration-tested in sim/host, not host-unit-testable)
        "actuator": 0.0,    # P1 target: motor/esc mixing currently 0% on host
    },
}


def component_of(filename: str) -> str:
    """Map a report filename to its top-level component dir under .../src/."""
    parts = filename.replace("\\", "/").split("/")
    if "src" in parts:
        i = parts.index("src")
        if i + 1 < len(parts) - 1:  # need at least src/<comp>/<file>
            return parts[i + 1]
    return "(root)"


def aggregate(report: dict) -> dict:
    """component -> (lines_covered, lines_total), summed from per-file totals."""
    acc = collections.defaultdict(lambda: [0, 0])
    for f in report.get("files", []):
        c = component_of(f["filename"])
        acc[c][0] += f["line_covered"]
        acc[c][1] += f["line_total"]
    return {c: (cv, tot) for c, (cv, tot) in acc.items()}


def main(argv: list) -> int:
    if len(argv) != 3 or argv[1] not in FLOORS:
        print(f"usage: {argv[0]} {{{'|'.join(FLOORS)}}} <gcovr-json-summary>",
              file=sys.stderr)
        return 2

    suite, path = argv[1], argv[2]
    floors = FLOORS[suite]
    with open(path) as fh:
        report = json.load(fh)

    comps = aggregate(report)
    overall_cv = sum(cv for cv, _ in comps.values())
    overall_tot = sum(tot for _, tot in comps.values())
    overall = 100.0 * overall_cv / overall_tot if overall_tot else 0.0

    print(f"Coverage gate: {suite}  (overall {overall:.1f}%  "
          f"{overall_cv}/{overall_tot} lines)")
    print(f"  {'component':12s} {'lines':>11s}  {'cover':>6s}  "
          f"{'floor':>6s}  status")

    failures = []
    for comp in sorted(comps, key=lambda c: -comps[c][1]):
        cv, tot = comps[comp]
        pct = 100.0 * cv / tot if tot else 0.0
        floor = floors.get(comp, 0.0)
        ok = pct + 1e-9 >= floor
        status = "ok" if ok else "BELOW FLOOR"
        print(f"  {comp:12s} {cv:5d}/{tot:<5d}  {pct:5.1f}%  "
              f"{floor:5.1f}%  {status}")
        if not ok:
            failures.append((comp, pct, floor))

    if failures:
        print(f"\nFAIL: {len(failures)} component(s) below floor:")
        for comp, pct, floor in failures:
            print(f"  - {suite}/{comp}: {pct:.1f}% < {floor:.1f}% "
                  f"(coverage regressed — add tests, or if intentional lower "
                  f"the floor in tools/coverage_gate.py)")
        return 1

    print("\nPASS: all components at or above floor.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
