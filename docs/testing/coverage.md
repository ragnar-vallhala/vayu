# Host coverage — tier 1 + the ratchet gate

This is **tier 1** (host) of the two-tier coverage plan in
[`../plans/on-hardware-test-and-coverage.md`](../plans/on-hardware-test-and-coverage.md):
gcov line coverage of the *portable* firmware logic, measured by running the
`sim/host` unit suite under instrumentation. Driver/bus/RTOS code that can only
run on the metal is **tier 2** (on-target gcov) and is expected to read low here.

## Run it

```sh
cmake -S sim/host -B build_cov -DVAYU_COVERAGE=ON
cmake --build build_cov -j
cmake --build build_cov --target coverage        # human gcovr summary
cmake --build build_cov --target coverage-gate   # + enforce the floor ratchet
```
or `tools/scripts/vayu.sh test --coverage` (runs the gate after the suite), and
the CI `coverage` job runs `coverage-gate` on every push.

## The ratchet

`tools/coverage_gate.py` reads a `gcovr --json-summary` and fails if any
component's line-% drops below a committed floor in `FLOORS`. Floors sit just
under the current measured value, so **coverage can only go up** — a regression
breaks CI. When a PR genuinely raises a component, it bumps that floor in the
same PR. Per-component (not one global %) because risk isn't uniform: `est` and
`calib` are protected high-water marks; `control`/`actuator` are the P1 targets
and start low on purpose. A component absent from `FLOORS` defaults to 0 (it's
reported but never gates) so a new directory can't silently fail the build.

## Baseline — 2026-06-28 (`chore/monorepo-restructure`, 15 ctest cases)

Overall **43.1%** (1661 / 3857 lines over `firmware/src/`).

| Component | Lines | Line % | Floor | Note |
|-----------|------:|-------:|------:|------|
| calib     | 217/234  | 92.7 | 92.0 | ellipsoid + engine math |
| est       | 454/630  | 72.1 | 72.0 | ekf / fusion / vertical — protected |
| storage   | 270/386  | 69.9 | 69.0 | fs owner / xfer state machine |
| comm      | 563/1381 | 40.8 | 40.0 | router/codec/xfer |
| sys       | 20/95    | 21.1 | 21.0 | |
| sensor    | 44/297   | 14.8 | 14.0 | driver code — bulk needs tier 2 |
| maths     | 7/55     | 12.7 | 12.0 | |
| control   | 75/693   | 10.8 | 10.0 | **P1 target** — pid/angle/rate/mixer |
| actuator  | 0/75     | 0.0  | 0.0  | **P1 target** — motor/esc, 0% on host |
| logger    | 11/11    | 100.0| 90.0 | tiny; band tolerates one new line |

## Where to push next (P1)

`control` and `actuator` are the lowest-risk-adjusted gaps: the rate/angle PID,
mixer, and motor/esc mapping are flight-critical but barely exercised on host.
Most of their logic is portable (the mixer allocate path, PID update, RC
normalisation) and can be unit-tested here cheaply; the genuinely
hardware-bound parts (PWM duty, ESC timing) belong to tier 2. Each new test
should also carry a `@verifies <REQ-ID>` tag so it shows up in
[`../reference/trace.md`](../reference/trace.md) — coverage and traceability
advance together.
