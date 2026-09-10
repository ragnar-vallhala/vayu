# Test Coverage Analysis — Vayu Stack

> **Archived 2026-09-10.** Written 2026-06-19 on the (now deleted) branch it
> names, and kept for the reasoning, not as a current map: every path predates the
> monorepo restructure — `src/` is now `firmware/src/`, `software/` is `navigator/`,
> `tools/sim_host/` is `sim/host/`.
>
> The numbers are a 2026-06-19 baseline, not current. See
> [`coverage.md`](coverage.md) for how the gate works today.

_Branch: `analysis/test-coverage` • Generated 2026-06-19_

This is a baseline coverage measurement across all four testable components of the
stack, read through an **aerospace-grade lens**: the question is not "what % is
green" but "is the flight-critical path proven, and can we trust the harness."

> **Method.** C/C++ via `gcov`/`gcovr` (instrumented `--coverage` builds in
> `build_cov/` for firmware+SITL core, `build_gcs_cov/` for the GCS). Python via
> `coverage.py` (`navlink/`, `software/headless-sdk/`). Reports reflect tests
> **actually run**, not the test suites that exist on paper.

---

## 1. Headline numbers

| Component | Lang | Reported cover | Files in suite | Files with **zero** test linkage | True coverage* |
|---|---|---|---|---|---|
| Firmware / SITL core (`src/`) | C | **28%** (649/2245 ln) | 31 / 41 | 10 | **~23%** |
| GCS / Navigator (`software/src/`) | C++/Qt | **37%** (1077/2871 ln) | 26 / 77 | **51** | **~12%** |
| navlink codec (`navlink/`) | Python | 70% total | — | — | **codec itself 99%**; 30% is `generate.py` tooling |
| headless-sdk (`software/headless-sdk/`) | Python | **36%** | — | — | 36% |

\* "True coverage" derates the reported number for source files that are never
compiled into **any** test binary (they cannot show up in a gcov report at all, so
the headline % is measured over an artificially small denominator). The gap
between "reported" and "true" is itself a finding.

**The single most important takeaway:** the **estimator is well-tested, the
controller and actuator path are not.** For a flight controller that is exactly
backwards from where the risk lives.

---

## 2. Firmware — the flight-critical path is the least tested

### Well covered (keep this bar)
| File | Cover | |
|---|---|---|
| `est/ekf.c` | **98%** | core state estimator |
| `est/ekf_selftest.c` | **98%** | |
| `comm/rc_safety.c` | **100%** | failsafe logic ✅ |
| `sys/state.c` | 100% | |
| `comm/perf_packet.c`, `logger/log_text.c` | 100% | |
| `est/sensor_fusion.c` | 58% | |
| `control/pid_config.c` | 64% | |

### Flight-critical and effectively untested ⚠️
| File | Cover | Why it matters |
|---|---|---|
| `control/angle_controller.c` | **0%** | outer-loop attitude control law |
| `control/angle_rate_controller.c` | **7%** | inner-loop rate control law — the thing that keeps it in the air |
| `control/pid.c` | **9%** | the PID kernel every loop depends on |
| `control/flight_mode.c` | **0%** | mode arbitration / arming logic |
| `actuator/motor.c` | **0%** | motor mixing |
| `actuator/esc.c` | **0%** | ESC output — the final command to the props |
| `est/attitude_task.c` | **0%** | attitude estimation task |
| `est/lpf.c` | **0%** | filter used in the control path |
| `comm/navlink_router.c` / `navlink_tx.c` | **0%** | command + telemetry plumbing |
| `sensor/bme280.c` | **0%** | baro driver (just landed) |

### Never compiled into any test (no instrumentation at all)
`rc_task.c`, `ibus.c` (RC input decode), `i2c_manager.c`, `bmx160.c` (IMU
driver), `heartbeat.c`, `boot.c`, `timer_callbacks.c`, `perf_telemetry.c`,
`main.c`, and a stray **`test_file.c`** (looks like dead scaffolding — should be
deleted or explained).

> **Aerospace read:** sensor in → estimate is solid (EKF 98%). Estimate →
> command → actuator out is a near-total blind spot. The closed loop is unproven
> end-to-end in unit tests; only the SITL integration test exercises it, and only
> along happy-path trajectories.

---

## 3. GCS / Navigator — wide, shallow, and the harness is broken

Reported 37%, but **51 of 77 source files are never linked into a test** — so
true coverage over the real codebase is roughly **12%**.

### Covered well
`core/SourceController.cpp` 100%, `autotune/Optimizer.cpp` 90%,
`autotune/Cost.cpp` 93%, `autotune/AutotuneEngine.cpp` 91%,
`protocol/TimeSyncEstimator.cpp` 96%, `core/CommandRegistry.cpp` 96%,
`replay/RecordSink.cpp` 90%, `ui/widgets/AboutDialog.cpp` 96%.

### Core data-path with **zero** coverage (not flight-critical, but the GCS's job)
`protocol/PacketDissector.cpp` (315 ln, 0%), `protocol/NavlinkRouter.cpp`
(248 ln, 0%), `protocol/DroneProtocol.cpp` (0%), `core/crc.cpp` (0%),
`TelemetryEngine.cpp`, `SerialManager.cpp`, `UdpManager.cpp`,
`PacketLogModel.cpp` — all **never linked into a test**. The packet-decode /
CRC / routing path (the most testable, most regression-prone code in the GCS) has
no unit tests at all.

### Harness health — failing/uncompilable test targets 🔴
Of 18 GCS test targets, **4 do not build or run**:

| Target | Status | Cause |
|---|---|---|
| `tst_replay_source` | link error | `ReplaySource` MOC (`staticMetaObject`, signals) not compiled into the test |
| `tst_calibration_wizard` | `BAD_COMMAND` | binary missing / link failed |
| `tst_shortcuts_editor` | Not Run | target not built |
| `tst_sitl_stack` | Not Run | target not built |

A test suite that doesn't compile is worse than no suite — it reads green in
intent and red in reality. **This is the first thing to fix.**

---

## 4. Python

- **navlink codec:** the generated codec + its `test_codec.py` are at **99%** —
  excellent. The 70% "total" is dragged down only by `generate.py`, the
  code-generator tool (17%), which is build-time, not flight-time. Realistically
  this component is in good shape; consider excluding `generate.py` from the
  headline metric or adding generator golden-file tests.
- **headless-sdk: 36%.** Weakest spots are `session.py` **13%** (283 stmts, the
  orchestration core), `client.py` **16%**, `world.py` **15%**,
  `autopilot.py` **22%**, `server.py` **31%**. Transport layer is healthy
  (`vsim.py` 94%, `navlink.py` 82%, `rc.py` 100%). Since this SDK is the
  SITL/HIL driver, its `session`/`client` core being ~15% undermines confidence
  in the integration harness itself.

---

## 5. Gap summary vs. aerospace-grade expectations

Aerospace structural-coverage practice (DO-178C lineage) layers three things this
repo does not yet have:

1. **Coverage tied to criticality, not a flat number.** Flight-critical code
   (control laws, mixer, actuator output, failsafe, mode/arming) should target
   the highest bar; UI/telemetry-display can sit lower. Today the criticality
   ordering is inverted — control/actuator are the *least* covered.
2. **Structural rigor beyond statement coverage.** Current measurement is
   line/statement only. Safety-critical decision logic (arming gates, failsafe
   triggers, mode transitions) wants **decision / MC/DC** coverage, which
   `gcovr --decisions` / branch metrics can begin to approximate.
3. **A trustworthy, enforced harness.** Four GCS targets don't build; coverage
   isn't gated in CI; "tests that exist" ≠ "tests that run." Aerospace-grade
   starts with: every declared test compiles, runs, and is measured every commit.

---

## 6. Recommended roadmap (toward the aerospace-grade framework)

**P0 — make the harness honest** — _✅ done, see below; CI gating still open_
- ~~Fix the 4 broken GCS test targets~~ — these were **stale incremental-build
  artifacts**, not source bugs: a clean reconfigure builds and passes all 18
  (`tst_sitl_stack`, `tst_replay_source`, `tst_calibration_wizard`,
  `tst_shortcuts_editor` included). Lesson: never trust a carried-over coverage
  build dir; CI must build from clean.
- ~~The firmware C tests were registered as ctest entries **in the GCS build
  tree**~~ (via the `tools/sim_host` subdir) but were `EXCLUDE_FROM_ALL` there —
  permanent "Not Run" reds. Now guarded to register only when `sim_host` is the
  top-level project (`tools/sim_host/CMakeLists.txt`). Firmware build still 6/6;
  GCS tree now lists exactly its 18 tests, all green.
- ~~Remove `test_file.c`~~ — deleted `src/utils/test_file.c` +
  `include/utils/test_file.h` (a demo "Hello How are you?" logger task, never
  spawned — its `task_create` was commented out — but globbed into the **flight
  firmware** binary). Cleaned the include + dead comment from `src/main.c`.
- **Still open:** wire `gcovr`/`coverage.py` into CI with a per-component floor
  that **fails the build on regression** (start at current numbers, ratchet up).

**P1 — cover the flight-critical loop (the real work)**
- Unit tests for `pid.c`, `angle_controller.c`, `angle_rate_controller.c`:
  step response, saturation/anti-windup, sign conventions, NaN/clamp guards.
- Unit tests for `motor.c` + `esc.c`: mixer matrix, output clamping, disarm →
  zero-throttle, idle behavior.
- Unit + decision coverage for `flight_mode.c`: every arm/disarm gate and mode
  transition, including the rejected/failsafe edges.
- Drivers (`bmx160.c`, `ibus.c`, `bme280.c`, `i2c_manager.c`): seam them behind
  the existing SITL/HAL boundary and test parse/scale/fault paths.

**P2 — GCS data path**
- Unit tests for `PacketDissector`, `NavlinkRouter`, `crc`, `DroneProtocol`,
  `TelemetryEngine` — golden-vector decode/round-trip tests. These are pure and
  cheap to cover; biggest coverage gain per hour.

**P3 — integration depth**
- Raise `headless-sdk` `session.py`/`client.py` off ~15% so the SITL harness
  driving all of the above is itself trustworthy.
- Add MC/DC-style branch coverage measurement for the P1 safety files and gate on
  it specifically.

---

## 7. Reproduce

```sh
# Firmware / SITL core (C) — sim_host must be the TOP-LEVEL project
cmake -S tools/sim_host -B build_cov -DVAYU_COVERAGE=ON && cmake --build build_cov -j
ctest --test-dir build_cov                       # 6/6
cmake --build build_cov --target coverage        # gcovr summary, incl. --decisions
gcovr --root . --filter 'src/' build_cov --gcov-ignore-errors=no_working_dir_found

# GCS (C++/Qt) — note: 4 targets currently fail to build
cmake --build build_gcs_cov -j -- -k
QT_QPA_PLATFORM=offscreen ctest --test-dir build_gcs_cov
gcovr --root software --filter 'software/src/' build_gcs_cov \
      --gcov-ignore-errors=no_working_dir_found

# Python
( cd navlink && python3 -m coverage report )
( cd software/headless-sdk && python3 -m coverage report )
```
