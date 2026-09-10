# Vayu Unified Test Framework — Methodology, Findings & Plan

> **Archived 2026-09-10.** Written 2026-06-19 on the (now deleted) branch it
> names, and kept for the reasoning, not as a current map: every path predates the
> monorepo restructure — `src/` is now `firmware/src/`, `software/` is `navigator/`,
> `tools/sim_host/` is `sim/host/`.
>
> Superseded in part: the coverage gate this plan proposed shipped as
> `tools/coverage_gate.py` plus the CI floors in `.github/workflows/ci.yml`.

_Status: proposed (not yet implemented). Companion to
[`coverage-analysis.md`](coverage-analysis.md). Work tracked on the
`analysis/test-coverage` branch._

---

## 1. Why this work

The test setup grew per-component and ad-hoc. It works, but it does not scale and it
hides gaps. The mandate: a **clean, in-depth, growable** framework across
C / C++ / Python where **one binary per module** replaces the executable sprawl,
**adding a test is dropping a file**, and **one command runs everything** — coverage,
cleanliness and scalability all improving together. Dismantling the current structure is
on the table.

---

## 2. Methodology

How the current state was established (all on the `analysis/test-coverage` worktree, clean
rebuilds to avoid stale-artifact false positives):

1. **Inventory** every test target and how it is declared:
   `software/tests/CMakeLists.txt` (18 `navigator_test` blocks), `tools/sim_host/CMakeLists.txt`
   (6 C tests), `navlink/tests/`, `software/headless-sdk/tests/`.
2. **Classify each test** by mechanism: GCS = QtTest `QObject` + `QTEST_{APPLESS,GUILESS,}_MAIN`
   (three app flavors); firmware = plain C with per-file `g_checks/g_fails`; Python =
   `unittest` (navlink) vs `pytest` (headless-sdk).
3. **Trace the build coupling**: confirmed the GCS app is `qt_add_executable(Navigator …)`
   over all 77 sources with **no shared library**, which forces every test to re-list the
   `src/*.cpp` it links — the structural root cause.
4. **Measure coverage honestly** per component with clean instrumented builds (gcov/gcovr,
   coverage.py), using full-codebase denominators (files never linked into a test are 0%,
   not invisible).
5. **Validate the keystone refactor** (extract `navigator_core`) line-by-line against
   `software/CMakeLists.txt` before proposing it, including the AUTOMOC/AUTORCC edge cases.

---

## 3. Findings

### 3.1 GCS — sprawl driven by a missing library
- **18 executables**, each a hand-written `navigator_test(name …)` block that re-lists its
  source files, include dirs, and links. ~120 lines of repetitive CMake.
- Cause: no core lib. `qt_add_executable(Navigator …)` compiles all sources directly
  (`software/CMakeLists.txt:236`). Tests can't link the app, so they recompile slices of it.
- Per-test drift: `${SRC}/autotune`, `${SRC}/ui/widgets`, `${REPLAY}`, `navlink_codec`,
  `ITelemetrySource.h` are re-added in scattered subsets — easy to get wrong, noisy to read.
- Validated fix is low-risk: `main()` is isolated at `src/app/main.cpp:8`; the only real
  gotcha is the single `.qrc` (keep it on the exe); making the lib's includes/Qt links
  `PUBLIC` lets the exe and every test inherit them.

### 3.2 Firmware C — no shared harness
- 6 executables; each re-implements an assert counter (`g_checks/g_fails`). No common
  `CHECK`/`EXPECT` vocabulary.
- 2 of the 6 are split for **real** reasons (compile flags), not noise:
  `test_phase0_assert` (`-DNDEBUG`, must not link `vayu_sitl_core`) and
  `test_phase3_est_ekf` (`-DEKF_SELFTEST`). The other 4 are mergeable.

### 3.3 Python — one clean, one outlier
- `headless-sdk` is the model: `pyproject.toml` with `[tool.pytest.ini_options]`, an
  `integration` marker, `unit/` + `integration/` split, `conftest.py`, pytest-cov.
- `navlink` is the outlier: a single `unittest` file run by hand via coverage.py; no config.

### 3.4 Coverage baseline (full-codebase denominators)
| Component | Cover | Notable |
|---|---|---|
| Firmware (C) | 28.9% | est 73% good; control/actuator ~0–17% (inverted risk) |
| GCS (C++) | 8.5% | replay 89%, autotune 41%; ui/vsim ~0–4% |
| navlink codec (Py) | codec 99% | total 70% dragged by `generate.py` codegen |
| headless-sdk (Py) | 36% | `session.py` 13%, `client.py` 16% |

A per-component **coverage gate already exists** (`tools/coverage_gate.py`, wired into CI) and
must keep passing — this refactor is the foundation that lets us raise those floors cleanly.

---

## 4. Plan (phased; each phase independently green & committable)

**Phase 1 — Extract `navigator_core` (keystone, no test changes).**
In `software/CMakeLists.txt`: drop `src/app/main.cpp` from `GCS_SOURCES`;
`add_library(navigator_core STATIC …)`; move include dirs + Qt/navlink/SITL/PulseAudio links
onto the lib as `PUBLIC`; `Navigator` exe becomes `main.cpp` + `.qrc` + `navigator_core`.
Verify: app + all 18 existing tests build & pass unchanged; gate green.

**Phase 2 — GCS per-module runners.**
New `software/tests/framework/`: `vtest.h` (a `VTEST("key", Class)` registration macro
replacing `QTEST_*_MAIN`) + `vtest_main.cpp` (generic dispatch `main` — `argv[1]`=key →
`QTest::qExec`; `--list`; no-arg = run all). Convert the 18 tests to `VTEST`, move them into
`tests/{autotune,protocol,replay,core,ui}/`. One CMake helper `vayu_qt_test_module(<module>)`
globs the dir, builds `tst_<module>` linking `navigator_core` + `Qt6::Test`, and registers one
ctest per file. **Result: 5 binaries, ~18 ctest entries, ~6 lines of CMake.**

**Phase 3 — Firmware C harness + grouped runner.**
`tools/sim_host/tests/vtest.h` (dependency-free `VT_CHECK`/`VT_EXPECT_*`). Merge the 4 vanilla
tests into one arg-dispatched `test_sitl`; keep the 2 flag-special ones separate but on the
same harness. Small `vayu_c_test()` helper. **Result: 3 binaries instead of 6.**

**Phase 4 — Python standardization.**
Give `navlink` a `pyproject.toml`/pytest config (pytest runs its `unittest` cases natively);
scope codec coverage away from the `generate.py` codegen. Align `headless-sdk` coverage keys.
Both emit coverage JSON.

**Phase 5 — Unified runner + gate + CI.**
`tools/test.py <firmware|gcs|py|all>`: one entry point that builds, tests, emits coverage JSON,
and calls the gate; `all` prints one table. Extend `coverage_gate.py` to also read coverage.py
JSON with `python` floors. Replace the inline CI scripts with `tools/test.py <suite>`.

---

## 5. How this boosts the three goals

### 5.1 Coverage
- **`navigator_core` is instrumented once.** Today coverage % depends on which slices each
  test happened to compile; after extraction the whole GCS codebase is measured against one
  consistent denominator, so the gate reflects reality and can't be gamed by re-listing.
- **Module dirs make gaps legible.** `tests/ui/` being thin is obvious at a glance; a missing
  module dir is a visible hole.
- **Python gets gated too.** Phase 5 brings `navlink`/`headless-sdk` under the same
  fail-on-regression ratchet that firmware/GCS already have — coverage stops silently rotting
  in the languages the gate can't currently see.
- **Lower friction → more tests get written.** When adding a test is one file (next point),
  the control-loop P1 / packet-path P2 gaps get filled faster. The framework is the
  precondition for actually raising the floors.

### 5.2 Cleanliness
- **Executables: 24 → ~8** (GCS 18→5, firmware 6→3).
- **GCS test CMake: ~120 lines → ~25** (one helper + five one-line module calls); per-test
  source/include/link re-listing **eliminated** — tests link `navigator_core`.
- **One harness vocabulary per language** (`VTEST` for C++, `VT_CHECK` for C, pytest for
  Python) instead of three QtTest main-macro variants and six bespoke C counters.
- **One way to run anything**: `tools/test.py` instead of remembering per-component cmake
  invocations and `QT_QPA_PLATFORM=offscreen` incantations.

### 5.3 Scalability
- **Add-a-test = drop-a-file.** A new `tests/<module>/tst_*.cpp` with a `VTEST(...)` line is
  auto-globbed into its module binary and auto-registered in ctest — **zero CMake edits**.
  (Smoke-tested in the plan: add `tests/core/tst_smoke.cpp`, reconfigure, see `gcs.core.smoke`.)
- **New module = new dir + one helper call.** Growth is linear and uniform, not a fresh
  hand-wired block each time.
- **Faster, less brittle builds.** Core compiles once instead of being recompiled across 18
  targets; fewer link steps; no chance of a test missing a source another test added.
- **Cross-language CI stays one-liners.** `tools/test.py <suite>` keeps CI legible as the
  suite grows, and the gate scales by editing one floor table.

---

## 6. Risks & mitigations
- **qrc-in-static-lib** (resource init dropped) → keep `resources.qrc` on the exe (no test uses
  `:/` resources). Validated.
- **Header-only `Q_OBJECT` bases** (`ITelemetrySource.h`) AUTOMOC under the lib → verify the
  two replay tests build in Phase 2 (expected fine; moc now comes from `navigator_core`).
- **Big-bang risk** → phased; every phase keeps its suite green and the gate passing before the
  next. Commit per phase.
- **Floors must not drop** → the refactor changes plumbing, not assertions; existing floors stay
  the source of truth.

## 7. Verification
Per phase: that suite builds clean, its ctest/pytest is fully green, its coverage gate passes.
Final: `python3 tools/test.py all` → firmware ctest (3 bins) + GCS ctest (5 bins/~18 entries) +
navlink/headless pytest all green, all gates at/above floors; growability smoke test passes.
