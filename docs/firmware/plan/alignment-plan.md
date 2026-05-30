# Vayu firmware — alignment plan with `docs/firmware/` standard

## Context

`docs/firmware/coding-guidelines.md` (12-section C99/C11 ruleset) and `docs/firmware/requirements.md` (three-level SYS / HLR / LLR spec) were freshly written and audited against the live codebase on 2026-05-26. The standard is committed; the codebase is partially conformant. The standard itself enumerates the work needed for full alignment:

- **CONV-01..06** in `coding-guidelines.md` §7.13 — infrastructure pieces the rules reference that don't exist yet (`vayu_status.h`, `vayu_assert.h`, `tools/trace.py`, `.clang-tidy`, coverage build, compiler-flag rollout).
- **~14 🟡 gap items** in `requirements.md` §6 — audit-derived behavioural gaps, each carrying a requirement ID.
- **R1.2, R2.1, R10.3, R12.1..R12.7** marked 🟡 in `coding-guidelines.md` — coding-standard items the current code violates.

This plan turns those into an executable phased rollout. The README's §"Next concrete steps" lists six steps in the right order; this plan fleshes them out into deliverables with dependencies. Goal: every 🟡 marker cleared and every R12 gate binding.

**Out of scope.** The Navigator GCS at `software/` has its own companion at `software/docs/requirements.md` and is not part of this plan. `extern/vaios/**` and `extern/vaios/extern/NavHAL/**` are vendored — vayu PRs do not edit them; vendor changes land via submodule bump (see vendor scope clause, `requirements.md` §2 + §5.3).

---

## Current state snapshot (verified by exploration)

- Source tree: `src/{actuator, comm, control, drivers, logger, maths, sensor, sys, utils}/`. `drivers/`, `maths/`, `utils/` don't map to a module prefix; R2.6 violated until Phase 4 folds them in.
- Only umbrella header that exists: `include/logger/logger.h`. Sensor/control/actuator/comm/sys umbrellas missing (R2.1 🟡).
- `include/vayu_status.h` and `include/vayu_assert.h` do **not** exist. Zero references in code.
- `CMakeLists.txt:69` warning flags: `-Wall` only (R1.2 🟡).
- `tools/trace.py` does **not** exist. Zero `@implements`/`@verifies` tags anywhere (vayu-owned or vendored).
- `tools/sim_host/` and `build_sitl/` already host the SITL build — the verification venue for Test (SITL) rows.
- Gap locations confirmed: `include/sys/state.h:16-17` (no validation), `src/maths/sensor_fusion.c:207-222` (no integral clamp), `src/comm/comm_processor.c:29-40` (no payload length check), `src/control/angle_rate_controller.c:334` (`v_delay(1)` polling), `src/utils/utils.c:10` (`vayu_log_queue` producer-only), `src/comm/telemetry_task.c:52` (heartbeat 900 ms, mislabelled "1 Hz").

---

## Design decisions

1. **Umbrella headers (R2.1) ship in Phase 4, not Phase 0.** Touching every include site early would balloon the foundations PR and predictably collide with Phases 2–3 (which also rewrite many of those files). Phase 0 ships only the cross-cutting headers everything else depends on.
2. **Trace gate ships in warn-only mode in Phase 1, flips to enforce in Phase 5.** *Unknown ID* enforcement runs from day one (the requirements doc is authoritative); *missing implementer/verifier* enforcement waits until Phase 2–3 PRs have built the tagging muscle. No bulk retrofit.
3. **Vendor exemption (§5.3) baked into `trace.py` from day one** — `extern/vaios/**` and `extern/vaios/extern/NavHAL/**` count for `@implements` discovery, never for `@verifies` (mark `verified-upstream`). `.clang-tidy` and warning widening also exclude those subtrees.
4. **`drivers/`, `maths/`, `utils/` get folded into module trees in Phase 4** to satisfy R2.6. Mapping is non-negotiable from §4: `drivers/i2c_manager.c` → `sensor/`; `maths/sensor_fusion.c` + `lpf.c` → new `est/`; `maths/pid.c` + `control_buffer.c` → `control/`; `utils/utils.c` (`vayu_log_queue`) → `logger/`.

---

## Phase 0 — Foundations (CONV-01, CONV-02)

**Goal.** Land the canonical error and assertion vocabulary.

**Deliverables.**
- `include/vayu_status.h` — `typedef enum { VAYU_OK, VAYU_ERR_INVALID, VAYU_ERR_TIMEOUT, VAYU_ERR_BUSY, VAYU_ERR_RANGE, VAYU_ERR_FAULT, VAYU_ERR_NOT_IMPL } vayu_status_t;` with `#pragma once` (R10.4), no transitive HAL includes, explicit underlying width per R4.4.
- `include/vayu_assert.h` — `VAYU_ASSERT(cond)` macro. Debug: trap + serial log + halt. Release: log + request `SYSTEM_STATE_FAILSAFE` (R9.2). Must be ISR-safe. Distinct from `TEST_ASSERT`.
- Status update in `docs/firmware/coding-guidelines.md` §7.13: mark CONV-01 / CONV-02 done.

**Verification.** Both `build/` (target) and `build_sitl/` (host SITL) compile clean. Add one canary `VAYU_ASSERT(true)` at `vayu_main` start so the headers are linked.

**Scope.** Small — one PR.

---

## Phase 1 — Traceability gate (CONV-03)

**Goal.** Land `tools/trace.py` so every subsequent PR declares what it implements and verifies.

**Deliverables.**
- `tools/trace.py` — walks `src/`, `extern/vaios/`, `extern/vaios/extern/NavHAL/` for `@implements` / `@verifies`; parses `MOD-SUB-NNN` rows from `docs/firmware/requirements.md` (handles `❌ (dropped: …)` and `❌ deferred — …` row syntax); emits `docs/firmware/trace.md`. CLI: default regenerates trace.md; `--check` is the CI gate. §5.3 vendor exemption hard-coded.
- `docs/firmware/trace.md` — generated, committed.
- `tools/trace.md` (or inline `--help`) — usage notes.
- Phase 1 enforcement: **unknown ID fails; missing implementer / verifier warns.** Flag default flips in Phase 5.

**Verification.** Script exits 0 on the current tree. Inject a bogus `@implements FOO-BAR-999` somewhere, run `--check`, confirm non-zero exit. Remove canary before merge.

**Scope.** Small-medium — one PR.

---

## Phase 2 — Safety cluster (4 PRs)

**Goal.** Close the four safety 🟡 gaps, in the themed slice the README suggests.

**PRs.**
- **2a — RC watchdog.** SYS-SAFE-002, COMM-RC-002. Elapsed-time tracker in `src/comm/` ibus consumer; expose `rc_has_signal()` (R3.6); raise FAILSAFE after > 1.0 s of no valid frame. SITL test: cut RC for 1 s, assert state transition.
- **2b — Estimator degraded flag.** SYS-SAFE-003, EST-MAH-002. Add `estimator_degraded` field on estimator output (touch `src/maths/sensor_fusion.c`); raise after 100 ms continuous SNS-IMU-002 rejection. Wire telemetry to emit. Unit test with fault injection.
- **2c — Allowed-transitions table.** SYS-SAFE-006. Replace `system_state_set()` in `include/sys/state.h:16-17` (the no-validation inline) with a `vayu_status_t`-returning function backed by a static `allowed_transitions[][]` table in `src/sys/state.c`. Old inline becomes a thin getter. Audit every caller (R7.5 return-value handling).
- **2d — Arm preconditions.** SYS-SAFE-005. Tighten STANDBY → ARMED gate: estimator converged (from 2b), RC valid (from 2a), calibration current, throttle stick min. Reuses table from 2c.

2a / 2b can run in parallel; 2c precedes 2d.

**Verification.** Per PR: target + SITL build clean; unit test (where applicable) green; SITL scenario green; `tools/trace.py --check` finds the new `@implements` / `@verifies` pairs.

**Scope.** Medium — 4 PRs.

---

## Phase 3 — Themed gap clusters (~9 PRs, parallel)

**Cluster 3-CTRL** (2 PRs):
- CTRL-RATE-101 — refactor `src/control/angle_rate_controller.c:334` from `v_delay(1)` polling to `vaios_queue_wait` on the IMU control queue with timeout. Preserve R8.6 (no locks in hot loop).
- EST-MAH-105 — clamp `integralFB[xyz]` per-axis at ±0.5 rad/s and reset on estimator reset in `src/maths/sensor_fusion.c:207-222`.

**Cluster 3-COMM** (4 PRs):
- COMM-CH-002 — `tx_overflow` counter surfaced through telemetry.
- COMM-CMD-002 — payload length validation in `src/comm/comm_processor.c:29-40` (current code reads `argc` then 4-byte fields without checking payload size).
- COMM-CMD-003 — `CMD_SET_PID` end-to-end: payload schema, dispatch, gain apply, persistence policy decision.
- COMM-TEL-002 — heartbeat fix in `src/comm/telemetry_task.c:52`. Current 150 ticks × 6 ms = 900 ms (= 1.11 Hz). Pick one: drop cadence to true 1 Hz or amend SYS-TEL-001 to 1.11 Hz. Decide explicitly.

**Cluster 3-SLOG** (3 PRs):
- SNS-BUF-002 — drop counter on IMU SPSC ring, surfaced via telemetry.
- LOG-TXT-002 — wire `vayu_log_queue` (`src/utils/utils.c:10`) to telemetry task consumer.
- LOG-SD-002 — wrap marker / counter on ring-buffer wrap in `src/logger/`.

**Verification.** Per PR: target + SITL build clean; unit test or SITL scenario per the Verification column on each requirement row; trace gate green. CTRL-RATE-101 specifically wants a rate-loop jitter histogram from a SITL log (target-bench verification deferred to a follow-up bench PR).

**Scope.** Medium — ~9 PRs across three independent threads, each thread one PR at a time.

---

## Phase 4 — Coding-standards rollout (CONV-06, R2.1, R2.6, R10.3)

**Goal.** Bring every owned module into structural conformance with §7.1–§7.10.

**Per-module work, in order SNS → EST → CTRL → ACT → COMM → LOG → SYS:**
- New umbrella header `include/<module>/<module>.h` containing only the module's public surface; existing per-type headers move into the source dir as private (R2.1, R2.3). `include/logger/logger.h` is the existing precedent shape.
- All `#include "<module>/<typename>.h"` call sites rewritten to `#include "<module>/<module>.h"`.
- Warning set widened per CONV-06 — flip `CMakeLists.txt:69` `-Wall` → the full R1.2 set, scoped to that module's TUs via per-file `target_compile_options`. Module is "done" when its TUs build clean with `-Werror`.
- Magic-number sweep (R10.3) within the module — promote raw literals to `static const` or `enum`. Anchor to `include/variables.h`.

**Folder consolidation lands in the relevant module's PR (R2.6):**
- `src/drivers/i2c_manager.c` → `src/sensor/`
- `src/maths/sensor_fusion.c`, `src/maths/lpf.c` → new `src/est/` (with `include/est/`)
- `src/maths/pid.c`, `src/maths/control_buffer.c` → `src/control/`
- `src/utils/utils.c` → `src/logger/`
- `src/utils/{math_utils.h, timer_callbacks.h, types.h}` → `include/sys/` or a single `include/vayu_common.h`

Tracking doc `docs/firmware/plan/warning-rollout.md` records each module's warning level.

**Verification.** Per module: both builds clean with full R1.2 flags; TUs flip to `-Werror`; trace gate re-runs (file paths shift, IDs do not); `warning-rollout.md` updated.

**Scope.** Large — one PR per module (≥ 7 PRs).

---

## Phase 5 — CI + sanitizers + coverage (CONV-04, CONV-05, R12.1..R12.7)

**Goal.** Make every §7.12 gate binding.

**Deliverables.**
- `.clang-tidy` (CONV-04) — narrow start (`bugprone-*`, `cert-*`, `readability-*`), excludes `extern/`.
- `tools/sim_host/CMakeLists.txt` — `coverage` target wrapping unit-test build under `-fprofile-arcs -ftest-coverage`; gcovr summary published (CONV-05).
- `.github/workflows/ci.yml` — jobs: (1) target build, (2) SITL build, (3) SITL with ASan + UBSan (R12.2), (4) unit tests, (5) coverage + threshold (R12.4, initial 70 % on CTRL / EST / COMM-PKT), (6) `tools/trace.py --check` in **fail-on-missing** mode (flag default flipped here, R12.5), (7) `clang-tidy` against baseline (R12.1, R12.3), (8) `cppcheck` (R12.1).
- Renode (R12.6) and state-machine SITL coverage (R12.7) wired as separate jobs once a transition-coverage harness exists; may slip to a follow-up phase if heavy.

**Verification.** A canary branch with a deliberate regression of each kind (unknown ID, missing brace, unused param, missing implementer for an active req, coverage drop) is opened against the workflow; each gate fires its expected failure.

**Scope.** Medium-large — ~3-4 PRs (clang-tidy + coverage build, GH Actions workflow, sanitizer job, gate-flip).

---

## Critical files

- `docs/firmware/coding-guidelines.md` — CONV-01..06 table; R12 gate set; status-cell updates per phase.
- `docs/firmware/requirements.md` — the 14 gap IDs in §6; §5.3 vendor exemption; §5.2 trace-gate contract.
- `include/sys/state.h` — Phase 2c rewrite of `system_state_set()` (lines 16-17); central to SYS-SAFE-005/006.
- `src/maths/sensor_fusion.c` — Phase 2b (degraded flag), Phase 3 EST-MAH-105 (integral clamp, lines 207-222).
- `src/control/angle_rate_controller.c:334` — Phase 3 CTRL-RATE-101 (wait-on-queue refactor).
- `src/comm/comm_processor.c:29-40` — Phase 3 COMM-CMD-002 (payload validation), COMM-CMD-003 (SET_PID).
- `src/comm/telemetry_task.c:52` — Phase 3 COMM-TEL-002 (heartbeat 900 ms → 1 Hz or amend spec).
- `src/utils/utils.c:10` — Phase 3 LOG-TXT-002 (`vayu_log_queue` consumer) and Phase 4 fold into `src/logger/`.
- `CMakeLists.txt:69` — Phase 4 per-module warning widening; Phase 5 SITL sanitizer / coverage variants.
- `tools/sim_host/CMakeLists.txt` — Phase 5 coverage target; SITL home for Phase 2/3 verifications.
- `tools/trace.py` *(new)* — Phase 1.
- `include/vayu_status.h` *(new)* — Phase 0.
- `include/vayu_assert.h` *(new)* — Phase 0.
- `.clang-tidy` *(new)* — Phase 5.
- `.github/workflows/ci.yml` *(new)* — Phase 5.

---

## End-to-end verification (when the whole plan lands)

- `tools/trace.py --check` exits 0 with no `🟡` markers remaining in `docs/firmware/requirements.md` §6.
- Every active requirement row has ≥ 1 `@implements` reference (vendor rows may resolve to `extern/vaios/**` or `extern/vaios/extern/NavHAL/**`).
- Every active vayu-owned row has ≥ 1 `@verifies` reference (vendor rows may be `verified-upstream`).
- `coding-guidelines.md` §7.13 has no rows without ✅ status.
- All `🟡 gap` cells in §7.1–§7.12 are cleared.
- Target build + SITL build + SITL ASan/UBSan build + coverage build all green in CI.
- Per-module TUs build with `-Werror` over the full R1.2 warning set.
- A SITL scenario exists for each `Test (SITL)` verification row exercised by Phases 2–3.
- `docs/firmware/plan/warning-rollout.md` shows every module at the full R1.2 warning level.

---

## Sequencing summary

```
Phase 0  →  Phase 1  →  Phase 2  →  Phase 3 (CTRL ∥ COMM ∥ SLOG)  →  Phase 4  →  Phase 5
  (small)   (small-med) (med 4PR)   (med ~9PR, 3 parallel threads)    (large 7+) (med-large 3-4)
```

Phases 0 and 1 must be sequential (Phase 1 depends on the status/assert vocabulary). Phase 2 must precede Phase 3 (Phase 3 consumes Phase 2 safety primitives). Phase 3 clusters are independent and can land in parallel. Phase 4 follows Phase 3 to avoid disruptive churn during gap-closure. Phase 5 follows Phase 4 because sanitizers + `-Werror` over the full set are only meaningful once Phase 4 has cleaned the baseline, and the trace-gate flip needs every prior phase's IDs already tagged.
