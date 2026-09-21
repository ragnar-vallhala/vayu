# Vayu — C Coding Guidelines

Pragmatic C99/C11 subset for the vayu flight-control firmware. Adapted
from a PX4-grade template
which itself borrows from MISRA C:2012, CERT C, the Barr Group Embedded
C Coding Standard, and NASA JPL's Power of Ten.

We are **not** formally MISRA / DO-178C compliant. We adopt the rules
that yield the highest return on engineering effort while staying
within reach of a small team. Every rule has a rationale; rationales
matter when a rule is uncomfortable.

> **Status convention.** A `🟡 gap` marker on a rule means current
> vayu code violates it. These gaps become tracked cleanups (see
> [`requirements.md`](requirements.md) §6). Rules without a marker are
> considered enforced (or trivially enforceable).

Companion to [`requirements.md`](requirements.md). Together they make
up the firmware engineering standard.

---

## 7.1 Language and toolchain

| ID    | Rule | Status |
|-------|------|--------|
| R1.1  | The codebase shall target C11 with GNU extensions disabled in upper layers (HAL and below may use vendor extensions where unavoidable). |  |
| R1.2  | All translation units shall compile cleanly with: `-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wcast-align -Wpointer-arith -Wstrict-prototypes -Wmissing-prototypes -Wundef -Wfloat-equal -Werror`. | ✅ Phase 4 complete — all 7 owned modules (ACT, EST, SNS, CTRL, COMM, LOG, SYS) compile under the full set with `-Werror` in the target build; vendor headers `-isystem`-exempt (§5.3). |
| R1.3  | Optimisation level for release shall be `-Os` or `-O2`; `-O3` is prohibited without a documented justification. Current vayu uses `-O2`. |  |
| R1.4  | Compiler version and flags shall be pinned in the build system and reproducible. Target chain: `arm-none-eabi-gcc` for Cortex-M4; `gcc`/`clang` for host SITL. |  |
| R1.5  | Cortex-M4 target builds shall use `-mfpu=fpv4-sp-d16 -mfloat-abi=hard -fsingle-precision-constant`. Already pinned in `CMakeLists.txt`. |  |
| R1.6  | Target MCU is **STM32F401RE** (Cortex-M4, 84 MHz, single-precision FPU). Any code that assumes a different MCU shall be gated by a HAL-layer `#ifdef`. |  |

---

## 7.2 File and module structure

| ID    | Rule | Status |
|-------|------|--------|
| R2.1  | Each module exposes exactly one public umbrella header in `include/<module>/<module>.h` (e.g. `include/sensor/sensor.h`) and may have any number of private headers in its source directory. | ✅ Phase 4 — every owned module has its umbrella: actuator.h, est.h, sensor.h, control.h, comm.h, logger.h, sys.h. |
| R2.2  | Public headers must be self-contained (compile when included in isolation) and idempotent (`#pragma once` or a unique include guard). |  |
| R2.3  | A public header must not include another module's *private* header. |  |
| R2.4  | **Layering rule.** Dependencies flow downward only: `LOG → COMM → CTRL → EST → SNS → SNS-drivers → HAL → VOS`. SYS sits above CTRL; ACT depends on CTRL + HAL. No upward edges. |  |
| R2.5  | A `.c` file shall not exceed 1000 lines; split otherwise. |  |
| R2.6  | Module boundary IDs `MOD-` in `requirements.md` must match the directory layout in §2 of that doc. |  |

---

## 7.3 Naming

| ID    | Rule |
|-------|------|
| R3.1  | Public symbols use the module prefix: `hal_imu_read()`, `vaios_queue_post()`, `ctrl_rate_step()`. |
| R3.2  | Static (file-local) symbols have no prefix but use `snake_case`. |
| R3.3  | Types use the `_t` suffix: `imu_sample_t`, `rate_setpoint_t`, `motor_cmd_t`. |
| R3.4  | Macros and compile-time constants are `UPPER_SNAKE_CASE`. |
| R3.5  | Single-letter names are permitted only for loop indices (`i, j, k`) and small math variables (`x, y, z`). |
| R3.6  | Boolean-returning functions read as predicates: `imu_is_valid()`, `rc_has_signal()`. |

---

## 7.4 Types

| ID    | Rule |
|-------|------|
| R4.1  | Use fixed-width integer types from `<stdint.h>` exclusively: `uint8_t, int16_t, uint32_t`, …. Plain `int, long, short` are prohibited in interfaces and storage. Loop counters may use `size_t` or `int` when the range is bounded by a literal. |
| R4.2  | Use `bool` from `<stdbool.h>` for boolean values; do not use integer 0 / 1. |
| R4.3  | Use `float` in control and estimation code; `double` only where dynamic range demands it, with a comment justifying the use. The Cortex-M4 FPU is single-precision; `double` is software-emulated and prohibitively slow on the rate loop. |
| R4.4  | Enumerations stored or transmitted must have an explicit underlying width — declare values that fit a documented type. |
| R4.5  | Avoid bit-fields; use explicit masks and shifts on `uint32_t` for register access. |
| R4.6  | Vector/quaternion math uses `float[3]` / `float[4]` (or named structs over those) — see `docs/coordinate_ref.md` for axis conventions. |

---

## 7.5 Memory and allocation

| ID    | Rule |
|-------|------|
| R5.1  | Dynamic allocation (`malloc`, `calloc`, `free`) is prohibited after the RTOS scheduler is started. |
| R5.2  | If allocation is used during init, it must come from a single statically declared pool whose size is checked at compile time. |
| R5.3  | Stack frames in any task shall be bounded; recursion is prohibited. |
| R5.4  | Variable-length arrays (VLAs) are prohibited. |
| R5.5  | All buffers crossing module boundaries must be passed with an explicit length parameter; null-terminated strings are restricted to logging output. |

---

## 7.6 Control flow

| ID    | Rule |
|-------|------|
| R6.1  | All loops shall have a statically determinable upper bound (JPL Power-of-Ten rule 2). For loops awaiting an external event, use a timeout. |
| R6.2  | `goto` is prohibited except for single-target cleanup at function end (Linux-kernel style). |
| R6.3  | Every `switch` on an `enum` shall either cover every value or have a `default` that asserts or returns an error. |
| R6.4  | `if`, `else`, `for`, `while`, and `do` bodies are always braced, even for a single statement. |
| R6.5  | Functions shall have a single point of return where it improves readability; early returns for guard clauses are permitted. |
| R6.6  | Functions shall be 50 statements or fewer where feasible (JPL rule 4). Long functions need a code-review note explaining why. |

---

## 7.7 Functions and parameters

| ID    | Rule |
|-------|------|
| R7.1  | Every function has a prototype before first use, in the appropriate header. |
| R7.2  | Public functions shall validate all input parameters at entry; assertions are for programming errors (precondition violations), returned status codes are for runtime conditions (e.g. queue empty, transient fault). |
| R7.3  | Pointer parameters shall be marked `const` where the pointee is not modified. |
| R7.4  | Output parameters appear after input parameters in the parameter list. |
| R7.5  | Functions shall return a status code (`typedef enum { … } vayu_status_t`) where failure is possible; the return value shall not be ignored at call site. Annotate with `__attribute__((warn_unused_result))` on GCC/Clang. |
| R7.6  | A canonical `vayu_status_t` lives in `include/vayu_status.h` (to be created — see §7.13 below). |

---

## 7.8 Concurrency and RTOS

| ID    | Rule |
|-------|------|
| R8.1  | Shared mutable state shall be protected by exactly one documented mechanism (mutex, queue, atomic). Mixing mechanisms on the same data is prohibited. |
| R8.2  | No RTOS API call may block while holding a mutex; lock scope shall be as small as possible. |
| R8.3  | Critical sections (interrupt disable) shall not exceed 5 microseconds and shall be commented with the worst-case duration. |
| R8.4  | ISRs shall do the minimum work and defer processing to a task via a queue or semaphore. Floating-point operations in ISRs are permitted only if the FPU context is saved by the port layer (vayu's Cortex-M4 port does this). |
| R8.5  | `volatile` is used only for memory-mapped registers and objects modified by ISRs. It is not a substitute for synchronisation. |
| R8.6  | See [`hardware-gotchas.md`](hardware-gotchas.md): **no locks in high-frequency loop paths**. The rate loop, IMU ISR, and motor task must not contend on any mutex. Use lock-free queues or single-writer/single-reader buffers. |

---

## 7.9 Error handling

| ID    | Rule |
|-------|------|
| R9.1  | Status codes are checked at every call site that can fail; ignoring an error is a defect. |
| R9.2  | `VAYU_ASSERT(cond)` is used for invariants that must hold by construction (programming errors). In release builds it shall transition the vehicle to a safe state (LAND if airborne, motor shutdown otherwise) rather than silently continuing. The macro lives in `include/vayu_assert.h` (to be created — see §7.13). |
| R9.3  | Errors are logged with a numeric code and originating subsystem; freeform strings are restricted to debug builds. |
| R9.4  | A defined error code shall not be redefined or reused with a different meaning. |
| R9.5  | `errno`-style global error state is prohibited; status codes are explicit on each call. |

---

## 7.10 Preprocessor

| ID    | Rule |
|-------|------|
| R10.1 | Function-like macros are avoided when an `inline` function would suffice. Macros that exist must be parenthesised and use `do { … } while (0)` when they expand to statements. |
| R10.2 | `#ifdef` for feature selection is restricted to the HAL layer and the build-configuration header (`include/vaios_app_config.h` and equivalents). Upper layers select features through link-time stubs or runtime config, not preprocessing. |
| R10.3 | Magic numbers in code are prohibited; use named constants (`static const` or `enum`). | ✅ Phase 4 — per-module sweep done (e.g. NUM_MOTORS, MS_PER_SECOND, discrete-selector integer compares). Ongoing discipline for new code. |
| R10.4 | `#pragma once` is preferred over include guards for new headers. |

---

## 7.11 Comments and documentation

| ID    | Rule |
|-------|------|
| R11.1 | Every public function has a Doxygen comment describing purpose, parameters, return value, preconditions, and the requirement IDs it implements (`@implements`). |
| R11.2 | Comments explain *why*, not *what*. Code that needs a 'what' comment should be rewritten or named more clearly. |
| R11.3 | `TODO` / `FIXME` comments shall include an issue tracker ID (or, until we have an issue tracker, a `requirements.md` ID). Stream-of-consciousness `// Wait, I need to …` is prohibited (see PR audit history). |
| R11.4 | Commented-out code is prohibited in committed code; delete it — the VCS keeps history. |
| R11.5 | Doxygen output is not gated on; but the tags must be parseable so the trace script can find `@implements` / `@verifies`. |

---

## 7.12 Tooling and CI gates

| ID    | Rule |
|-------|------|
| R12.1 | CI shall run on every PR and shall fail the build on any of: compiler warnings (per R1.2), `clang-tidy` violations, `cppcheck` violations (including MISRA addon where adopted), failed unit tests, drop in coverage. | ✅ wired — `.github/workflows/ci.yml`. Always-on gates: target build (`-Werror`), trace `--check`, cppcheck. SITL gates (tests, ASan/UBSan, coverage) auto-activate once the host test harness lands with the vsim integration. clang-tidy is now an always-on gate (CONV-04), split across the `clang-tidy` job (firmware + sim databases) and the `navigator` job (Qt6 build + its 41 ctest cases + the same gate over its own database); together they leave no non-vendored source unanalysed. |
| R12.2 | Host-side unit-test builds shall enable AddressSanitizer (`-fsanitize=address`) and UndefinedBehaviorSanitizer (`-fsanitize=undefined`). | ✅ `VAYU_SANITIZE` option + CI `sanitizers` job; the 5-suite ctest runs ASan+UBSan clean. |
| R12.3 | A static-analysis baseline shall be maintained; new violations are not allowed, existing ones are tracked and burned down. | ✅ both analysers hard-gate CI. cppcheck (`--error-exitcode=1`) — fixed a real null-deref (calibration_task) + a wrong-IRQ-detach in del_handler, suppressed one false positive. clang-tidy: baseline burned down across all 240 non-vendored sources (ARM **and** host), `WarningsAsErrors: '*'`, no `continue-on-error`. The host-build parse caveat that held this at 🟡 was the cause, not a caveat: CI ran the firmware against the *SITL* compile database, which does not contain 11 of the sources. `tools/dev/run_clang_tidy.sh` drives every database and ends by naming any source none of them covered — locally that list is empty, and the two CI jobs' lists have no file in common. |
| R12.4 | Coverage reports (`gcov`/`lcov`) are published per build; branch coverage on `CTRL`, `EST`, and `COMM-PKT` modules shall not drop below an agreed threshold (initial target 70 %, growing). | 🟡 mechanism landed (gcov + gcovr `coverage` target/CI job); 70 % threshold deferred until integration tests exercise CTRL/EST/COMM (unit suite ≈ 26 % today). |
| R12.5 | Every requirement ID referenced in code or tests must exist in `docs/firmware/requirements.md`; the `tools/dev/trace.py` CI step (to be written) fails the build otherwise. | ✅ CI `trace-gate` runs `tools/dev/trace.py --check` — unknown IDs fail the build. (Fail-on-missing-implementer stays warn: most requirements are unimplemented future features.) |
| R12.6 | Renode-based ISR/timing tests run on every PR that touches `HAL`, `SNS-IMU`, or `CTRL-RATE`. | 🟡 deferred — needs a Renode harness (heavy); tracked as a follow-up. |
| R12.7 | SITL coverage of the state machine — every transition in `docs/state_machine/` shall be exercised by at least one SITL test. | 🟡 deferred — needs a state-transition coverage harness; follow-up. |

---

## 7.13 Conventions vayu must adopt (not yet present)

These are pieces of infrastructure that the rules above reference but
that don't exist in the codebase yet. They become tracked work items.

| ID      | Item                                | Notes                                                                                                                       |
|---------|-------------------------------------|-----------------------------------------------------------------------------------------------------------------------------|
| CONV-01 | `include/vayu_status.h`             | ✅ landed. Canonical `vayu_status_t` enum with `VAYU_OK`, `VAYU_ERR_INVALID`, `VAYU_ERR_TIMEOUT`, `VAYU_ERR_BUSY`, `VAYU_ERR_RANGE`, `VAYU_ERR_FAULT`, `VAYU_ERR_NOT_IMPL`. `_Static_assert` bounds width to `int32_t`. |
| CONV-02 | `include/vayu_assert.h`             | ✅ landed. `VAYU_ASSERT(cond)` macro forwarding to `vayu_assert_fail()` in `src/sys/assert.c`. Debug: log + `v_panic` trap. Release (`NDEBUG`): log + request `SYSTEM_STATE_FAILSAFE` + halt calling task. Distinct from `TEST_ASSERT`. |
| CONV-03 | `tools/dev/trace.py`                    | ✅ landed (warn-only mode, per Phase 1). Walks `firmware/src`, `extern/vaios/`, `extern/vaios/extern/NavHAL/`, parses `@implements` / `@verifies`, produces `docs/firmware/trace.md`. `--check` fails on unknown ID; missing implementer / verifier currently warn. Flip to fail-on-missing lands in Phase 5 (R12.5). |
| CONV-04 | `.clang-tidy` baseline              | ✅ landed and **flipped to failing**, as planned. `.clang-tidy` (bugprone-* + cert-*, noisy checks disabled with the reason recorded per family); scope is the whole tree via `tools/dev/run_clang_tidy.sh`, not firmware/src alone. CI pins `clang-tidy-18` so a toolchain bump cannot land as a red build on an unrelated PR. |
| CONV-05 | Host SITL coverage build            | ✅ landed. `sim/host` `VAYU_COVERAGE` option + `coverage` (human summary) and `coverage-gate` (per-component floor ratchet via `tools/coverage_gate.py`, fails on regression) targets run the unit suite under gcov; CI `coverage` job enforces. Baseline + floors in [`testing/coverage.md`](../testing/coverage.md). |
| CONV-06 | Compiler-flag widening rollout      | ✅ Phase 4 complete. All 7 owned modules flipped to the full R1.2 warning set with `-Werror`. |

---

## Appendix A — Recommended free tools

| Purpose          | Tool(s)                                              |
|------------------|------------------------------------------------------|
| Unit testing (C) | Existing: `navtest` (in `extern/vaios/extern/NavHAL/include/navtest/`), `framework.h` (in `extern/vaios/tests/`). Consider Unity + Ceedling + CMock for the vayu-owned `src/` once host build matures. |
| Static analysis  | `clang-tidy`, `cppcheck` (+ MISRA addon), `scan-build`, Infer |
| Sanitizers       | ASan, UBSan, TSan, MSan (gcc/clang)                   |
| Memory checking  | Valgrind (host build only)                            |
| Coverage         | gcov + lcov + gcovr                                   |
| Fuzzing          | libFuzzer, AFL++                                      |
| Formal methods   | CBMC (PID anti-windup), Frama-C (EVA, WP)             |
| MCU emulation    | Renode (already used) |
| Simulation       | In-app C++/OpenGL sim (current — see [`navigator/docs/requirements.md`](https://github.com/ragnar-vallhala/vayu-navigator/blob/main/navigator/docs/reference/requirements.md) FR-SIM-*); legacy Gazebo/JSBSim if needed |
| Requirements     | Plain Markdown today; consider Doorstop / StrictDoc / OpenFastTrace if scale demands |
| CI               | GitHub Actions (host build + unit tests + trace gate); target-arch builds run in containers |

---

## Appendix B — References

- RTCA DO-178C, *Software Considerations in Airborne Systems and Equipment Certification*
- MISRA C:2012, *Guidelines for the Use of the C Language in Critical Systems*
- CERT C Coding Standard, SEI Carnegie Mellon
- Barr Group, *Embedded C Coding Standard*
- NASA JPL, *The Power of Ten — Rules for Developing Safety-Critical Code* (Holzmann)
- ARP4754A — *Guidelines for Development of Civil Aircraft and Systems*
- ARP4761 — *Guidelines and Methods for Conducting the Safety Assessment Process on Civil Airborne Systems and Equipment*

---

*This document is a starting standard. Adapt rules, numeric thresholds,
and tool selections to evolving needs; record adaptations as PR diffs
to this file rather than parallel guidelines elsewhere.*
