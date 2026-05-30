# Vayu — Firmware Engineering Standard

This subdirectory holds the engineering standard for the vayu
flight-control firmware: what the firmware must do, how it is allowed
to be coded, and how the two trace to each other.

A separate companion lives at
[`software/docs/requirements.md`](../../software/docs/requirements.md)
for the Navigator GCS.

## Contents

- **[`requirements.md`](requirements.md)** — the spec. Three-level
  hierarchy (SYS → HLR → LLR), nine module prefixes, drafted SYS-level
  entries plus seed HLR / LLR per module. Reserved ID ranges so new
  requirements have unambiguous homes.

- **[`coding-guidelines.md`](coding-guidelines.md)** — the pragmatic
  C99/C11 subset. Twelve sections (R1–R12) covering toolchain, naming,
  types, memory, control flow, concurrency, error handling,
  preprocessor, comments, tooling/CI. Adapted from the PX4-grade
  template with vayu-specific tweaks.

## How to use these documents

**When adding a feature:**

1. Find or write the requirement(s) it satisfies in `requirements.md`.
   Pick the right module prefix (see §2 of that doc).
2. Tag the implementation file:
   ```c
   /**
    * @implements CTRL-RATE-001, CTRL-RATE-101
    */
   ```
3. Tag the unit / SITL test:
   ```c
   /* @verifies CTRL-RATE-001 */
   ```
4. CI's trace gate (`tools/trace.py`, planned — see
   `coding-guidelines.md` §7.13 CONV-03) fails the build if any ID is
   unimplemented, unverified, or unknown.

**When reviewing a PR:** check that every claim in the code is backed
by either a status code path or an assertion (R7.2, R9.1, R9.2). Check
that any new requirement IDs exist in `requirements.md`. Check the
layering rule (R2.4).

**When the spec and the code disagree:** treat the spec as the contract
unless the disagreement is intentional, in which case update the spec
in the same PR. Drift is a defect.

## Other firmware documentation in this repo

The existing per-area docs are still authoritative for their domains;
the requirements doc references them rather than duplicating their
content:

- [`docs/sensor_fusion/`](../sensor_fusion/) — Mahony filter derivation
  and tuning. Owns the math behind `EST-MAH-*` requirements.
- [`docs/telemetry/`](../telemetry/) — wire format authority. Owns the
  packet layout referenced by `SYS-TEL-004` and `COMM-PKT-*`.
- [`docs/state_machine/`](../state_machine/) — high-level system states
  and transitions. Owns the diagrams referenced by `SYS-SAFE-005` and
  `CTRL-ARM-001`.
- [`docs/tasks/`](../tasks/) — RTOS task layout. Owns the runtime view
  referenced by `VOS-*` and the `CTRL-RATE-101` trigger contract.
- [`docs/coordinate_ref.md`](../coordinate_ref.md) — NED conventions.
  Owns the axis definitions referenced by `EST-*` and `CTRL-MIX-*`.
- [`docs/in-app-sim.md`](../in-app-sim.md) — SITL architecture.
  Verification target for any `Test (SITL)` row.
- [`docs/changelog/`](../changelog/) — historical record of firmware
  changes.
- [`docs/note_to_myself.md`](../note_to_myself.md) — survival notes
  (e.g. the "no locks in high-frequency loop paths" rule lifted into
  R8.6).

## Status snapshot

This is a starter draft. The shape is committed; the content needs to
grow as code matures.

| Area                              | State                                                        |
|-----------------------------------|--------------------------------------------------------------|
| Requirement structure + IDs       | ✅ defined                                                   |
| Module taxonomy (9 prefixes)      | ✅ defined                                                   |
| Verification methods              | ✅ defined                                                   |
| SYS-level requirements            | ✅ drafted from audit; ~25 entries across SAFE/STATE/TIM/CTRL/TEL/CAL/PWR |
| Per-module HLR + LLR              | ✅ audit-anchored; ~80 entries across SNS/EST/CTRL/ACT/COMM/LOG |
| Reality vs spec drift             | ✅ §6 captures every correction made by the audit            |
| 🟡 cleanup backlog                | ✅ enumerated in §6 — ~14 tracked gap items                  |
| Traceability tag convention       | ✅ defined                                                   |
| Trace-gate CI script              | ❌ planned (`tools/trace.py`) — see CONV-03                  |
| Coding guidelines R1–R12          | ✅ adopted                                                   |
| Compiler-flag rollout (R1.2)      | 🟡 currently `-Wall` only; widening tracked as CONV-06       |
| Sanitizer CI                      | ❌ planned                                                   |
| `vayu_status_t` + `VAYU_ASSERT`   | ❌ planned — see CONV-01 / CONV-02                           |
| Existing `docs/{sensor_fusion,telemetry,state_machine,tasks}` | ✅ in place; referenced from here   |

## Next concrete steps

1. ✅ **Code audit pass** — done 2026-05-26. See `requirements.md` §6.
2. **Burn down the 🟡 cleanup backlog** — `requirements.md` §6 lists
   ~14 audit-derived gaps. Each carries a requirement ID. Pick a
   coherent slice (e.g. *RC watchdog + estimator-degraded flag +
   state-transition guard* makes a safety-themed PR).
3. **Author `include/vayu_status.h` and `include/vayu_assert.h`** —
   the foundation R7.5, R9.1, R9.2 lean on.
4. **Write `tools/trace.py`** — the CI gate. Without it the
   `@implements` / `@verifies` tags are advisory; with it they're
   binding.
5. **CI scaffolding** — GitHub Actions workflow: host build + unit
   tests + trace gate on every PR.
6. **Compiler-flag rollout** — module-by-module, enable the full R1.2
   warning set and fix the fallout.
