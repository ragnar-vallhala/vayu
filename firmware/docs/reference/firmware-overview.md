# Vayu — Firmware Engineering Standard

This subdirectory holds the engineering standard for the vayu
flight-control firmware: what the firmware must do, how it is allowed
to be coded, and how the two trace to each other.

A separate companion lives at
[`navigator/docs/requirements.md`](https://github.com/ragnar-vallhala/vayu-navigator/blob/main/navigator/docs/reference/requirements.md)
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
4. CI's trace gate (`tools/dev/trace.py` — see
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

- [`sensor-fusion/`](sensor-fusion/) — Mahony filter derivation
  and tuning. Owns the math behind `EST-MAH-*` requirements.
- [`navlink messages`](https://github.com/ragnar-vallhala/navlink/blob/main/docs/reference/messages) — wire format authority. Owns the
  packet layout referenced by `SYS-TEL-004` and `COMM-PKT-*`.
- [`state-machine/`](state-machine/) — high-level system states
  and transitions. Owns the diagrams referenced by `SYS-SAFE-005` and
  `CTRL-ARM-001`.
- [`tasks/`](tasks/) — RTOS task layout. Owns the runtime view
  referenced by `VOS-*` and the `CTRL-RATE-101` trigger contract.
- [`coordinate_ref.md`](coordinate_ref.md) — NED conventions.
  Owns the axis definitions referenced by `EST-*` and `CTRL-MIX-*`.
- [`gcs-in-app-simulator-and-world-collision.md`](https://github.com/ragnar-vallhala/vayu-navigator/blob/main/navigator/docs/journal/changelog/gcs-in-app-simulator-and-world-collision.md) — SITL architecture (`vsim_d` daemon).
  Verification target for any `Test (SITL)` row.
- [`journal/changelog/`](../journal/changelog/) — historical record of firmware
  changes.
- [`hardware-gotchas.md`](hardware-gotchas.md) — survival notes
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
| Trace-gate CI script              | ✅ shipped (`tools/dev/trace.py` + generated `trace.md`) — CONV-03 |
| Coding guidelines R1–R12          | ✅ adopted                                                   |
| Compiler-flag rollout (R1.2)      | ✅ shipped — `VAYU_R12_WARN_FLAGS` in `CMakeLists.txt` (CONV-06) |
| Sanitizer CI                      | ✅ shipped — `.github/workflows/ci.yml`                     |
| `vayu_status_t` + `VAYU_ASSERT`   | ✅ shipped — `include/vayu_status.h`, `include/vayu_assert.h` (CONV-01/02) |
| Existing `docs/{sensor_fusion,telemetry,state_machine,tasks}` | ✅ in place; referenced from here   |

## Next concrete steps

The original starter-draft action items (status/assert headers, `tools/dev/trace.py`,
CI scaffolding, compiler-flag rollout) have all shipped. The **living tracker** is
now the CONV table in `docs/firmware/coding-guidelines.md` and the gap list in
`requirements.md` §6 — consult those for what remains open.
