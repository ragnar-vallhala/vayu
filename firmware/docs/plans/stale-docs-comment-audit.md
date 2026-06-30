# Fix stale docs & code comments across the codebase (excl. `extern/`)

## Context

Two waves of change left documentation and code comments describing behavior the
code no longer has:
1. **The IMU calibration overhaul** (commits `6755073`..`22c42b2`): accel 6-point
   → pose-tolerant full-3×3 ellipsoid; gyro blind-average → stillness-gated;
   `cal.bin` v2→v3 (`acc_scale[3]`→`acc_soft_iron[9]`); new `calib_step` EDGE
   codes; shared `src/calib` module.
2. **Earlier refactors** whose comments/docs were never updated: telemetry base
   loop 166→500 Hz, TX channel buffer 512→2048 B, control loops moved to
   `task_delay_until`, EKF decimation switched to latest-sample, sysid dump moved
   SD→RAM, plus a few dead declarations.

A codebase-wide audit (Explore agents over `src/`, `include/`, `tools/`, `docs/`,
`navlink/docs/`, `navigator/`; `extern/` excluded) surfaced the items below. Goal:
make every comment/doc match the current code. These are doc/comment edits + a few
trivial dead-symbol cleanups + one dialect-enum addition; **no behavioral code
change**.

Scope note: historical `docs/journal/log-analysis/2026*` archives are
point-in-time records and are NOT edited. `navigator/src/vsim/**`
(renderer/procgen) excluded.

## Guiding principle — clean overwrite (no backward notes), except versioned systems

Default: **replace stale text with the current behavior, cleanly.** Do NOT add
"was X → now Y" / "previously" / "supersedes" history notes, and do NOT leave
tombstone comments for removed symbols — just delete them. Git history is the
changelog; the docs should read as the current truth, not a diff.

**Exception — versioned / on-disk / wire-compatibility systems**, where a prior
version must stay documented because a loader/decoder needs it:
- `cal.bin` file format: v3 keeps documenting that the old v2 layout
  (`acc_scale[3]`) is rejected-on-load — backward compat is real here.
- `calib_step` and other wire enums: **append-only** — prior values keep their
  meaning; new codes are added after them (never renumber/erase).
- NavLink message/version semantics generally.

Everything else (loop rates, buffer sizes, internal algorithm descriptions, dead
declarations, UI strings) gets a clean current-state update with no historical
residue.

---

## Part A — Calibration

### A1. `firmware/docs/reference/requirements.md` (rewrite stale algorithms)
- **SYS-CAL-002** "Accel 6-axis calibration" → pose-tolerant full-3×3 accel cal
  over ~12 holds (6 faces + 6 edges/corners), per-pose GCS progress.
- **SNS-CAL-001** "Accel offset+scale …" → accel **offset + 3×3** (`acc_soft_iron`)
  + gyro offset + mag hard/soft iron; note `cal.bin` **v3**.
- **SNS-CAL-101** (rewrite) → magnitude-only ellipsoid over ~12 stillness-gated
  poses; `calib_fit_ellipsoid` → offset + full 3×3, rescaled to `‖a‖=g`; commit
  only on success.
- **SNS-CAL-102** (rewrite) → single **stillness-gated** gyro bias capture
  (`GYRO_CAL_STILL_SAMPLES`, timeout, `GYRO_CAL_VAR_MAX`); drop "two modes/6-point".
- **SNS-CAL-103** (fix long-stale) → mag is an **online ellipsoid LS fit** (offset
  + 3×3) with raw-span per-axis coverage — not min/max.
- (SYS-CAL-001 title "bias-only" → "Gyro bias calibration"; SNS-CAL-002 auto-trim
  row is accurate, leave.)

### A2. `firmware/docs/reference/trace.md`
- Update `SYS-CAL-002` and `SNS-CAL-101` row titles to match A1 (IDs unchanged).

### A3. `navigator/docs/reference/requirements.md`
- **FR-RX-08** note "Bias-only + 6-axis + mag axis coverage" → accel 12-pose
  (faces+edges) + gyro + mag, pose/axis-coverage progress.

### A4. `navlink/docs/reference/messages/command.md`
- Line 27: drop accel "bias-only vs full"; accel is one full-3×3 routine (firmware
  ignores the mode nibble for accel/mag).

### A5. `calib_step` enum — add the new codes everywhere it is listed
- `navlink/dialect.json` `calib_step`: append `EDGE_1..EDGE_6` (11–16) after
  `FAILED`=10 (COMPLETE/FAILED already present).
- Regenerate: `python3 navlink/generate.py` (updates `navlink/generated/{c,py,html}`
  — enum constants only, no message/CRC change).
- `navlink/docs/reference/messages/system_status.md` + `navlink-v2-spec.md` (§14.8):
  extend the step tables to include 9 COMPLETE, 10 FAILED, 11–16 EDGE_1..6.

### A6. `firmware/docs/reference/pipeline-overview.md`
- Filtering step (line ~92): note accel now applies **offset + 3×3** (not
  offset+scale) and gyro applies bias subtraction, alongside the mag iron step.

### A7. GCS calibration comments/strings
- `navigator/src/ui/widgets/CalibrationWidget.cpp:35` gyro tooltip "bias-only
  zeroing" → "stillness-gated bias".
- `navigator/src/ui/widgets/CalibrationWidget.h:68` `// Axis Status (for 6-axis)` →
  reflect 3×3 / 12-pose.
- `CalibrationWidget.cpp:330-331` mode-nibble comment overstates → note firmware
  ignores it for accel/mag.

---

## Part B — Codebase-wide drift (firmware)

### B1. Telemetry base loop rate: docs say ~166/150 Hz; code is **500 Hz** (`TELEM_BASE_MS=2`, `telemetry_task.c:207`)
- `firmware/docs/reference/tasks/imu_telemetry_task.md:6` "150 Hz (6 ms)" → ~500 Hz (2 ms).
- `firmware/docs/reference/software-flow.md` lines 58, 290, 299, 338 ("@166 Hz") → ~500 Hz.
- `firmware/docs/reference/requirements.md:455` COMM-TEL-001 "~166 Hz (`v_delay(6)`)" →
  ~500 Hz (`v_delay(TELEM_BASE_MS)`, =2).
- In each, add: per-stream effective rates are set by the ms-based `TELEM_GATE`,
  independent of the base loop.

### B2. TX channel buffer: comments say 512 B; actual `CHANNEL_TX_BUF_SIZE = 2048` (cap 1280 for telemetry)
- `src/comm/channel.c:24` "active 512 B buffer" → 2048 B (1280 B telemetry cap).
  Line 4's rationale stays but reframed as the *current* justification ("2048 B
  holds the worst-case ~1336 B perf burst"), dropping the "was 512 B" framing.
- `firmware/docs/reference/requirements.md:448` COMM-CH-002 "512 B capacity" → 2048 B.
- `firmware/docs/reference/software-flow.md:67` and `:312` "ping-pong 512 B" → 2048 B.
  **Do NOT touch `software-flow.md:260` `rx_raw_buf 512 B`** — the RX ring is
  genuinely 512 (`serializer.c`).

### B3. Control-loop "blocks on …_wait()" comments are stale (loops use `task_delay_until`)
- `src/control/angle_controller.c:286-287` → "no v_delay; `task_delay_until` for a
  drift-free periodic schedule; drains freshest attitude each tick."
- `src/control/angle_rate_controller.c:473-474` → same, `task_delay_until` /
  freshest IMU. (Both already noted stale in the sitl-lockstep plan.)

### B4. EKF decimation comment
- `src/est/attitude_task.c:14-15` top-doc says accel/gyro "averaged over the
  decimation window" → code feeds the **latest** sample (no averaging) per the
  inline note at `:70-71`. Fix the top-doc to match.

### B5. sysid dump is RAM now, not SD
- `include/control/sysid.h:58-63` doc-comments reference "0:sysid.bin" / "the SD
  file" → the dump streams from the **RAM capture buffer** (`sysid.c` dump is a
  no-op on SD). Update both doc-comments.

### B6. Dead / mismatched declarations (tiny code cleanups — plain delete)
- `include/sensor/bmx160.h:241` — delete the `bmx160_raw_mag_to_uT` decl (impl
  removed).
- `include/sensor/bmx160.h:246` — delete the `bmx160_get_attitude` decl (no impl).
- `src/sensor/bmx160.c:863` — delete the unused `extern uint32_t bmx160_task_id`.
- `include/maths/maths_interface.h:1-2` — header guard `MATHS_SENSOR_FUSION_H` →
  `MATHS_INTERFACE_H` (match filename).

---

## Part C — GCS app

### C1. Clear factual fixes
- `navigator/src/protocol/DroneProtocol.h:9-19` — top comment still describes a
  newline-terminated **ASCII** `$IMU,...` parser; the parser is **binary NavLink
  v2** only. Replace with the v2-frame description. (`gcs-architecture.md:247`
  already flags this as known-stale.)
- `navigator/src/protocol/DroneProtocol.cpp:52` — ack string `"TEMP_REJECTED"` →
  `"TEMPORARILY_REJECTED"` to match the `command_result` enum (value 1).
- `navigator/docs/reference/requirements.md:151` (FR-SIM-04) — marked ❌/"no UI yet"
  but the **Sensor Models panel** is implemented (`SimulatorWidget` buildSensorPanel
  → `sendNoise()` / `VSIM_CTL_SET_NOISE`). Mark ✅; and `:327` drop the
  "still pending" clause.

### C2. Low-priority / judgment items (confirm before changing)
- **Phase-label breadcrumbs** in `navigator/src/ui/main/MainWindow.cpp:403,414,430`
  and `MainWindow.h:115` cite wrong phase numbers (e.g. "Phase-2 2E/2C/2A",
  "Phase-1 1C") vs the requirements phase plan. Concrete but low-value developer
  comments; fix the labels.
- **"STABILISE" vs "ANGLE"** flight-mode naming: the dialect enum names mode 0
  `ANGLE`, but the GCS labels it `"STABILISE"` in `PacketDissector.cpp:31`,
  `MainWindow.cpp:597/1155`, `MainWindow.h:205`, `SimHudWidget.h:25/39`,
  `SimulatorWidget.cpp:2666`. This reads like a **deliberate UI synonym**
  (stabilise = self-level/angle), and the project's own altitude-hold plan uses
  "STABILISE" too — so likely NOT a bug. **Recommend: leave as-is** (or, if you
  want wire/UI consistency, align all to one term). Flagged, not auto-fixed.

Clean: protocol msgids (BARO 1039, VERTICAL_STATE 1040), nav-state enums, codec
params, telemetry-engine/threading, recording/replay, feature-matrix shipped
markers — all accurate.

---

## Critical files
- **Firmware docs**: `firmware/docs/reference/requirements.md`, `firmware/docs/reference/trace.md`,
  `firmware/docs/reference/software-flow.md`, `firmware/docs/reference/pipeline-overview.md`,
  `firmware/docs/reference/tasks/imu_telemetry_task.md`
- **Firmware code/comments**: `src/comm/channel.c`,
  `src/control/angle_controller.c`, `src/control/angle_rate_controller.c`,
  `src/est/attitude_task.c`, `include/control/sysid.h`,
  `include/sensor/bmx160.h`, `src/sensor/bmx160.c`,
  `include/maths/maths_interface.h`
- **NavLink**: `navlink/docs/reference/messages/command.md`,
  `.../messages/system_status.md`, `navlink/docs/reference/navlink-v2-spec.md`,
  `navlink/dialect.json` (+ regenerated `navlink/generated/**`)
- **GCS**: `navigator/docs/reference/requirements.md`,
  `navigator/src/ui/widgets/CalibrationWidget.{cpp,h}`,
  `navigator/src/protocol/DroneProtocol.{h,cpp}`,
  `navigator/src/ui/main/MainWindow.{cpp,h}` (C2 breadcrumbs only)

## Verification
- `python3 navlink/tests/run_tests.py` passes after the dialect/codec regen
  (enum-only change → no wire/CRC change); `git diff navlink/generated` shows only
  new `EDGE_*` enum entries.
- Build firmware (`cmake --build build`) + SITL tests + GCS `tst_calibration_wizard`
  to confirm the dead-decl/header-guard cleanups don't break compilation.
- Re-run the audit greps (`6-point|6-axis|166 ?Hz|512 ?B|acc_scale|two modes|
  min/max|wait_for_orientation|0:sysid.bin`) over `src include docs navigator/docs
  navlink/docs` → clean (excluding the historical journal archives + the legit
  RX-ring 512 B references).
- Spot-read each edited comment against the code it now describes.

## Decision (resolved)
Include the dialect.json + codec regen for `calib_step` so the calib_step docs
stay consistent with their authoritative source.
