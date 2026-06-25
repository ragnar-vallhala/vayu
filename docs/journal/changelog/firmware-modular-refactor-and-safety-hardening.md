# Vayu Firmware — Changelog

The on-target flight-control firmware (`src/`, ARM Cortex-M, built on the vaios
RTOS + NavHAL). Newest first. Discrete bug investigations keep their own
deep-dive files — see [implement-mahony-filter-and-quaternions.md](implement-mahony-filter-and-quaternions.md)
and [implement-rc-telemetry-and-gcs-ui.md](../../../navigator/docs/journal/changelog/implement-rc-telemetry-and-gcs-ui.md).

Convention/requirement tags (`CONV-0x`, `R1.2`, `R12.3`, `COMM-RC-002`, …) refer
to the FCS requirements & coding guidelines doc
(`docs/drone_fcs_requirements_and_coding_guidelines.pdf`).

---

## 2026-05-30 → 2026-05-31 — Modular refactor, safety hardening & CI

A staged (Phase 0–5) restructuring of the firmware into self-contained modules
behind umbrella headers, with safety instrumentation, static-analysis
clean-down, and a CI gate. Each phase compiles `-Werror` (R1.2) and treats
vendor headers as `-isystem`.

### Added

- **Phase 0 — foundations (CONV-01/02):** `vayu_status` unified return/result
  type and `vayu_assert` runtime-assertion facility, used as the base contract
  for every module below.
- **Phase 1 — traceability gate (CONV-03):** `tools/trace.py`, which links
  source to requirement tags so coverage of the requirements doc is checkable
  in CI.
- **Phase 2 — safety cluster:** RC watchdog, estimator-degraded detection, and
  an arming/flight **state gate** that blocks unsafe transitions.
- **Phase 3 — CTRL cluster:** event-driven rate loop (runs on new gyro samples
  rather than free-running) with an integral clamp.
- **Phase 3 — COMM cluster:** TX-overflow counter, inbound payload validation,
  `SET_PID` command, and heartbeat handling.
- **Phase 3 — SLOG cluster:** IMU-drop counter, log-queue drain, and an SD
  wrap counter for the on-board logger.
- **Phase 5 — CI/quality (CONV-04/05, R12.\*):** GitHub Actions workflow with
  clang-tidy, coverage, and sanitizer builds.
- **Tests:** `COMM-CH-002` `tx_overflow` unit test (+ verified target build).
- **COMM-RC-002:** fast **100 ms** `rc_loss` detection with two-tier RC-loss
  timing (quick failsafe entry, separate hold/timeout stage).

### Changed

- **Phase 4 — module reorganisation behind umbrella headers**, rolled out one
  cluster at a time, each `-Werror`: **ACT**, **EST** (`src/est/`), **SNS**
  (folded `drivers/`), **CTRL** (folded `pid/` + `control_buffer`), **COMM**,
  **LOG** (extracted `vayu_log` to `logger/`), and **SYS** (tore down the old
  `utils/`). Completes the `utils/` → typed-module migration.
- **`MAX_ANGLE_CUTOFF` raised to 70°** (was tripping the attitude failsafe too
  eagerly); FCS requirements doc added alongside.

### Fixed

- **Static-analysis burn-down (R12.3):** cleared the clang-tidy baseline and the
  cppcheck baseline — including a null-dereference and a wrong-IRQ-detach bug.
- Cleared stale safety/traceability gap markers (Phase-2 markers,
  `EST-MAH-002`, `SYS-SAFE-003`) and flagged the `COMM-RC-002` threshold
  mismatch for follow-up.

---

## 2026-05-31 — GCS software-arm command path

### Added

- **`CMD_ARM` / `CMD_DISARM` over the comm link** so the GCS can arm/disarm in
  software using a 4-channel stick model (no hardware arm switch required).
- End-to-end **wire-level test** verifying the GCS arm link.

---

## 2026-05-24 — Control-loop stabilisation (SITL bring-up)

Tuning and correctness fixes surfaced while bringing the firmware up under the
software-in-the-loop simulator; all live in the on-target control path.

### Fixed

- **Rate-PID anti-windup:** integral gating + reset so the rate loops don't
  wind up while saturated.
- **Saturation handling preserves throttle** — mixer saturation no longer leaks
  extra collective lift (phantom climb on aggressive commands).
- **Yaw dropped from the `MAX_ANGLE_CUTOFF` failsafe check** (yaw angle isn't a
  tilt-safety quantity and was causing false trips).
- **SITL PID gains scaled down (16× total, in two 4× steps)** to stop on-arm
  saturation in the simulated airframe.
- **`hal_crc_compute` matches the STM32 hardware CRC32** the GCS expects, so
  SITL frames validate against the same checksum as real hardware.

---

## Build / Toolchain

- Own the **NavHAL Kconfig in the superproject** while keeping the `vaios`
  submodule pristine; set `$srctree` so NavHAL's post-M7 modular Kconfig
  resolves; bumped the `vaios` submodule (NavHAL `main@217eca6` + perf module).
- Dropped an orphan top-level `extern/NavHAL` submodule entry and stopped
  leaking the superproject `include/` into the `vaios` subtree.

---

## Earlier foundations (2025-09 → 2026-03)

Condensed; see the per-feature deep-dive changelogs where noted.

- **Attitude estimation:** IMU integration, attitude calculation, and a **Mahony
  AHRS filter on quaternions** with centralised configuration and a gyro-
  sensitivity fix — full writeup in
  [implement-mahony-filter-and-quaternions.md](implement-mahony-filter-and-quaternions.md).
- **RC + telemetry:** iBus RC parsing (UART1 DMA circular buffer) and telemetry
  broadcasting, including the critical UART1 BRR/baudrate fix — full writeup in
  [implement-rc-telemetry-and-gcs-ui.md](../../../navigator/docs/journal/changelog/implement-rc-telemetry-and-gcs-ui.md).
- **Comm core:** packet types, serialization infrastructure, and a dedicated
  physical heartbeat module.
- **Math backend:** introduced custom `m_`-prefixed math routines and removed
  the ARM CMSIS-DSP backend, standardising on `math.h` to simplify the build.
