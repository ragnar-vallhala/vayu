# Hardware test suites — HIL comms + on-hardware bench

Two new test categories that, unlike everything in `vayu.sh test` today, need a
real flight controller (see [build.md §test](../../build.md) for the existing
host/SITL suites). Both are **opt-in and auto-skip** when no FC is present, so CI
and laptop runs stay green.

| Suite | `vayu.sh` | What it exercises | Needs |
| --- | --- | --- | --- |
| **HIL comms** | `test hil`  | NavLink comms against the **real (production) firmware** over UDP/UART — link bring-up, telemetry rates, command/ACK, TIME_SYNC gate, file transfer, robustness | An FC running normal firmware + a link (ESP UDP bridge or USB-serial) |
| **On-hardware bench** | `test onhw` | Drivers, RTOS/OS, sensors, and physical I/O, run **on the MCU** by a dedicated **test firmware** that reports results over NavLink | An FC flashed with the *test* firmware image |

`test hw` = run both. Both fold into `test all` and report into the same
`Files | Asserts | Pass | Fail | Skip` table; when no FC is detected the planned
checks land in the **Skip** column and the run still passes.

---

## Decisions (locked)

1. **HIL transport: UDP first, UART second.** Abstract a transport seam; ship the
   ESP8266/WiFi UDP bridge path first (it's what the existing `tools/telemetry/*`
   already use), then add direct USB-serial→USART6.
2. **On-hardware = separate test firmware, not instrumented production firmware.**
   The bench tests and their NavLink messages are compiled **only** into a
   dedicated `hwtest` image. Production firmware never references them, so they add
   **zero flash/RAM bloat** to the real FC. (The F401 is RAM-starved — see
   `memory/f401-sram-heap-overflow.md` — so this separation is also what makes the
   bench scaffolding fit.)
3. **New NavLink test messages live in a separate dialect** in the reserved
   vendor/test msgid range (`0x800000–0xFFFFFF`, per the v2 spec), generated into
   the `hwtest` firmware + host runner only — never into the production codec.
4. **Fold into `test all` with auto-skip.** One command runs everything available;
   missing hardware is a clean skip, not a failure.

---

## Background (verified in-repo)

### NavLink transport reality
- Wire protocol: NavLink v2 (`navlink/dialect.json`, `navlink/generate.py`;
  spec `navlink/docs/reference/navlink-v2-spec.md`). Frame =
  `sync(0x56) ver(0x02) len flags seq sysid compid msgid(u24) payload crc16`.
- **Real HW transports:**
  - **UART** — FC `USART6` @ 460800 8N1 = telemetry/commands; `USART2` = RC in
    (`firmware/src/main.c`, `firmware/src/comm/channel.c`). See
    `memory/uart-port-roles.md`.
  - **UDP** — ESP8266 `tools/arduino/telemetry_bridge/` relays whole frames
    over WiFi UDP `:14555`; host broadcasts `GCS-HELLO`, bridge unicasts back.
    Frames are coalesced per datagram (MTU-capped).
- **Command gate:** the FC silently rejects commands until a **TIME_SYNC**
  handshake completes (spec §10.5; `firmware/src/comm/navlink_router.c`). Every
  command test must TIME_SYNC first — this is the same gate documented in
  `memory/harness-timesync-gate.md`.
- Ready-made references: `tools/telemetry/udp_telem_sniff.py`,
  `tools/telemetry/fs_xfer_udp_test.py` (discovery + TIME_SYNC + xfer over UDP),
  Python codec `navlink/generated/python/`, frame parser `navlink/sim/frame.py`,
  SDK decode `navigator/headless-sdk/vayu_headless/transport/navlink.py`.

### On-target self-test template
- `EKF_SELFTEST` is the gold-standard pattern: a CMake option
  (`firmware/CMakeLists.txt`) gates `firmware/src/est/ekf_selftest.c`, which runs
  **before the scheduler starts** (`firmware/src/main.c`) and reports each check
  via `vayu_log()` → NavLink `Statustext`. Identical source builds on host + target.
- OS/HAL surface: vaios kernel (`extern/vaios/include/{task,ipc,memory,perf}.h`)
  + NavHAL drivers (I2C/UART/GPIO/PWM/DMA/CRC/DWT, `firmware/navhal.config`).
  Perf counters expose stack/heap watermarks, sched switches, ISR latency.
- Sensors: BMX160 IMU (`WHO_AM_I` 0x00→0xD8) + BME280 baro (0xD0→0x60) share
  I2C1 on a single-owner DMA loop (`memory/i2c-bus-sharing.md`). Still-detector
  thresholds already defined in `firmware/src/sensor/bmx160.c`.
- Flash/report: `tools/scripts/flash.sh` (objcopy → `st-flash --connect-under-reset`).
- **Safety:** motors only emit PWM in ARMED/IN_AIR; the bench firmware stays in
  STANDBY and never arms. PWM/ESC checks are duty/GPIO-level with **props-off**
  operator confirmation.

---

## Architecture

```
                       ┌────────────────────────── host ──────────────────────────┐
   real FC  ◀── UDP/UART ──▶  live link transport  ──▶  HIL comms pytest  (Suite A)
 (prod fw)                    (navlink py codec)        tests prod-fw comms

  test fw   ◀── UDP/UART ──▶  hwtest host runner   ──▶  on-hw bench report (Suite B)
 (hwtest)        NavLink test msgs (0x800000+)         flashes image, tallies results
```

### Shared host foundation (Phase 0)
- **`vayu_headless.transport.live`** — a transport seam with two backends:
  - `UdpBridge` (broadcast `GCS-HELLO`, learn peer, de-coalesce datagrams) — first.
  - `SerialLink` (`pyserial` to `/dev/tty*` @ 460800) — second.
  Both yield the same decoded-frame stream the SDK already consumes.
- **`LiveSession`** — mirrors the `SitlSession` surface (`.telem`,
  `.telem_counts`, `send()`, `time_sync()`) but talks to a real FC. Lets HIL tests
  reuse existing decode/Pilot helpers.
- **Detection / skip:** `fc_available()` → UDP: a heartbeat within N s of
  `GCS-HELLO`; serial: port exists and heartbeats decode. Drives pytest skips and
  the table's Skip column.
- **Config (env):** `VAYU_FC_TRANSPORT=udp|serial`, `VAYU_FC_UDP_PORT=14555`,
  `VAYU_FC_SERIAL=/dev/ttyUSB0`, `VAYU_FC_BAUD=460800`, `VAYU_FC_HWTEST_BIN=…`.

### Suite A — HIL comms (`test hil`)
Pytest under `navigator/headless-sdk/tests/hil/`, marked `@pytest.mark.hil`,
skipped unless `fc_available()`. Drives the **production** firmware. Candidate
scenarios (all disarmed-safe):

- **Link** — UDP discovery / serial open; heartbeat ≥1 Hz; reconnect after drop.
- **Telemetry** — per-message rates vs nominal (ATTITUDE_EULER ~50 Hz, IMU_RAW
  ~1.7 Hz, IMU_COMPRESSED ~50 Hz, RC ~11 Hz, BARO 5 Hz); decode integrity; seq
  continuity (loss%); CRC resync after injected junk.
- **TIME_SYNC** — handshake completes; pre-sync command is rejected, post-sync
  accepted (proves the §10.5 gate).
- **Commands + ACK** — `CMD_SET_PID`, `CMD_SET_GYRO_LPF`, `CMD_ARM` (expect
  TEMPORARILY_REJECTED on the bench), deferred-ACK timing; `PING` RTT histogram.
- **Params** — `PARAM_REQUEST_READ` → `PARAM_VALUE`; `PARAM_SET` round-trip.
- **File transfer** — download + upload a small blob over the xfer substrate;
  SHA-256 round-trip; lossy-resume (drop-nth) correctness; throughput vs the
  ~44 KB/s 460800-baud ceiling.

### Suite B — on-hardware bench (`test onhw`)
A dedicated **test firmware** + a host runner.

- **Test firmware** — `firmware/tests/onboard/` with its own entry/runner, built
  via `cmake -S firmware -B build_hwtest -DVAYU_HW_TEST=ON` → `build_hwtest/hwtest`
  (+`.bin`). Reuses HAL/RTOS/drivers; runs a check list and streams results over
  the test dialect. Production build (`-DVAYU_HW_TEST` off) compiles none of it.
- **Test NavLink dialect** — `navlink/test_dialect.json` (msgids `0x800000+`):
  - `HW_TEST_BEGIN { total }`
  - `HW_TEST_RESULT { id, name[24], pass(u8), value(f32), units[8] }`
  - `HW_TEST_DONE  { passed, failed, skipped }`
  Generated C → `hwtest` only; Python → host runner only.
- **Host runner** — `tools/hwtest/run.py`: flash `hwtest.bin` (reuse flash flow),
  open the live link, collect `HW_TEST_*`, tally, print, feed the table.
- **Check categories** (from the on-hw surface map):
  - *Boot/OS* — clock freq; STANDBY reached; heap/stack watermarks; sched switch
    + SysTick jitter; IPC (sema/mutex give-from-ISR).
  - *Drivers/bus* — I2C blocking + DMA-async read; bus unstick/recovery;
    register write/readback; CRC peripheral; DWT cycle counter.
  - *Sensors* — BMX160/BME280/BMM150 `WHO_AM_I`; I2C scan; gyro/accel stillness
    (< thresholds); baro pressure/temp plausibility & drift.
  - *I/O* — USART6 loopback; RC iBus parse (injected frame); **PWM duty/GPIO**
    check (DISARMED, props-off confirmation) + disarm→0 safety.
  - *Estimation* — reuse `ekf_selftest_run()` (8 branch checks) as one bench item.
  - *Persistence* — SD/FatFS write→readback CRC; calib save/restore.

### `vayu.sh` + table integration (Phase 0)
- New `ensure_hil_build` (pip + live transport deps), `ensure_onhw_build`
  (`build_hwtest`), `run_hil_tests`, `run_onhw_tests`; reuse the tee+parse pattern.
- Dispatch: `test hil`, `test onhw`, `test hw`; `test_all` calls them after the
  host suites.
- Auto-skip: if `fc_available()` is false, emit a clear note and record the planned
  count as **Skip** (no Fail). New table rows: `hil (live FC)`, `onhw (bench)`.
- Parsing: HIL via `parse_pytest`; onhw runner prints a pytest-style summary line
  (`N passed, M failed, K skipped`) so the same parser applies.

---

## Phasing

- **P0 — plumbing.** Transport seam (UDP), `fc_available()`, `vayu.sh`
  hil/onhw/hw subcommands, table rows + auto-skip. Empty suites that skip cleanly.
- **P1 — HIL core (UDP).** Discovery, heartbeat, telemetry rates, TIME_SYNC gate,
  PING, command/ACK. The first real green/skip on hardware.
- **P2 — HIL full.** xfer round-trip + lossy-resume, params, robustness/resync,
  throughput; add the **serial** backend.
- **P3 — bench skeleton.** Test dialect + codegen wiring; `hwtest` firmware with
  boot/OS + sensor `WHO_AM_I` + EKF self-test; host runner flashes & tallies.
- **P4 — bench full matrix.** I2C/DMA, IMU noise/bias, baro, UART loopback, RC,
  PWM-safe, scheduler/heap/stack, flash/SD, calibration.
- **P5 — docs + CI.** `build.md` updates, a hardware-setup guide, and notes for an
  optional self-hosted HW runner (default CI skips).

---

## Risks / open items
- **Flakiness:** WiFi/UDP loss & jitter make tight rate/latency asserts flaky —
  use tolerance bands (cf. the SITL "known-wobbly" bands) and loss-% thresholds,
  not exact counts. Prefer serial for deterministic timing-sensitive checks.
- **Flashing in CI:** `test onhw` reflashes the FC; gate behind an explicit
  env/flag so `test all` on a dev box with an FC attached doesn't reflash
  unexpectedly (auto-skip should default to *not* flashing unless `test onhw`/`hw`
  is requested or `VAYU_HW_ALLOW_FLASH=1`).
- **Test-dialect drift:** keep `test_dialect.json` regen in the codegen gate so C
  and Python test messages can't diverge.
- **Safety interlocks:** bench PWM tests require props off; require an explicit
  `VAYU_HW_PROPS_OFF=1` (or interactive confirm) before any PWM/ESC item runs.
- **F401 RAM:** even the `hwtest` image must respect the heap/BSS ceiling
  (`memory/f401-sram-heap-overflow.md`); drop unneeded production tasks to fit.
