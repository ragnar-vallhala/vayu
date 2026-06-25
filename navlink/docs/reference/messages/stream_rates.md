# Stream Rates (0x2007 / `CMD_SET_STREAM_RATE`) — design & implementation plan

Status: **SUPERSEDED** (never implemented). Per-stream rate control is now folded
into the generic bulk-transfer/streaming substrate: a live stream is opened with
`XFER_OPEN{mode=stream, service_id=stream, arg=<source>, rate_hz=<rate>}` (see
[`xfer.md`](xfer.md)), which reuses one mechanism instead of a bespoke command. The
msgid **8201** once sketched here for `CMD_SET_STREAM_RATE` is now `XFER_OPEN`. If a
GCS-controlled enable/throttle of the *always-on periodic* telemetry (IMU/MOTOR/
PERF/…) is still wanted, design it as a thin command atop the substrate. The
original design is retained below for reference.

---

This document specifies a GCS-controlled, per-stream **enable + request-rate**
mechanism for FC→GCS telemetry, so unused streams can be disabled or throttled to
fit the link.

## Motivation

The FC↔GCS link is an ESP8266 WiFi/UART bridge with limited downlink (~115 KB/s,
and lossy in practice). Today every telemetry stream is hardcoded **on** at a
fixed cadence — the gates are `packet_counter % N == 0` literals in
`src/comm/telemetry_task.c` (plus a fixed 1 Hz loop in
`src/comm/perf_telemetry.c`). There is no way to:

- disable a stream you aren't watching (e.g. PERF, CONTROL_TRACE, MOTOR), or
- lower a stream's rate to free headroom (e.g. drop IMU_COMPRESSED 27 → 5 Hz).

The GCS Settings page already ships a **disabled placeholder** for exactly this
(`software/src/ui/main/SettingsWidget.cpp`, "Telemetry Streams … needs
SET_STREAM_RATE", columns *Stream / Type / Rate (Hz) / On*). This feature wires it
up and persists the choice **on the FC's SD card**, so bandwidth is controlled
from power-on, before any GCS connects.

## Stream model

Each stream has a stable id (reusing the placeholder's "Type" codes) and a
request rate in Hz; **rate 0 = disabled**. Defaults equal the *current* effective
cadence, so behaviour is unchanged until the user touches it.

| id | stream | tx fn / source | default | notes |
|----|--------|----------------|---------|-------|
| 0  | HEARTBEAT       | `navlink_tx_heartbeat`            | 1 Hz  | **clamped ≥1 Hz** (COMM-TEL-002 liveness); can't be disabled |
| 1  | IMU_FULL        | `navlink_tx_imu_full`            | ~2 Hz | keyframe for the compressed delta stream |
| 2  | IMU_COMPRESSED  | `navlink_tx_imu_compressed`      | ~27 Hz| f16 delta from last IMU_FULL |
| 3  | CONTROL_TRACE   | `navlink_tx_pid_error`           | ~21 Hz| PID loop trace (tuning only) |
| 4  | ATTITUDE        | `navlink_tx_attitude`            | ~11 Hz| |
| 5  | RC              | `navlink_tx_rc_channels`         | ~11 Hz| |
| 6  | STATUS          | `navlink_tx_flight_mode` + `navlink_tx_health` | 2 Hz | vehicle mode + health counters |
| 7  | LOG             | `navlink_tx_log`                 | ~15 Hz| event-driven drain; "rate" = drain check cadence |
| 8  | MOTOR           | `navlink_tx_motor`               | ~21 Hz| |
| 9  | PERF            | `perf_telemetry.c` global/task/fifo | 1 Hz | kernel/task/FIFO snapshot |
| 10 | EST_PERF        | `navlink_tx_est_perf`            | 1 Hz  | estimator cost probe |

Not configurable (procedural / handshake, always on):
`CALIBRATION_STATUS` (0x3020) and `TIME_SYNC` (0xA).

The telemetry loop runs a ~500 Hz base tick (`TELEM_BASE_MS = 2`). Each stream is
gated by a **millisecond period** via the `TELEM_GATE` macro, so its effective
rate is independent of the base-loop frequency; the loop emits a stream when its
gate elapses (gate floor = 1 tick) and skips it entirely when its period is 0.

## Wire format

New command, NavLink v2 msgid **0x2007** (`CMD_SET_STREAM_RATE`), in the
command range `0x2000–0x2FFF` — so it is **§10.5-gated** (rejected until the FC is
time-synced) and auto-acknowledged with `COMMAND_ACK`. One command sets one
stream; the GCS sends one per changed row.

`navlink/dialect.json`:

```json
{ "msgid": 8199, "name": "CMD_SET_STREAM_RATE",
  "doc": "Enable/throttle one FC->GCS telemetry stream. rate_hz=0 disables; HEARTBEAT clamped >=1.",
  "fields": [
    { "index": 0, "name": "target_sys",  "type": "u8" },
    { "index": 1, "name": "target_comp", "type": "u8" },
    { "index": 2, "name": "req_seq",     "type": "u8" },
    { "index": 3, "name": "stream_id",   "type": "u8",  "enum": "telemetry_stream" },
    { "index": 4, "name": "rate_hz",     "type": "u16", "unit": "Hz", "doc": "0 = off" } ] }
```

plus an enum `telemetry_stream` with the ids above. Legacy id:
`CMD_SET_STREAM_RATE = 0x000E` in `include/comm/comm_types.h`.

## FC behaviour

New module `src/comm/telemetry_config.{c,h}`, mirroring `src/control/pid_config.c`:

- `uint16_t s_rate_hz[N_STREAMS]` held in RAM, persisted to `0:telem.bin` with a
  versioned `{magic, version, size}` header (same scheme as the calibration file).
- `telemetry_config_init()` — load from SD at boot; on missing/invalid file use
  the compiled-in defaults. Called from `src/main.c` next to `pid_config_init()`.
- `telemetry_config_apply_command(payload, len)` — COMM-CMD-002 validation
  (`argc`/length, finite, range), clamp HEARTBEAT ≥1 Hz, store, save to SD, log.
  Returns `VAYU_OK` / `VAYU_ERR_INVALID` (drives the `COMMAND_ACK`).
- `uint16_t telemetry_config_period_ticks(uint8_t stream_id)` — getter the loop
  uses (0 = disabled); a `generation` counter lets the loop cache periods.

`src/comm/telemetry_task.c` replaces the hardcoded `send_x = (packet_counter % N
== 0)` lines with `send_x = stream_due(STREAM_X, packet_counter)`, and **splits
HEARTBEAT out of the STATUS bundle** so it has its own rate (flight_mode + health
remain under STATUS). `src/comm/perf_telemetry.c` gates its loop on the PERF
stream (skip when rate 0, else derive its report period from the rate).

Routing: `src/comm/navlink_router.c` adds `on_cmd_set_stream_rate`
(`build_cmd` → `telemetry_config_apply_command`) and registers it;
`src/comm/comm_processor.c` adds the dispatch branch (mirroring `CMD_SET_GYRO_LPF`).
SITL: add `telemetry_config.c` to `tools/sim_host/CMakeLists.txt`.

## GCS behaviour

- `software/src/protocol/CommandCodec.{h,cpp}`: `encodeSetStreamRate(streamId,
  rateHz, devId=42)` (follows `encodeCalibrate`).
- `software/src/core/SettingsManager.h`: add `quint16 streamRateHz[11]` to
  `GcsSettings` (defaults = current rates), appended to the versioned
  `operator<</>>` with an `atEnd()` sentinel (back-compat).
- `software/src/ui/main/SettingsWidget.{h,cpp}`: make the existing "Telemetry
  Streams" table live — "On" → `QCheckBox`, "Rate (Hz)" → `QSpinBox` (disabled
  when off; unchecking sets rate 0), add CONTROL_TRACE + EST_PERF rows, drop the
  `setEnabled(false)`/"coming soon". The bandwidth-estimate label becomes live
  (Σ rate × frame-size).
- `software/src/ui/main/MainWindow.cpp`: in `applyAllSettings`, send one
  `CMD_SET_STREAM_RATE` per changed row via `sendToFc`; also push the saved config
  once **after time-sync locks** on connect (reuse the `m_tsEst.synced()` gate) so
  a fresh GCS imposes its view. FC SD-persistence covers the pre-connect window.

No new telemetry RX handling is needed — it's a command, so the existing
`COMMAND_ACK` path applies. Disabling a stream simply makes its GCS panel go
stale/blank via the existing freshness ("NA") handling.

## Verification

1. **Codec** — `python3 navlink/tests/run_tests.py` (parity/frame/sim) after the
   dialect change; `tst_command_codec` in the GCS.
2. **Builds** — firmware (`arm-none-eabi-gcc -Werror`) and `Navigator`.
3. **Wire** — with the FC synced, send `CMD_SET_STREAM_RATE` to disable IMU (ids
   1+2 → 0) and lower MOTOR; the msgid sniffer confirms those streams stop/slow
   while HEARTBEAT stays ≥1 Hz and others are unaffected.
4. **SD persistence** — set a config, power-cycle the FC, sniff *before* any GCS
   connects → the saved rates are in effect from boot.
5. **GCS end-to-end** — Settings → uncheck IMU / set Motor to 5 Hz → Apply → the
   IMU panel goes stale and the motor graph slows; reconnect → re-applied after
   sync-lock.
6. **Clamp** — set HEARTBEAT to 0 → FC clamps to 1 Hz (ACK_OK, heartbeat keeps
   flowing).

## Status / follow-ups

- Flip `Status` to **implemented** once the code lands (as `time_sync.md` did).
- Possible future work: a one-shot "request all current rates" query so the GCS
  table reflects the FC's persisted config on connect instead of imposing its own.
