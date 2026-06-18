# Log archive — 2026-06-17 12:42:10

Self-contained analysis archive for one real-hardware telemetry capture. The
raw `.bin` is kept here so every number can be re-derived later.

## Contents

| file | what |
|---|---|
| [`export-20260617-124210.bin`](export-20260617-124210.bin) | raw capture (source of truth) |
| [`session-analysis.md`](session-analysis.md) | full session analysis (link, flight timeline, controller, health) |
| [`sensor-analysis.md`](sensor-analysis.md) | sensor calibration & reporting deep-dive |
| `csv/` | per-message-type CSVs for plotting (git-ignored, regenerable) |

Decoder lives one level up: [`../parse_log.py`](../parse_log.py). Reproduce
everything from repo root:

```sh
python3 docs/log-analysis/parse_log.py docs/log-analysis/20260617-124210/export-20260617-124210.bin
python3 docs/log-analysis/parse_log.py docs/log-analysis/20260617-124210/export-20260617-124210.bin --csv docs/log-analysis/20260617-124210/csv
```

## Provenance & integrity

| | |
|---|---|
| Original path | `/home/ragnar/vayu-logs/export-20260617-124210.bin` |
| Archived | 2026-06-18 |
| Size | 2,032,071 bytes |
| SHA-256 | `8b4f82ccb50a7166c23fd9149f51069063fd522de7b40df86898ad5aed039e0c` |

## Container header (`VREC`, `software/src/replay/RecordFormat.h`)

| field | value |
|---|---|
| magic | `0x56524543` ("VREC") |
| format version | 1 |
| protocol version | 1 |
| start wall clock | 1781680333706 ms → **2026-06-17 07:12:13 UTC** |
| records (byte chunks) | 38,292 |
| decoded frames | 38,292 |
| **CRC errors** | **0** |
| unknown msgids | none |
| time span | 375.2 s (6.25 min), t_us 31810007043 … 32185159631 |
| trailing bytes | 0 |

## Message inventory

| msgid | message | count | rate (Hz) |
|---:|---|---:|---:|
| 1025 | ImuCompressed | 8,764 | 23.4 |
| 1030 | ControlTrace | 7,450 | 19.9 |
| 1029 | MotorTelemetry | 6,562 | 17.5 |
| 1026 | AttitudeEuler | 3,520 | 9.4 |
| 1028 | RcChannels | 3,458 | 9.2 |
| 1035 | PerfTask | 3,173 | 8.5 |
| 0 | Heartbeat | 1,078 | 2.9 |
| 1036 | PerfFifo | 1,062 | 2.8 |
| 3 | FlightMode | 1,015 | 2.7 |
| 2 | SystemHealth | 969 | 2.6 |
| 1024 | ImuRaw | 508 | 1.4 |
| 1034 | PerfGlobal | 372 | 1.0 |
| 1033 | EstPerf | 284 | 0.8 |
| 10 | TimeSync | 77 | 0.2 |

Absent: `Statustext`, `CommandAck`, `Ping`, `Capabilities`, `Param*`,
`CalibrationStatus`, all `Cmd*`.

## Raw findings log (everything we dug out)

Quick reference; see the two analysis docs for context and method.

**Decode/transport**
- 0 CRC errors / 0 unknown msgids over 38,292 frames — wire stack is clean on HW.
- Avg downlink ~4.2 kB/s (33.5 kbps). 9 record gaps > 200 ms (max 312 ms).
- `SystemHealth.tx_overflow` 612 → 2680 (~2,068 telemetry TX-buffer drops): link saturated.

**Flight state**
- Armed 4× (19.6 s, 98.7 s, 44.0 s, 50.7 s) — **never `IN_AIR`**. Bench/handheld.
- Mode ANGLE throughout except ACRO 185–219 s. All mode changes `source=RC`.
- Every RC dropout (chan 0/1 → 62954 = −2582 as int16) → `FAILSAFE` (615/620 spike frames). Safety path correct.

**Controller (ARMED windows, ° / °·s⁻¹)**
- roll angle err rms 23.3°, pitch err rms 33.1° — dominated by hand motion, not tuning.
- Throttle ≤ 0.3, motors ≤ 0.69, never saturated. Loop timing 250 Hz outer / 1 kHz inner, zero jitter.

**Sensors**
- Gyro: bias < 0.1 °/s, noise σ 0.3–0.5 °/s — ✅ good.
- Accel: +7.8 % scale error at rest (motors off); not vibration — ⚠ needs cal.
- Mag: `|mag|` 22–92 µT (140 % of mean), hard-iron offsets to 25 µT — ❌ uncalibrated.
- Temp: 35.6–37.1 °C, smooth — ✅ good.
- Units: gyro °/s, accel m/s², mag µT, temp °C.

**Telemetry data-quality bugs (firmware-side)**
1. `AttitudeEuler.{rollspeed,pitchspeed,yawspeed}` always 0 (body rates not populated).
2. `SystemHealth.cpu_load` always 0.
3. `ImuRaw.sample_time_us` always 0.
4. `ImuCompressed.ref_seq` always 0 — broken keyframe linkage; compressed stream not reconstructable.
5. Unit mismatch: `ControlTrace` angles in degrees vs `AttitudeEuler` in radians (verified 1.00 ratio).

**Calibration evidence**
- 0 `CalibrationStatus` messages; `nav_state` never `CALIBRATING`. No calibration captured this session.
