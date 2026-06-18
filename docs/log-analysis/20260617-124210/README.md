# Log archive — 2026-06-17 12:42:10

Self-contained analysis archive for one real-hardware telemetry capture. The
raw `.bin` is kept here so every number can be re-derived later.

## Contents

| file | what |
|---|---|
| [`20260617-124210-analysis.pdf`](20260617-124210-analysis.pdf) | **all docs below combined into one printable PDF** (with plots & diagrams) |
| [`export-20260617-124210.bin`](export-20260617-124210.bin) | raw capture (source of truth) |
| [`session-analysis.md`](session-analysis.md) | full session overview (link, flight timeline, controller, health) |
| [`control-loop-analysis.md`](control-loop-analysis.md) | cascade controller: the throttle-only tilt, yaw spins, gains/authority |
| [`motor-analysis.md`](motor-analysis.md) | quad-X mixer, motor balance, saturation |
| [`kernel-analysis.md`](kernel-analysis.md) | vaios RTOS: CPU, stacks, heap, IPC, FIFO drops |
| [`sensor-analysis.md`](sensor-analysis.md) | sensor calibration + **fusion** (EKF) accuracy & reporting |
| `plots/` | figures embedded in the docs above (committed; regenerate with `make_plots.py`) |
| `csv/` | per-message-type CSVs for plotting (git-ignored, regenerable) |

Decoder lives one level up: [`../parse_log.py`](../parse_log.py). Reproduce
everything from repo root:

```sh
python3 docs/log-analysis/parse_log.py docs/log-analysis/20260617-124210/export-20260617-124210.bin
python3 docs/log-analysis/parse_log.py docs/log-analysis/20260617-124210/export-20260617-124210.bin --csv docs/log-analysis/20260617-124210/csv
python3 docs/log-analysis/make_plots.py docs/log-analysis/20260617-124210   # regenerate plots/
python3 docs/log-analysis/build_pdf.py docs/log-analysis/20260617-124210    # rebuild the combined PDF
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

Quick reference; see the per-domain analysis docs for context and method.
**Rig context:** captured on a bench rig — never `IN_AIR`; the frame was
hand-spun in yaw and rested tilted. Interpret all attitude/controller numbers as
bench, not flight.

**Decode/transport**
- 0 CRC errors / 0 unknown msgids over 38,292 frames — wire stack is clean on HW.
- Avg downlink ~4.2 kB/s (33.5 kbps). 9 record gaps > 200 ms (max 312 ms).
- `SystemHealth.tx_overflow` 612 → 2680 (~2,068 telemetry TX-buffer drops): link saturated.

**Flight state**
- Armed 4× (19.6 s, 98.7 s, 44.0 s, 50.7 s) — **never `IN_AIR`**. Bench rig.
- Mode ANGLE throughout except ACRO 185–219 s. All mode changes `source=RC`.
- Every RC dropout (chan 0/1 → 62954 = −2582 as int16) → `FAILSAFE` (615/620 spike frames). Safety path correct.

**Control loop** ([detail](control-loop-analysis.md))
- Level command but frame held **pitch +25°, roll −10°** — tilt is **real** (fusion matches gravity), not a sensor lie.
- Controller corrects in the right direction but with almost no authority: rate Kp roll/pitch = 0.0005 (Kd 0, integral-dominated) vs **yaw Kp 0.018 (36×)**. Empirical slopes confirm (0.00033 / 0.0117).
- Throttle averaged 0.25, **100 % below the 0.30 PID-full-authority point** → roll/pitch further attenuated all session.
- Yaw: `yaw_rate_sp`=0 always (no yaw commanded); frame disturbed ±80 °/s; `yaw_out` saturates 0.5 % of frames; alternating ±65–80 °/s spins at 343–351 s = hand-spin/clamp (possible yaw limit-cycle).
- Loop timing 250 Hz outer / 1 kHz inner, **zero jitter**.

**Motors** ([detail](motor-analysis.md))
- Quad-X mixer signs verified vs firmware. M1 FR/CW, M2 RR/CCW, M3 RL/CW, M4 FL/CCW; `cmd[]` 0..1.
- Peak 0.69, **never saturated** (0 frames > 0.95). Throttle-only imbalance (M1 −19 %, M2 +14 %) is the controller's corrections mixed in, not a raw actuator fault — can't assess motor health from a rig.

**Kernel / vaios** ([detail](kernel-analysis.md))
- CPU **~63 %** (steady, via cycle deltas), estimator EKF 250 Hz using **22 %** of its 4 ms budget.
- Heap 0 OOM (37 % peak), IPC 0 timeouts, stacks in margin (worst task_id 4 = 74 %).
- **FIFO drops are telemetry-only**: control FIFOs 0 drops; telemetry FIFOs pinned 100 %, drop counter saturated at u16 65535. Control path never starved.

**Sensors & fusion** ([detail](sensor-analysis.md))
- Gyro: bias < 0.1 °/s, noise σ 0.3–0.5 °/s — OK good.
- Accel: +7.8 % scale error at rest (motors off); not vibration — WARN needs cal.
- Mag: `|mag|` 22–92 µT (140 % of mean), hard-iron offsets to 25 µT — FAIL uncalibrated.
- Temp: 35.6–37.1 °C, smooth — OK good.
- **Fusion (EKF):** roll/pitch accurate (fused tilt 33.7° vs accel 37.5°, Δ −3.8°) — tilt is real OK. **Yaw untrustworthy** (pinned by uncalibrated mag) FAIL.
- Units: gyro °/s, accel m/s², mag µT, temp °C.

**Telemetry data-quality bugs (firmware-side)**
1. `AttitudeEuler.{rollspeed,pitchspeed,yawspeed}` always 0 (body rates not populated).
2. `SystemHealth.cpu_load` always 0.
3. `ImuRaw.sample_time_us` always 0.
4. `ImuCompressed.ref_seq` always 0 — broken keyframe linkage; compressed stream not reconstructable.
5. Unit mismatch: `ControlTrace` angles in degrees vs `AttitudeEuler` in radians (verified 1.00 ratio).

**Calibration evidence**
- 0 `CalibrationStatus` messages; `nav_state` never `CALIBRATING`. No calibration captured this session.
