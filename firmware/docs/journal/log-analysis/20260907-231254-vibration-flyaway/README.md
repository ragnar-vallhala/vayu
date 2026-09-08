# Prop-vibration collapse and airmode flyaway — real drone (2026-09-07 23:12)

Two bench/lift captures taken the evening after the first successful hover. The
session was meant to be a routine repeat; instead it recorded the first
**uncommanded climb** and, behind it, an accelerometer that stops measuring
gravity the moment the props spin.

**Headline: with the props turning, the accelerometer reads 2.5 m/s² where 9.81
is the answer — it loses three quarters of gravity.** That one mechanical fault
produces every critical finding here. Through the accel path it destroys the
vertical estimate (`climb_rate` −9.7 m/s on a stationary craft); through the gyro
path it drives the rate loop into saturation, where `MIXER_AIRMODE_RP` converts
saturation into collective and the craft climbs while the stick is coming down.

The `accel_unhealthy` veto added three days earlier caught the half it could see
and kept the height mode from ever engaging. Nothing guards the rate-loop half.

## Provenance

| | |
|---|---|
| Airframe | 5" racer, 650 g, 220 mm, 2400 KV on 3S ([`racer5`](../../../plans/altitude-hold-and-in-air-plan.md)) |
| Firmware | `0ae8b38` "perf(telem): spend the link budget on the vertical estimator" |
| Build config | `Release`, **`VAYU_FFT_NOTCH=OFF`** — see Finding 10 |
| Capture | Navigator app → VREC (`navigator/src/replay/RecordFormat.h`) |
| Link | ESP8266 UDP bridge, :14555 |
| Mode | stabilise throughout (`FLIGHT_MODE` = `(0,0)` in every sample); height mode requested `HOLD` and never engaged |

## Inventory

| file | bytes | records | frames | span | frame loss | VERTICAL_STATE |
|---|---|---|---|---|---|---|
| `export-20260907-231254.bin` | 331,098 | 8,421 | 8,312 | 76.6 s | **26.9%** | 10.5 Hz rx (20.7 Hz sent) |
| `export-20260907-231430.bin` | 0 | 0 | — | — | — | — |
| `export-20260907-231706.bin` | 383,743 | 9,801 | 9,767 | 47.3 s | **0.7%** | 20.4 Hz |

Both surviving files are valid `VREC` v1/proto-1 with no truncated trailing
record. The middle file is **0 bytes — not even the 20-byte header**, so
`RecordSink::open()` produced a file and no header write landed; it is kept as
evidence of that failure mode, which is indistinguishable from "no data arrived"
unless you check the length.

The loss difference between the two is the flaky ESP VCC jumper, found and fixed
between them — 26.9% → 0.7% with no firmware change. This retires the earlier
"it's the RF link" conclusion (see [`recommendations.md`](recommendations.md)).

**Neither capture ever reached `IN_AIR`**, including the one where the craft
visibly left the ground and climbed to 1.6 m. The takeoff detector keys off
`climb_rate`, which was reading −9.7 m/s, so the FC believed it was falling
throughout.

## Contents

- [`analysis.md`](analysis.md) — the ten findings, by severity, with the numbers
- [`recommendations.md`](recommendations.md) — what to fix, in order, and what NOT to try
- `export-20260907-231254.bin`, `export-20260907-231706.bin` — the raw captures

## Reproduce

```sh
python3 tools/telemetry/analyze_log.py --vertical <archive>/export-20260907-231706.bin
```
