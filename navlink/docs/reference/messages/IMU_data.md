# IMU Data

> Updated for NavLink v2. Authoritative wire spec: navlink/dialect.json + docs/analysis/navlink-v2-spec.md.

IMU data is sent FC → GCS at ~100 Hz. There are two messages:

1. **`IMU_RAW` (msgid 1024)** — full keyframe.
2. **`IMU_COMPRESSED` (msgid 1025)** — delta from the last keyframe.

## Keyframe / delta model

The full keyframe carries **10 IEEE-754 32-bit floats**: accelerometer X/Y/Z, gyroscope X/Y/Z, magnetometer X/Y/Z, and temperature (°C), with a timestamp.

`IMU_COMPRESSED` is sent for intermediate updates and carries the **10 values as f16 (half-precision) deltas** from the last `IMU_RAW` keyframe. A keyframe is emitted every N samples or when the delta exceeds threshold.

For the exact field order, packing, and CRC, see `../../navlink/dialect.json` and `../analysis/navlink-v2-spec.md`.

## Changelog

| Date       | Author               | Description                                  |
| ---------- | -------------------- | -------------------------------------------- |
| 06/03/2026 | Ashutosh Vishwakarma | Initial version                              |
| 06/2026    | —                    | NavLink v2: IMU_RAW 1024 / IMU_COMPRESSED 1025 |
