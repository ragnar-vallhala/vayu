# System Status [FC → GCS]

> Updated for NavLink v2. Authoritative wire spec: navlink/dialect.json + docs/analysis/navlink-v2-spec.md.

The FC reports structured events, sensor health, and operator instructions to the GCS. The v1 single `0x6` container (an `origin` byte + a padded `values[n]` float array) has been retired. In NavLink v2 each former "origin" is its **own message**:

| Former v1 origin              | v2 message (msgid)            |
| ----------------------------- | ----------------------------- |
| `SYSTEM_ORIGIN_HEALTH`        | `SYSTEM_HEALTH` (2)           |
| `SYSTEM_ORIGIN_SYS_STATE`     | `FLIGHT_MODE` (3)             |
| `SYSTEM_ORIGIN_PID_ERROR`     | `CONTROL_TRACE` (1030)        |
| (estimator perf)              | `EST_PERF` (1033)             |
| `SYSTEM_ORIGIN_CALIBRATION`   | `CALIBRATION_STATUS` (12320)  |

For the exact payload fields of each message, see `../../navlink/dialect.json` and `../analysis/navlink-v2-spec.md`.

---

## CALIBRATION_STATUS (12320)

This message provides real-time feedback and **operator instructions** during sensor calibration. Payload:

| Field      | Type      | Description                                            |
| ---------- | --------- | ----------------------------------------------------- |
| `step`     | enum `calib_step` | Current calibration step (see table below).   |
| `progress` | u8        | Progress percentage, 0–100.                           |
| `coverage` | f32 × 3   | Live mag axis coverage (range_x, range_y, range_z), valid for `MAG_AXIS_COVERAGE`. |

### Calibration steps (`calib_step`)

| Value | Mnemonic            | Description                                              |
| ----: | :------------------ | :------------------------------------------------------ |
| 0     | `PROGRESS`          | Progress update (`progress` is the percentage).         |
| 1     | `NOSE_UP`           | Place drone nose up (X axis aligned with +g).           |
| 2     | `NOSE_DOWN`         | Place drone nose down (X axis aligned with -g).         |
| 3     | `RIGHT_DOWN`        | Right side down (Y axis aligned with +g).               |
| 4     | `LEFT_DOWN`         | Left side down (Y axis aligned with -g).                |
| 5     | `UPRIGHT`           | Upright (Z axis aligned with +g).                       |
| 6     | `UPSIDE_DOWN`       | Upside down (Z axis aligned with -g).                   |
| 7     | `FREE_ROT`          | Rotate freely in all directions (mag calibration).      |
| 8     | `MAG_AXIS_COVERAGE` | Live mag coverage (`coverage` = range_x, range_y, range_z). |
| 9     | `COMPLETE`          | Terminal: routine finished and persisted OK.            |
| 10    | `FAILED`            | Terminal: routine aborted / fit or save failed.         |
| 11–16 | `EDGE_1`…`EDGE_6`   | Accel edge/corner holds — rest the board on an edge/corner so gravity is shared between axes (the off-diagonal terms of the 3×3 fit). |

> Authoritative mapping: the `calib_step` enum in `../../navlink/dialect.json`.

### Expected Measurement (Developer Reference)

Approximate accelerometer readings expected for each pose, for debugging/validation:

| Pose            | Expected accel (approx) |
| :-------------- | :---------------------- |
| **Nose Up**     | (+g, 0, 0)              |
| **Nose Down**   | (-g, 0, 0)              |
| **Right Down**  | (0, +g, 0)              |
| **Left Down**   | (0, -g, 0)              |
| **Upright**     | (0, 0, +g)              |
| **Upside Down** | (0, 0, -g)              |

## Changelog

| Date    | Author | Description                                            |
| ------- | ------ | ----------------------------------------------------- |
| 06/2026 | —      | NavLink v2: split 0x6 container into per-origin messages |
