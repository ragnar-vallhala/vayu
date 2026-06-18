# Attitude Data

> Updated for NavLink v2. Authoritative wire spec: navlink/dialect.json + docs/analysis/navlink-v2-spec.md.

Attitude reports the estimated orientation and angular rates of the drone, typically at 10–20 Hz.

In NavLink v2 this is **`ATTITUDE_EULER` (msgid 1026)**. The payload is **6 × f32 in RADIANS**, in the **NED** frame:

| Field        | Type | Units |
| ------------ | ---- | ----- |
| `roll`       | f32  | rad   |
| `pitch`      | f32  | rad   |
| `yaw`        | f32  | rad   |
| `rollspeed`  | f32  | rad/s |
| `pitchspeed` | f32  | rad/s |
| `yawspeed`   | f32  | rad/s |

> Note: v1 sent only 3 floats in degrees. v2 adds the angular rates and uses radians.

Byte layout and CRC: `../../navlink/dialect.json`, `../analysis/navlink-v2-spec.md`.

## Changelog

| Date       | Author      | Description                                        |
| ---------- | ----------- | -------------------------------------------------- |
| 11/03/2026 | Antigravity | Initial version                                    |
| 06/2026    | —           | NavLink v2: msgid 1026, 6×f32 radians, NED, +rates |
