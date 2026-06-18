# Telemetry & Logging Protocol Specification

> Updated for NavLink v2. Authoritative wire spec: navlink/dialect.json + docs/analysis/navlink-v2-spec.md.

**Purpose:** This document indexes the telemetry and logging messages for the Vayu flight controller.

---

## Wire Frame

The retired NAVLINK 1.0 frame (8-byte header with a 4-bit packet type and a trailing CRC32) has been removed. The firmware now emits **NavLink v2 exclusively**: each message is framed by msgid with a `CRC_EXTRA`-seeded CRC-16. The byte-exact frame layout, integrity scheme, and field encodings are defined normatively in:

- `../../navlink/dialect.json` — the message dialect (single source of truth)
- `../analysis/navlink-v2-spec.md` — the wire spec

The pages below describe the per-message semantics; consult the spec/dialect for the on-wire bytes.

---

## Messages

| Page                            | v2 message (msgid)                                                                   | v1 type |
| ------------------------------- | ------------------------------------------------------------------------------------ | ------- |
| [Heartbeat](heartbeat.md)       | `HEARTBEAT` (0)                                                                       | 0x0     |
| [IMU Data](IMU_data.md)         | `IMU_RAW` (1024), `IMU_COMPRESSED` (1025)                                             | 0x1/0x2 |
| [Command](command.md)           | `CMD_*` (8192–8198), acks via `COMMAND_ACK` (5)                                       | 0x3     |
| [Attitude Data](attitude.md)    | `ATTITUDE_EULER` (1026)                                                               | 0x4     |
| [RC Data](rc_channels.md)       | `RC_CHANNELS` (1028)                                                                  | 0x5     |
| [System Status](system_status.md) | `SYSTEM_HEALTH` (2), `FLIGHT_MODE` (3), `CONTROL_TRACE` (1030), `EST_PERF` (1033), `CALIBRATION_STATUS` (12320) | 0x6     |

---

## Changelog

| Date       | Author               | Description                        |
| ---------- | -------------------- | ---------------------------------- |
| 06/03/2026 | Ashutosh Vishwakarma | Initial version                    |
| 11/03/2026 | Antigravity          | Added Attitude and RC Data packets |
| 06/2026    | —                    | Migrated to NavLink v2             |
