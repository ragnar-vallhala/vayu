# Telemetry Specification

This directory contains the protocol definitions for serial communication between the Vayu firmware and the Ground Control Station (GCS).

> **Updated for NavLink v2.** The firmware now emits NavLink v2 frames exclusively
> (24-bit msgids + CRC_EXTRA); the hand-rolled v1 wire format (4-bit packet types
> `0x0–0x6`) is retired. The **authoritative** wire spec is
> [`../navlink-v2-spec.md`](../navlink-v2-spec.md) and the message
> dialect is [`../../../dialect.json`](../../../dialect.json). The docs
> below cover the timeless payload concepts; defer to the dialect for exact field
> layouts.

## Packet Overview

### Data Packets (Firmware -> GCS)

| Document                          | v2 message (msgid)            | was (v1) | Description                                          |
| :-------------------------------- | :---------------------------- | :------- | :--------------------------------------------------- |
| [Heartbeat](heartbeat.md)         | `HEARTBEAT` (0)               | `0x00`   | System life-check and high-level nav state.          |
| [IMU Data](IMU_data.md)           | `IMU_RAW` (1024) / `IMU_COMPRESSED` (1025) | `0x01`/`0x02` | Raw and compressed sensor readings (Accel, Gyro, Mag). |
| [Attitude](attitude.md)           | `ATTITUDE_EULER` (1026)       | `0x04`   | Estimated orientation + body rates (radians, NED).   |
| [System Status](system_status.md) | `SYSTEM_HEALTH` (2), `FLIGHT_MODE` (3), `CONTROL_TRACE` (1030), `EST_PERF` (1033), `CALIBRATION_STATUS` (12320) | `0x06` | The v1 status container split into discrete messages. |
| [RC Channels](rc_channels.md)     | `RC_CHANNELS` (1028)          | `0x05`   | Passthrough of radio control input values (18 ch).   |

### Control Packets (GCS -> Firmware)

| Document                          | v2 message (msgid)                 | was (v1) | Description                                       |
| :-------------------------------- | :--------------------------------- | :------- | :------------------------------------------------ |
| [Commands](command.md)            | `CMD_*` (8192–8198), acked by `COMMAND_ACK` (5) | `0x03` | Remote actions (Arm, Calibrate, PID/LPF/geometry/mode tuning). |
| [Stream Rates](stream_rates.md)   | *superseded by `XFER_*`*           | —        | Per-stream rate control, folded into the xfer substrate (not shipped standalone). |

### Bidirectional Packets (GCS <-> Firmware)

| Document                  | v2 message (msgid) | Description                                                |
| :------------------------ | :----------------- | :--------------------------------------------------------- |
| [Time Sync](time_sync.md) | `TIME_SYNC` (10)   | NTP-style clock-sync handshake (replaces the heartbeat timestamp jam). |
| [Bulk Transfer](xfer.md)  | `XFER_OPEN` (8201), `XFER_INFO` (1042), `XFER_DATA` (1043), `XFER_ACK` (1044), `XFER_CLOSE` (8202) | Generic FTP-like file transfer + live streaming substrate (provider-based). |
| [Filesystem Nav](fs_nav.md) | `FS_LIST` (8203), `FS_ENTRY` (1045), `FS_INFO` (8204), `FS_INFO_REPLY` (1046) | Browse the SD: list a directory, stat a path (missing → DENIED). |
