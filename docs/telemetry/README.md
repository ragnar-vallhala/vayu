# Telemetry Specification

This directory contains the protocol definitions for serial communication between the Vayu firmware and the Ground Control Station (GCS).

## Packet Overview

The Vayu protocol uses a framed binary format (defined in [Data Frame](data_frame.md)) to encapsulate various telemetry and command types.

### Data Packets (Firmware -> GCS)

| Document                          | Packet Type     | Description                                            |
| :-------------------------------- | :-------------- | :----------------------------------------------------- |
| [Heartbeat](heartbeat.md)         | `0x00`          | System life-check and high-level state.                |
| [IMU Data](IMU_data.md)           | `0x01` / `0x02` | Raw and filtered sensor readings (Accel, Gyro, Mag).   |
| [Attitude](attitude.md)           | `0x04`          | Estimated orientation (Roll, Pitch, Yaw).              |
| [System Status](system_status.md) | `0x06`          | Calibration progress, error codes, and health metrics. |
| [RC Channels](rc_channels.md)     | `0x05`          | Passthrough of radio control input values.             |

### Control Packets (GCS -> Firmware)

| Document               | Packet Type | Description                                          |
| :--------------------- | :---------- | :--------------------------------------------------- |
| [Commands](command.md) | `0x03`      | Remote control actions (Arm, Calibrate, PID tuning). |