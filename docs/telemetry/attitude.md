# Attitude Data

Attitude Data is a packet that contains the calculated orientation of the drone (Roll, Pitch, Yaw) in degrees. It is typically sent at 10-20 Hz.

## Packet Structure (Type 0x4, N=12)

The payload consists of three IEEE 754 32-bit floats.

| Bits    | Description     | Unit |
| ------- | --------------- | ---- |
| 64-95   | Roll (float32)  | deg  |
| 96-127  | Pitch (float32) | deg  |
| 128-159 | Yaw (float32)   | deg  |
| 160-191 | CRC32           | -    |

## Changelog

| Date       | Author      | Description     |
| ---------- | ----------- | --------------- |
| 11/03/2026 | Antigravity | Initial version |
