# Attitude Data

Attitude Data is a packet that contains the calculated orientation of the drone (Roll, Pitch, Yaw) in degrees. It is typically sent at 10-20 Hz.

## Packet Structure (Type 0x4, N=12)

The payload consists of three IEEE 754 32-bit floats.

```mermaid
packet-beta
    0-7: "Sync (0x56) [0:7]"
    8-11: "Protocol Version [8:11]"
    12-15: "Packet Type (0x4) [12:15]"
    16-23: "Payload Length (12) [16:23]"
    24-31: "Device ID [24:31]"
    32-63: "Timestamp (Unix) [32:63]"
    64-95: "Roll (float32, deg) [64:95]"
    96-127: "Pitch (float32, deg) [96:127]"
    128-159: "Yaw (float32, deg) [128:159]"
    160-191: "CRC32 [160:191]"
```

## Changelog

| Date       | Author      | Description     |
| ---------- | ----------- | --------------- |
| 11/03/2026 | Antigravity | Initial version |
