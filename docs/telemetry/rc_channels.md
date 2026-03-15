# RC Data

RC Data is a packet that contains the raw values of the Remote Control channels received via iBus. It is sent at 10 Hz.

## Packet Structure (Type 0x5, N=28)

The payload consists of 14 unsigned 16-bit integers representing microsecond (µs) values for each channel.

```mermaid
packet-beta
    0-7: "Sync (0x56) [0:7]"
    8-11: "Protocol Version [8:11]"
    12-15: "Packet Type (0x5) [12:15]"
    16-23: "Payload Length (28) [16:23]"
    24-31: "Device ID [24:31]"
    32-63: "Timestamp (Unix) [32:63]"
    64-79: "Channel 1 (uint16) [64:79]"
    80-95: "Channel 2 (uint16) [80:95]"
    96-271: "Channels 3-13 [96:271]"
    272-287: "Channel 14 (uint16) [272:287]"
    288-319: "CRC32 [288:319]"
```

## Failsafe Detection

If Channel 1 is 0, the receiver is in Failsafe mode or disconnected.

## Changelog

| Date       | Author      | Description     |
| ---------- | ----------- | --------------- |
| 11/03/2026 | Antigravity | Initial version |
