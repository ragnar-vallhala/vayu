# Heartbeat

Heartbeat is a packet that is sent from the device to the navigator to indicate that the device is alive and well. It is sent at a rate of 1 Hz.

This is a header only packet. It does not have any payload. The length of the payload is 0.

## Packet Structure

```mermaid
packet-beta
    0-7: "Sync (0x56) [0:7]"
    8-11: "Protocol Version [8:11]"
    12-15: "Packet Type [12:15]"
    16-23: "Length (N) [16:23]"
    24-31: "Device ID [24:31]"
    32-63: "Timestamp [32:63]"
    64-95: "CRC32 [64:95]"
```

## Changelog

|Date|Author|Description|
|----|----|----|
|06/03/2026|Ashutosh Vishwakarma|Initial version|