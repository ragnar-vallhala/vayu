# IMU Data

IMU Data is a packet that is sent from the device to the navigator to indicate the IMU data. It is sent at a rate of 100 Hz.

There are two types of IMU data packets:

1. Full IMU Data - Packet Code 0x1
2. Delta IMU Data - Packet Code 0x2

## Packet Structure

```mermaid
packet-beta
    0-7: "Sync (0x56) [0:7]"
    8-11: "Protocol Version [8:11]"
    12-15: "Packet Type [12:15]"
    16-23: "Length (N) [16:23]"
    24-31: "Device ID [24:31]"
    32-63: "Timestamp [32:63]"
    64-79: "Accelerometer X [64:79]"
    80-95: "Accelerometer Y [80:95]"
    96-111: "Accelerometer Z [96:111]"
    112-127: "Gyroscope X [112:127]"
    128-143: "Gyroscope Y [128:143]"
    144-159: "Gyroscope Z [144:159]"
    160-175: "Magnetometer X [160:175]"
    176-191: "Magnetometer Y [176:191]"
    192-207: "Magnetometer Z [192:207]"
    208-239: "CRC32 [208:239]"
```

## Changelog

|Date|Author|Description|
|----|----|----|
|06/03/2026|Ashutosh Vishwakarma|Initial version|