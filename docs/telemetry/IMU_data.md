# IMU Data

IMU Data is a packet that is sent from the device to the navigator to indicate the IMU data. It is sent at a rate of 100 Hz.

There are two types of IMU data packets:

1. Full IMU Data - Packet Code 0x1
2. Delta IMU Data - Packet Code 0x2

## Packet Structure

IMU packets use an 8-byte header followed by the payload and a 4-byte CRC32.

### Full IMU Data (Type 0x1, N=40)

Sent every 10 samples or when delta exceeds threshold. Payload consists of 10 IEEE 754 32-bit floats.

```mermaid
packet-beta
    0-7: "Sync (0x56) [0:7]"
    8-11: "Protocol Version [8:11]"
    12-15: "Packet Type (0x1) [12:15]"
    16-23: "Payload Length (40) [16:23]"
    24-31: "Device ID [24:31]"
    32-63: "Timestamp (Unix) [32:63]"
    64-95: "Accelerometer X (float32) [64:95]"
    96-127: "Accelerometer Y (float32) [96:127]"
    128-159: "Accelerometer Z (float32) [128:159]"
    160-191: "Gyroscope X (float32) [160:191]"
    192-223: "Gyroscope Y (float32) [192:223]"
    224-255: "Gyroscope Z (float32) [224:255]"
    256-287: "Magnetometer X (float32) [256:287]"
    288-319: "Magnetometer Y (float32) [288:319]"
    320-351: "Magnetometer Z (float32) [320:351]"
    352-383: "Temperature (°C) (float32) [352:383]"
    384-415: "CRC32 [384:415]"
```

### Delta IMU Data (Type 0x2, N=20)

Sent for intermediate updates. Payload consists of 10 IEEE 754 16-bit half-precision floats (`f16`). These represent the difference from the last Full IMU Data packet.

```mermaid
packet-beta
    0-7: "Sync (0x56) [0:7]"
    8-11: "Protocol Version [8:11]"
    12-15: "Packet Type (0x2) [12:15]"
    16-23: "Payload Length (20) [16:23]"
    24-31: "Device ID [24:31]"
    32-63: "Timestamp (Unix) [32:63]"
    64-79: "Delta Acc X (f16) [64:79]"
    80-95: "Delta Acc Y (f16) [80:95]"
    96-111: "Delta Acc Z (f16) [96:111]"
    112-127: "Delta Gyr X (f16) [112:127]"
    128-143: "Delta Gyr Y (f16) [128:143]"
    144-159: "Delta Gyr Z (f16) [144:159]"
    160-175: "Delta Mag X (f16) [160:175]"
    176-191: "Delta Mag Y (f16) [176:191]"
    192-207: "Delta Mag Z (f16) [192:207]"
    208-223: "Delta Temp (f16) [208:223]"
    224-255: "CRC32 [224:255]"
```

## Changelog

| Date       | Author               | Description     |
| ---------- | -------------------- | --------------- |
| 06/03/2026 | Ashutosh Vishwakarma | Initial version |
