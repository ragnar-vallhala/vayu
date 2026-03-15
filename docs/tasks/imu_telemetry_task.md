# Telemetry Task (`imu_telemetry_task`)

**Source File**: `src/comm/telemetry_task.c`
**Stack Size**: 2048
**Priority**: 0
**Loop Rate**: Base loop runs at 150 Hz (6ms delay)

## Description

Prepares and transmits telemetry packets down to the Ground Control Station (GCS) on UART2. Data is popped from the IMU averaging ring buffer logic. The scheduling rate multiplexes based on frame divisors:

- **Full IMU Data** (`PACKET_TYPE_IMU_DATA_FULL`): 1 Hz
- **Compressed Delta IMU** (`PACKET_TYPE_IMU_DATA_COMPRESSED`): 50 Hz
- **Attitude** (`PACKET_TYPE_ATTITUDE`): 10 Hz
- **RC Channels** (`PACKET_TYPE_RC_CHANNELS`): 10 Hz
- **System Status** (`PACKET_TYPE_SYSTEM_STATUS`): 2 Hz
