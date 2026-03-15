# BMX160 IMU Task (`bmx160_initiate_read`)

**Source File**: `src/sensor/bmx160.c`
**Stack Size**: 2048
**Priority**: 0
**Loop Rate**: Governed by the `wake_imu_read_task` High-Frequency Timer

## Description

Triggered continuously by a high-frequency timer unblocking the task. It initiates an I2C DMA read of 30 bytes from the BMX160 IMU sensor. The transfer captures accelerometer, gyroscope, magnetometer, and temperature data, which are subsequently resolved into standard physical units by a DMA callback and pushed into the IMU ring buffer via `imu_buffer_push(..)`.
