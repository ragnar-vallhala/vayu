# BMX160 IMU Task (`bmx160_initiate_read`)

**Source File**: `src/sensor/bmx160.c`
**Stack Size**: 2048
**Priority**: 0
**Loop Rate**: Governed by the `wake_imu_read_task` High-Frequency Timer

## Description

Triggered continuously by a high-frequency timer unblocking the task. It issues per-region asynchronous I2C DMA reads from the BMX160 IMU sensor (separate transfers for the accelerometer/gyroscope, magnetometer, and temperature register regions). Each region's data is resolved into standard physical units by its DMA callback and pushed into the IMU ring buffer via `imu_buffer_push(..)`.
