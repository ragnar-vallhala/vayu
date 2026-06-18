# Sensor Fusion Algorithms

This directory contains documentation for the various sensor fusion algorithms implemented in the Vayu flight controller.

## Implemented Algorithms

- [Complementary Filter](complementary_filter.md): A simple Euler-based filter that combines accelerometer and gyroscope data.
- [Mahony Filter](mahony_filter.md): A robust AHRS filter using quaternions for orientation estimation.
- **EKF** (`src/est/ekf.c`): An Extended Kalman Filter estimating attitude (and gyro bias). This is the **default active filter** (`SF_FILTER_USED = SF_EKF` in `include/variables.h`); the Complementary and Mahony filters are retained as selectable alternatives.

## Configuration

The active algorithm and its parameters are configured in `include/variables.h` (see `SF_FILTER_USED`). The filter dispatch lives in `src/est/sensor_fusion.c`.
