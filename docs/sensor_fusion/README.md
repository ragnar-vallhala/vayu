# Sensor Fusion Algorithms

This directory contains documentation for the various sensor fusion algorithms implemented in the Vayu flight controller.

## Implemented Algorithms

- [Complementary Filter](complementary_filter.md): A simple Euler-based filter that combines accelerometer and gyroscope data.
- [Mahony Filter](mahony_filter.md): A robust AHRS filter using quaternions for orientation estimation.

## Configuration

The active algorithm and its parameters are configured in `include/variables.h`.
