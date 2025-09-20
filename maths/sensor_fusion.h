#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include "dsp_interface.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// =======================
// Orientation Structure
// =======================
typedef struct {
  float roll;  // Rotation around X-axis (degrees)
  float pitch; // Rotation around Y-axis (degrees)
  float yaw;   // Rotation around Z-axis (degrees, compass heading)
} orientation_t;

// =======================
// Sensor Fusion Functions
// =======================

// -----------------------
// Simple Acc + Mag
// -----------------------
// Computes roll, pitch, and yaw using only accelerometer + magnetometer
// ax, ay, az : accelerometer raw or converted (m/s²)
// mx, my, mz : magnetometer raw or converted (µT)
void sf_acc_mag(const float ax, const float ay, const float az, const float mx,
                const float my, const float mz, orientation_t *ori);

// -----------------------
// Gyroscope integration
// -----------------------
// Computes orientation using gyroscope only
// gx, gy, gz : gyroscope rates (°/s)
// dt         : time delta in seconds
void sf_gyro(const float gx, const float gy, const float gz, float dt,
             orientation_t *ori);

// -----------------------
// Sensor fusion (complementary filter)
// -----------------------
// Combines accel+mag and gyro to get stable orientation
// alpha : complementary filter factor (0 < alpha < 1)
// dt    : time delta in seconds
void sf_fusion(const float ax, const float ay, const float az, const float mx,
               const float my, const float mz, const float gx, const float gy,
               const float gz, float dt, float alpha, orientation_t *ori);

#ifdef __cplusplus
}
#endif

#endif // SENSOR_FUSION_H
