#include "sensor_fusion.h"

// ============================
// Helper: Normalize a 3D vector
// ============================
static void normalize3(float v[3]) {
  float mag = m_sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  if (mag > 1e-6f) {
    v[0] /= mag;
    v[1] /= mag;
    v[2] /= mag;
  }
}

// ============================
// Simple Acc + Mag (tilt + heading)
// ============================
void sf_acc_mag(const float ax, const float ay, const float az, const float mx,
                const float my, const float mz, orientation_t *ori) {
  // Normalize accelerometer
  float acc[3] = {ax, ay, az};
  normalize3(acc);

  // Roll & pitch from accelerometer
  float roll = m_atan2(acc[1], acc[2]);
  float pitch = m_atan2(-acc[0], m_sqrt(acc[1] * acc[1] + acc[2] * acc[2]));

  // Tilt-compensated magnetometer
  float mag_x = mx * m_cos(pitch) + mz * m_sin(pitch);
  float mag_y = mx * m_sin(roll) * m_sin(pitch) + my * m_cos(roll) -
                mz * m_sin(roll) * m_cos(pitch);

  float yaw = m_atan2(-mag_y, mag_x);

  // Convert to degrees
  ori->roll = m_rad2deg(roll);
  ori->pitch = m_rad2deg(pitch);
  ori->yaw = m_rad2deg(yaw);
}

// ============================
// Gyroscope integration
// ============================
void sf_gyro(const float gx, const float gy, const float gz, float dt,
             orientation_t *ori) {
  // Integrate gyro rates to get angles
  ori->roll += gx * dt;
  ori->pitch += gy * dt;
  ori->yaw += gz * dt;

  // Keep yaw in [0, 360)
  while (ori->yaw >= 360.0f)
    ori->yaw -= 360.0f;
  while (ori->yaw < 0.0f)
    ori->yaw += 360.0f;
}

// ============================
// Complementary filter fusion
// ============================
void sf_fusion(const float ax, const float ay, const float az, const float mx,
               const float my, const float mz, const float gx, const float gy,
               const float gz, float dt, float alpha, orientation_t *ori) {
  orientation_t acc_mag_ori;
  sf_acc_mag(ax, ay, az, mx, my, mz, &acc_mag_ori);

  // Gyro integration
  orientation_t gyro_ori = *ori; // previous orientation
  sf_gyro(gx, gy, gz, dt, &gyro_ori);

  // Complementary filter: combine gyro (short-term) and acc+mag (long-term)
  ori->roll = alpha * gyro_ori.roll + (1.0f - alpha) * acc_mag_ori.roll;
  ori->pitch = alpha * gyro_ori.pitch + (1.0f - alpha) * acc_mag_ori.pitch;
  ori->yaw = alpha * gyro_ori.yaw + (1.0f - alpha) * acc_mag_ori.yaw;

  // Keep yaw in [0, 360)
  while (ori->yaw >= 360.0f)
    ori->yaw -= 360.0f;
  while (ori->yaw < 0.0f)
    ori->yaw += 360.0f;
}
