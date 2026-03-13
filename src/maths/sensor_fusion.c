#include "maths/sensor_fusion.h"
#include "maths/maths_interface.h"

void m_acc_mag(const float ax, const float ay, const float az, const float mx,
               const float my, const float mz, attitude_t *ori) {
  float ax_n = ax;
  float ay_n = ay;
  float az_n = az;

  float norm_a = m_sqrt(ax * ax + ay * ay + az * az);
  if (norm_a > 0.0f) {
    ax_n /= norm_a;
    ay_n /= norm_a;
    az_n /= norm_a;
  }

  float mx_n = mx;
  float my_n = my;
  float mz_n = mz;

  float norm_m = m_sqrt(mx * mx + my * my + mz * mz);
  if (norm_m > 0.0f) {
    mx_n /= norm_m;
    my_n /= norm_m;
    mz_n /= norm_m;
  }

  // Roll and Pitch
  ori->roll = to_degrees(m_atan2(ay_n, az_n));
  ori->pitch = to_degrees(m_atan2(-ax_n, m_sqrt(ay_n * ay_n + az_n * az_n)));

  float sin_roll = m_sin(to_radians(ori->roll));
  float cos_roll = m_cos(to_radians(ori->roll));
  float sin_pitch = m_sin(to_radians(ori->pitch));
  float cos_pitch = m_cos(to_radians(ori->pitch));

  float mx2 = mx_n * cos_pitch + mz_n * sin_pitch;
  float my2 = mx_n * sin_roll * sin_pitch + my_n * cos_roll -
              mz_n * sin_roll * cos_pitch;

  // Yaw (Tilt-compensated)
  ori->yaw = to_degrees(m_atan2(-my2, mx2));
}

void m_complementary_filter(const float ax, const float ay, const float az,
                            const float gx, const float gy, const float gz,
                            const float mx, const float my, const float mz,
                            float dt, attitude_t *ori) {
  // 1. Get accelerometer/magnetometer based orientation (noisy but stable)
  attitude_t acc_mag_ori;
  m_acc_mag(ax, ay, az, mx, my, mz, &acc_mag_ori);

  // 2. Complementary Filter
  // Roll and Pitch: Integrate gyro and fuse with acc
  // Alpha typically 0.96 to 0.99
  const float alpha = 0.98f;

  ori->roll = alpha * (ori->roll + gx * dt) + (1.0f - alpha) * acc_mag_ori.roll;
  ori->pitch =
      alpha * (ori->pitch + gy * dt) + (1.0f - alpha) * acc_mag_ori.pitch;

  // Yaw: Integrate gyro and fuse with mag-based yaw
  ori->yaw = alpha * (ori->yaw + gz * dt) + (1.0f - alpha) * acc_mag_ori.yaw;

  // Normalization for angles
  if (ori->roll > 180.0f)
    ori->roll -= 360.0f;
  if (ori->roll < -180.0f)
    ori->roll += 360.0f;
  if (ori->pitch > 180.0f)
    ori->pitch -= 360.0f;
  if (ori->pitch < -180.0f)
    ori->pitch += 360.0f;
  if (ori->yaw > 180.0f)
    ori->yaw -= 360.0f;
  if (ori->yaw < -180.0f)
    ori->yaw += 360.0f;
}
