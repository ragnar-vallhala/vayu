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
