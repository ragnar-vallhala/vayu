#include "maths/sensor_fusion.h"
#include "maths/maths_interface.h"
#include "variables.h"

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
  const float alpha = SF_COMPLEMENTARY_ALPHA;

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

// Helper to convert quaternion to Euler angles
void m_quat_to_euler(const quaternion_t *q, attitude_t *ori) {
  ori->roll = to_degrees(m_atan2(2.0f * (q->w * q->x + q->y * q->z),
                                 1.0f - 2.0f * (q->x * q->x + q->y * q->y)));
  ori->pitch = to_degrees(m_asin(2.0f * (q->w * q->y - q->z * q->x)));
  ori->yaw = to_degrees(m_atan2(2.0f * (q->w * q->z + q->x * q->y),
                                1.0f - 2.0f * (q->y * q->y + q->z * q->z)));
}

// Mahony Filter Implementation
// Gains
#define Kp SF_MAHONY_KP
#define Ki SF_MAHONY_KI

static float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;

void m_mahony_filter(const float ax, const float ay, const float az,
                     const float gx, const float gy, const float gz,
                     const float mx, const float my, const float mz, float dt,
                     attitude_t *ori) {
  float q0 = ori->q.w, q1 = ori->q.x, q2 = ori->q.y, q3 = ori->q.z;
  float norm;
  float hx, hy, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;

  // Auxiliary variables to avoid repeated fractional multiplications
  float q0q0 = q0 * q0;
  float q0q1 = q0 * q1;
  float q0q2 = q0 * q2;
  float q0q3 = q0 * q3;
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q3q3 = q3 * q3;

  // Normalise accelerometer measurement
  norm = m_sqrt(ax * ax + ay * ay + az * az);
  if (norm <= 0.0f)
    return;
  float axn = ax / norm;
  float ayn = ay / norm;
  float azn = az / norm;

  // Normalise magnetometer measurement
  norm = m_sqrt(mx * mx + my * my + mz * mz);
  if (norm <= 0.0f)
    return;
  float mxn = mx / norm;
  float myn = my / norm;
  float mzn = mz / norm;

  // Reference direction of Earth's magnetic field
  hx = 2.0f * mxn * (0.5f - q2q2 - q3q3) + 2.0f * myn * (q1q2 - q0q3) +
       2.0f * mzn * (q1q3 + q0q2);
  hy = 2.0f * mxn * (q1q2 + q0q3) + 2.0f * myn * (0.5f - q1q1 - q3q3) +
       2.0f * mzn * (q2q3 - q0q1);
  bx = m_sqrt(hx * hx + hy * hy);
  bz = 2.0f * mxn * (q1q3 - q0q2) + 2.0f * myn * (q2q3 + q0q1) +
       2.0f * mzn * (0.5f - q1q1 - q2q2);

  // Estimated direction of gravity and magnetic field
  vx = 2.0f * (q1q3 - q0q2);
  vy = 2.0f * (q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;
  wx = 2.0f * bx * (0.5f - q2q2 - q3q3) + 2.0f * bz * (q1q3 - q0q2);
  wy = 2.0f * bx * (q1q2 - q0q3) + 2.0f * bz * (q0q1 + q2q3);
  wz = 2.0f * bx * (q0q2 + q1q3) + 2.0f * bz * (0.5f - q1q1 - q2q2);

  // Error is sum of cross product between estimated direction and measured
  // direction of field vectors
  ex = (ayn * vz - azn * vy) + (myn * wz - mzn * wy);
  ey = (azn * vx - axn * vz) + (mzn * wx - mxn * wz);
  ez = (axn * vy - ayn * vx) + (mxn * wy - myn * wx);

  // Compute and apply integral feedback if enabled
  if (Ki > 0.0f) {
    integralFBx += Ki * ex * dt; // integral error scaled by Ki
    integralFBy += Ki * ey * dt;
    integralFBz += Ki * ez * dt;
  } else {
    integralFBx = 0.0f; // prevent integral windup
    integralFBy = 0.0f;
    integralFBz = 0.0f;
  }

  // Apply proportional feedback
  float gxc = gx + Kp * ex + integralFBx;
  float gyc = gy + Kp * ey + integralFBy;
  float gzc = gz + Kp * ez + integralFBz;

  // Integrate rate of change of quaternion
  float dq0 = 0.5f * (-q1 * gxc - q2 * gyc - q3 * gzc);
  float dq1 = 0.5f * (q0 * gxc + q2 * gzc - q3 * gyc);
  float dq2 = 0.5f * (q0 * gyc - q1 * gzc + q3 * gxc);
  float dq3 = 0.5f * (q0 * gzc + q1 * gyc - q2 * gxc);

  q0 += dq0 * dt;
  q1 += dq1 * dt;
  q2 += dq2 * dt;
  q3 += dq3 * dt;

  // Normalise quaternion
  norm = m_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  ori->q.w = q0 / norm;
  ori->q.x = q1 / norm;
  ori->q.y = q2 / norm;
  ori->q.z = q3 / norm;

  // Convert to Euler for external use
  m_quat_to_euler(&ori->q, ori);
}
