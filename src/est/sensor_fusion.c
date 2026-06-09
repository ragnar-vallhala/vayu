#include "est/est.h"
#include "maths/maths_interface.h"
#include "navhal.h"
#include "sys/state.h"
#include "utils.h"              /* v_get_ticks (vaios) */
#include "logger/logger.h"        /* vayu_log */
#include "vaios_config_default.h"
#include "variables.h"

/* ----------------------------------------------------------------------------
 * Estimator health (EST-MAH-002 / SYS-SAFE-003) — see sensor_fusion.h.
 *
 * Single-writer (the IMU/fusion task) / multi-reader (telemetry, CTRL,
 * safety task). Each field is a 32-bit or smaller aligned scalar, so a
 * volatile load/store is atomic on Cortex-M4 (R8.6: no locks in hot
 * loops).
 * --------------------------------------------------------------------------*/
static volatile bool     s_in_rejection      = false;
static volatile uint32_t s_rejection_start_ms = 0;
static volatile bool     s_degraded          = false;

/**
 * @implements EST-MAH-002
 */
void estimator_mark_sample(bool valid) {
  if (valid) {
    if (s_degraded) {
      vayu_log("[EST] degraded CLEARED");
    }
    s_in_rejection = false;
    s_degraded     = false;
    return;
  }
  uint32_t now = v_get_ticks();
  if (!s_in_rejection) {
    s_in_rejection       = true;
    s_rejection_start_ms = now;
    return;
  }
  if (!s_degraded && (now - s_rejection_start_ms) > EST_DEGRADED_TIMEOUT_MS) {
    s_degraded = true;
    vayu_log("[EST] degraded RAISED (continuous reject %u ms)",
             (unsigned)(now - s_rejection_start_ms));
  }
}

/**
 * @implements EST-MAH-002, SYS-SAFE-003
 */
bool estimator_is_degraded(void) {
  return s_degraded;
}

/* States in which an estimator failure should drive FAILSAFE. Mirrors
 * the RC watchdog gating in src/comm/rc_task.c — INIT and CALIBRATING
 * are spared. */
static bool estimator_safety_state_active(sys_state_t s) {
  return s == SYSTEM_STATE_STANDBY || s == SYSTEM_STATE_PREARM ||
         s == SYSTEM_STATE_ARMED   || s == SYSTEM_STATE_IN_AIR;
}

/**
 * @implements SYS-SAFE-003
 */
void estimator_safety_step(void) {
  if (!s_degraded) {
    return;
  }
  sys_state_t cur = system_state_get();
  if (estimator_safety_state_active(cur) && cur != SYSTEM_STATE_FAILSAFE) {
    VAYU_DISCARD(system_state_set(SYSTEM_STATE_FAILSAFE));
  }
}

static inline float get_dt(void) {
  static uint32_t last_dwt = 0; // used to calculate dt
  uint32_t now = hal_cycle_counter_get();
  // convert to s
  float dt = ((float)(now - last_dwt)) / (float)SYS_CLOCK_FREQ;
  if (dt < 1e-3f)
    dt = 1e-3f;
  last_dwt = now;
  return dt;
}
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
  ori->roll = to_degrees(m_atan2(-ay_n, az_n));
  ori->pitch = -to_degrees(m_atan2(ax_n, m_sqrt(ay_n * ay_n + az_n * az_n)));

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
                            attitude_t *ori) {
  float dt = get_dt();
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

  // Yaw: fuse the mag if present, with a heavier mag weight than roll/pitch use
  // (the mag is the only absolute heading reference; too little weight and the
  // heading drifts on gyro bias and yaw-hold chases it).
  float norm_m = m_sqrt(mx * mx + my * my + mz * mz);
  if (norm_m > 1e-6f) {
    const float ya = SF_YAW_COMPLEMENTARY_ALPHA;
    ori->yaw = ya * (ori->yaw + gz * dt) + (1.0f - ya) * acc_mag_ori.yaw;
  } else {
    // Gyro only integration for yaw
    ori->yaw = ori->yaw + gz * dt;
  }

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

static void m_quat_to_euler(const quaternion_t *q, attitude_t *ori) {
  // Roll (X-axis rotation)
  ori->roll = to_degrees(m_atan2(2.0f * (q->w * q->x + q->y * q->z),
                                 1.0f - 2.0f * (q->x * q->x + q->y * q->y)));

  // Pitch (Y-axis rotation) — CLAMP HERE
  float sinp = 2.0f * (q->w * q->y - q->z * q->x);

  // Clamp to [-1, 1]
  if (sinp > 1.0f)
    sinp = 1.0f;
  else if (sinp < -1.0f)
    sinp = -1.0f;

  ori->pitch = to_degrees(m_asin(sinp));

  // Yaw (Z-axis rotation)
  ori->yaw = to_degrees(m_atan2(2.0f * (q->w * q->z + q->x * q->y),
                                1.0f - 2.0f * (q->y * q->y + q->z * q->z)));
}

// Mahony Filter Implementation
// Gains
#define Kp SF_MAHONY_KP
#define Ki SF_MAHONY_KI

/* EST-MAH-105: per-axis bound on the Mahony integral feedback term.
 * Without it a small persistent error (sensor bias, ground vibration)
 * winds integralFB up unbounded over hours of runtime, eventually
 * poisoning the rate setpoint. 0.5 rad/s is well above any legitimate
 * steady-state gyro bias correction yet far below a flight rate. */
#define SF_MAHONY_INTEGRAL_LIMIT 0.5f

static float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;

/**
 * @brief Re-initialise the Mahony integral feedback state to zero.
 *
 * Drops any accumulated integral windup. The caller owns the quaternion
 * (attitude_t), which it re-seeds to identity separately (EST-MAH-106).
 *
 * @implements EST-MAH-105
 */
void estimator_reset(void) {
  integralFBx = 0.0f;
  integralFBy = 0.0f;
  integralFBz = 0.0f;
}

void m_mahony_filter(const float ax, const float ay, const float az,
                     const float gx, const float gy, const float gz,
                     const float mx, const float my, const float mz,
                     attitude_t *ori) {
  float dt = get_dt();
  float q0 = ori->q.w, q1 = ori->q.x, q2 = ori->q.y, q3 = ori->q.z;
  float norm;
  float hx, hy;
  float vx, vy, vz;
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

  // Tilt-compensated, YAW-ONLY mag fusion. The classic full-3D mag error term
  // (m × b) couples the mag's noise/bias (and any field-convention error) into
  // roll & pitch and diverges the estimate during a maneuver. Instead the mag
  // is used purely as an absolute HEADING reference: rotate it into the world
  // frame, take the horizontal heading error, and feed the correction back ONLY
  // about the world-vertical (gravity) axis. Roll/pitch stay pinned by the
  // accelerometer alone, so a noisy/late mag can never tip the airframe.
  int mag_valid = 0;
  float mxn = 0.0f, myn = 0.0f, mzn = 0.0f;
  norm = m_sqrt(mx * mx + my * my + mz * mz);
  if (norm > 1e-6f) {
    mxn = mx / norm;
    myn = my / norm;
    mzn = mz / norm;
    mag_valid = 1;
  }

  // Estimated direction of gravity (ALWAYS computed)
  vx = -2.0f * (q1q3 - q0q2);
  vy = -2.0f * (q0q1 + q2q3);
  vz = -(q0q0 - q1q1 - q2q2 + q3q3);

  // Tilt error from the accelerometer (cross of measured vs estimated gravity)
  // — pins roll & pitch. Always applied.
  ex = (ayn * vz - azn * vy);
  ey = (azn * vx - axn * vz);
  ez = (axn * vy - ayn * vx);

  // Yaw error from the magnetometer, applied about the world vertical only.
  if (mag_valid) {
    // Rotate the unit mag into the world frame; (hx, hy) is its horizontal
    // projection. With the field pointing magnetic north (declination 0) a
    // correct heading gives hy = 0, so atan2(hy, hx) IS the heading error.
    hx = 2.0f * mxn * (0.5f - q2q2 - q3q3) + 2.0f * myn * (q1q2 - q0q3) +
         2.0f * mzn * (q1q3 + q0q2);
    hy = 2.0f * mxn * (q1q2 + q0q3) + 2.0f * myn * (0.5f - q1q1 - q3q3) +
         2.0f * mzn * (q2q3 - q0q1);
    const float heading_err = m_atan2(hy, hx);
    // Feed the heading error back about the WORLD vertical only: v is the
    // world-down direction expressed in the body frame (already unit length),
    // so heading_err·v is a pure tilt-compensated yaw correction — it can never
    // disturb the gravity-stabilised roll/pitch. Sign drives heading_err -> 0.
    const float mw = SF_MAHONY_MAG_WEIGHT;
    ex += mw * heading_err * vx;
    ey += mw * heading_err * vy;
    ez += mw * heading_err * vz;
  }

  // Compute and apply integral feedback if enabled
  if (Ki > 0.0f) {
    integralFBx += Ki * ex * dt; // integral error scaled by Ki
    integralFBy += Ki * ey * dt;
    integralFBz += Ki * ez * dt;

    /* EST-MAH-105: clamp each axis to ±SF_MAHONY_INTEGRAL_LIMIT rad/s. */
    if (integralFBx > SF_MAHONY_INTEGRAL_LIMIT)  integralFBx = SF_MAHONY_INTEGRAL_LIMIT;
    if (integralFBx < -SF_MAHONY_INTEGRAL_LIMIT) integralFBx = -SF_MAHONY_INTEGRAL_LIMIT;
    if (integralFBy > SF_MAHONY_INTEGRAL_LIMIT)  integralFBy = SF_MAHONY_INTEGRAL_LIMIT;
    if (integralFBy < -SF_MAHONY_INTEGRAL_LIMIT) integralFBy = -SF_MAHONY_INTEGRAL_LIMIT;
    if (integralFBz > SF_MAHONY_INTEGRAL_LIMIT)  integralFBz = SF_MAHONY_INTEGRAL_LIMIT;
    if (integralFBz < -SF_MAHONY_INTEGRAL_LIMIT) integralFBz = -SF_MAHONY_INTEGRAL_LIMIT;
  } else {
    integralFBx = 0.0f; // prevent integral windup
    integralFBy = 0.0f;
    integralFBz = 0.0f;
  }

  // Apply proportional feedback
  float gxc = to_radians(gx) + Kp * ex + integralFBx;
  float gyc = to_radians(gy) + Kp * ey + integralFBy; // gy is nose down
  float gzc = to_radians(gz) + Kp * ez + integralFBz;

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
