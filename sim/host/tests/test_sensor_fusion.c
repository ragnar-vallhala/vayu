/* Accel/mag -> Euler attitude helper (firmware/src/est/sensor_fusion.c).
 *
 * m_acc_mag() is pure (no RTOS, no state) and is the roll/pitch reference the
 * complementary path fuses against, so its sign convention must match the rest
 * of the stack. The driver reports the GRAVITY vector — bmx160_convert negates
 * X and Z, so a LEVEL board reads ~-g on Z — and the EKF models world-DOWN in
 * body, gb = R^T(0,0,-1) (ekf.c: "using world-up here flips the estimate
 * 180deg"). World-UP in body is therefore -a/|a|, giving for ZYX:
 *
 *     roll  = atan2(-ay, -az)
 *     pitch = atan2( ax, hypot(ay, az))
 *
 * NOTE: yaw is deliberately NOT asserted here. Its tilt compensation was built
 * on the old (180deg-flipped) roll/pitch and has never been validated against
 * hardware; the complementary path is inactive at runtime (SF_FILTER_USED ==
 * SF_EKF), so it is left alone rather than "fixed" blind. */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "est/est.h"

static int g_checks = 0, g_fails = 0;
#define CHECK(cond, msg)                                                       \
  do {                                                                         \
    g_checks++;                                                                \
    if (!(cond)) {                                                             \
      g_fails++;                                                               \
      printf("    FAIL: %s\n", (msg));                                         \
    }                                                                          \
  } while (0)

#define G 9.80665f
#define CLOSE(a, b, tol) (fabsf((a) - (b)) < (tol))

/* m_mahony_filter integrates ori->q in place, so it must start as the identity
 * quaternion -- a zeroed attitude_t normalises 0/0 and goes NaN on step one.
 * attitude_task.c:72 does the same thing before the estimator's first sample. */
static attitude_t mahony_state(void) {
  attitude_t o;
  memset(&o, 0, sizeof o);
  o.q.w = 1.0f;
  return o;
}

/* Gravity vector in body for a given roll/pitch, in the driver's convention:
 * a = -g * (world-up expressed in body). */
static void grav_body(float roll_deg, float pitch_deg, float out[3]) {
  float r = roll_deg * (float)M_PI / 180.0f;
  float p = pitch_deg * (float)M_PI / 180.0f;
  float up[3] = {-sinf(p), sinf(r) * cosf(p), cosf(r) * cosf(p)};
  for (int i = 0; i < 3; i++)
    out[i] = -G * up[i];
}

int main(void) {
  printf("test_sensor_fusion: m_acc_mag attitude convention\n");
  /* A benign mag reading — present so the yaw branch runs, never asserted. */
  const float mx = 20.0f, my = 0.0f, mz = -40.0f;

  printf("  [1] a level board reads zero roll and pitch\n");
  {
    attitude_t o = {0};
    m_acc_mag(0.0f, 0.0f, -G, mx, my, mz, &o);
    CHECK(CLOSE(o.roll, 0.0f, 0.05f), "level -> roll ~ 0 (not 180)");
    CHECK(CLOSE(o.pitch, 0.0f, 0.05f), "level -> pitch ~ 0");
  }

  printf("  [2] roll and pitch recover the pose that generated the vector\n");
  {
    const float poses[][2] = {{0, 0},   {10, 0},  {-10, 0}, {0, 10},
                              {0, -10}, {20, 15}, {-25, -8}};
    for (unsigned i = 0; i < sizeof poses / sizeof poses[0]; i++) {
      float a[3];
      grav_body(poses[i][0], poses[i][1], a);
      attitude_t o = {0};
      m_acc_mag(a[0], a[1], a[2], mx, my, mz, &o);
      char b[80];
      snprintf(b, sizeof b, "roll %.0f recovered (got %.2f)", poses[i][0],
               o.roll);
      CHECK(CLOSE(o.roll, poses[i][0], 0.05f), b);
      snprintf(b, sizeof b, "pitch %.0f recovered (got %.2f)", poses[i][1],
               o.pitch);
      CHECK(CLOSE(o.pitch, poses[i][1], 0.05f), b);
    }
  }

  printf("  [3] signs match the EKF: right-bank positive roll, nose-up "
         "positive pitch\n");
  {
    attitude_t o = {0};
    /* Bank right by 30deg: gravity leans onto -Y in this convention. */
    float a[3];
    grav_body(30.0f, 0.0f, a);
    m_acc_mag(a[0], a[1], a[2], mx, my, mz, &o);
    CHECK(o.roll > 25.0f && o.roll < 35.0f, "right bank -> positive roll");
    /* Nose up by 30deg. */
    grav_body(0.0f, 30.0f, a);
    m_acc_mag(a[0], a[1], a[2], mx, my, mz, &o);
    CHECK(o.pitch > 25.0f && o.pitch < 35.0f, "nose up -> positive pitch");
  }

  printf("  [4] a zero accel vector does not produce NaN\n");
  {
    attitude_t o = {0};
    m_acc_mag(0.0f, 0.0f, 0.0f, mx, my, mz, &o);
    CHECK(!isnan(o.roll) && !isnan(o.pitch), "degenerate input stays finite");
  }

  /* ------------------------------------------------------------------
   * The two filters m_acc_mag feeds. Neither is the runtime path
   * (SF_FILTER_USED == SF_EKF), which is exactly why they need a test: an
   * unexercised fallback rots quietly and is only discovered the day the EKF
   * is switched off to debug something else.
   * ------------------------------------------------------------------ */

  printf("  [5] complementary: gyro dominates over one step, accel anchors\n");
  {
    /* alpha = 0.98, so one step of a 100 deg/s roll rate at 10 ms should move
     * roll by ~0.98 deg, not by the accel's answer. */
    attitude_t o = {0};
    float a[3];
    grav_body(0.0f, 0.0f, a);
    m_complementary_filter(a[0], a[1], a[2], 100.0f, 0.0f, 0.0f, mx, my, mz,
                           0.01f, &o);
    CHECK(CLOSE(o.roll, 0.98f, 0.05f),
          "one step tracks the gyro, scaled by alpha");

    /* Held level with zero rate, the accel must pull the estimate back to 0. */
    o.roll = 10.0f;
    for (int i = 0; i < 500; i++)
      m_complementary_filter(a[0], a[1], a[2], 0.0f, 0.0f, 0.0f, mx, my, mz,
                             0.01f, &o);
    CHECK(fabsf(o.roll) < 0.5f, "a level accel drags roll back to zero");
  }

  printf("  [6] complementary: yaw falls back to gyro-only without a mag\n");
  {
    attitude_t with_mag = {0}, no_mag = {0};
    float a[3];
    grav_body(0.0f, 0.0f, a);
    for (int i = 0; i < 100; i++) {
      m_complementary_filter(a[0], a[1], a[2], 0.0f, 0.0f, 10.0f, mx, my, mz,
                             0.01f, &with_mag);
      m_complementary_filter(a[0], a[1], a[2], 0.0f, 0.0f, 10.0f, 0.0f, 0.0f,
                             0.0f, 0.01f, &no_mag);
    }
    /* 10 deg/s for 1 s: the gyro-only branch integrates it whole; the fused
     * branch is dragged back toward the (static) mag heading. */
    CHECK(CLOSE(no_mag.yaw, 10.0f, 0.5f), "no mag -> pure gyro integration");
    CHECK(fabsf(with_mag.yaw) < fabsf(no_mag.yaw),
          "a mag present pulls yaw back toward its heading");
  }

  printf("  [7] complementary: angles stay wrapped into [-180, 180]\n");
  {
    /* Drive each axis past the wrap point and confirm it comes back inside. */
    attitude_t o = {0};
    float a[3];
    grav_body(0.0f, 0.0f, a);
    o.roll = 179.0f;
    o.pitch = 179.0f;
    o.yaw = 179.0f;
    m_complementary_filter(a[0], a[1], a[2], 500.0f, 500.0f, 500.0f, 0.0f, 0.0f,
                           0.0f, 0.01f, &o);
    CHECK(o.roll >= -180.0f && o.roll <= 180.0f, "roll wrapped");
    CHECK(o.pitch >= -180.0f && o.pitch <= 180.0f, "pitch wrapped");
    CHECK(o.yaw >= -180.0f && o.yaw <= 180.0f, "yaw wrapped");

    o.roll = -179.0f;
    o.pitch = -179.0f;
    o.yaw = -179.0f;
    m_complementary_filter(a[0], a[1], a[2], -500.0f, -500.0f, -500.0f, 0.0f,
                           0.0f, 0.0f, 0.01f, &o);
    CHECK(o.roll >= -180.0f && o.roll <= 180.0f, "roll wrapped, negative");
    CHECK(o.pitch >= -180.0f && o.pitch <= 180.0f, "pitch wrapped, negative");
    CHECK(o.yaw >= -180.0f && o.yaw <= 180.0f, "yaw wrapped, negative");
  }

  printf("  [8] mahony: converges to the accel attitude from rest\n");
  {
    attitude_t o = mahony_state();
    float a[3];
    grav_body(20.0f, 0.0f, a);
    for (int i = 0; i < 4000; i++)
      m_mahony_filter(a[0], a[1], a[2], 0.0f, 0.0f, 0.0f, mx, my, mz, 0.002f,
                      &o);
    CHECK(CLOSE(o.roll, 20.0f, 2.0f), "settles on a 20 deg bank");

    attitude_t p = mahony_state();
    grav_body(0.0f, -15.0f, a);
    for (int i = 0; i < 4000; i++)
      m_mahony_filter(a[0], a[1], a[2], 0.0f, 0.0f, 0.0f, mx, my, mz, 0.002f,
                      &p);
    CHECK(CLOSE(p.pitch, -15.0f, 2.0f), "settles on a 15 deg nose-down");
  }

  printf("  [9] mahony: a zero accel vector is ignored, not integrated\n");
  {
    /* norm <= 0 has to bail BEFORE the divide, or the estimate goes NaN and
     * never recovers -- an IMU dropout would take the attitude with it. */
    attitude_t o = mahony_state();
    float a[3];
    grav_body(10.0f, 0.0f, a);
    for (int i = 0; i < 2000; i++)
      m_mahony_filter(a[0], a[1], a[2], 0.0f, 0.0f, 0.0f, mx, my, mz, 0.002f,
                      &o);
    const float before = o.roll;
    for (int i = 0; i < 50; i++)
      m_mahony_filter(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, mx, my, mz, 0.002f,
                      &o);
    CHECK(!isnan(o.roll) && !isnan(o.pitch) && !isnan(o.yaw),
          "a dead accel does not produce NaN");
    CHECK(CLOSE(o.roll, before, 0.001f), "and leaves the estimate untouched");
  }

  printf("  [10] mahony: runs without a mag, and the mag moves yaw\n");
  {
    attitude_t no_mag = mahony_state();
    float a[3];
    grav_body(0.0f, 0.0f, a);
    for (int i = 0; i < 2000; i++)
      m_mahony_filter(a[0], a[1], a[2], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                      0.002f, &no_mag);
    CHECK(!isnan(no_mag.yaw), "a zero mag keeps the filter finite");
    CHECK(fabsf(no_mag.roll) < 2.0f && fabsf(no_mag.pitch) < 2.0f,
          "and roll/pitch still pin to the accel");

    /* Roll/pitch must stay accel-pinned even with a mag pointing somewhere
     * unhelpful -- that is the whole reason the mag is yaw-only here. */
    attitude_t skewed = mahony_state();
    for (int i = 0; i < 2000; i++)
      m_mahony_filter(a[0], a[1], a[2], 0.0f, 0.0f, 0.0f, 0.0f, 30.0f, 40.0f,
                      0.002f, &skewed);
    CHECK(fabsf(skewed.roll) < 2.0f && fabsf(skewed.pitch) < 2.0f,
          "a skewed mag cannot tip roll/pitch");
  }

  printf(
      "  [11] mahony: the integral term is clamped, so bias cannot wind up\n");
  {
    /* A sustained, physically impossible accel keeps the error term saturated.
     * Without the EST-MAH-105 clamp the integral runs away and the estimate
     * diverges; with it the output stays bounded. */
    attitude_t o = mahony_state();
    for (int i = 0; i < 20000; i++)
      m_mahony_filter(0.0f, G, 0.0f, 0.0f, 0.0f, 0.0f, mx, my, mz, 0.002f, &o);
    CHECK(!isnan(o.roll) && !isnan(o.pitch) && !isnan(o.yaw),
          "a sustained impossible accel stays finite");
    CHECK(fabsf(o.roll) <= 180.0f && fabsf(o.pitch) <= 90.001f,
          "and stays inside the Euler ranges");
  }

  printf("\n  %d checks, %d failures\n", g_checks, g_fails);
  return g_fails ? 1 : 0;
}
