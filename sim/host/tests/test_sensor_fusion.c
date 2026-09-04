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
      snprintf(b, sizeof b, "roll %.0f recovered (got %.2f)", poses[i][0], o.roll);
      CHECK(CLOSE(o.roll, poses[i][0], 0.05f), b);
      snprintf(b, sizeof b, "pitch %.0f recovered (got %.2f)", poses[i][1], o.pitch);
      CHECK(CLOSE(o.pitch, poses[i][1], 0.05f), b);
    }
  }

  printf("  [3] signs match the EKF: right-bank positive roll, nose-up positive pitch\n");
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

  printf("\n  %d checks, %d failures\n", g_checks, g_fails);
  return g_fails ? 1 : 0;
}
