/**
 * @file sim/host/tests/test_calib_ellipsoid.c
 * @brief Host unit test for the shared ellipsoid fit (src/calib/calib_ellipsoid.c).
 *
 * Synthesises static sensor samples from a KNOWN ground truth — a bias `b` plus a
 * symmetric positive-definite distortion `A` (scale + cross-axis misalignment) —
 * over orientations spread across the sphere, then checks that calib_fit_ellipsoid
 * recovers it:
 *   measured m = A·(radius·û) + b           (raw, what the sensor reports)
 *   fit:        offset (scaled) + soft (3x3)
 *   contract:   soft·(m/radius − offset) lands every sample on a common sphere,
 *               with offset·radius == b and the corrected direction == û.
 *
 * For an SPD A the fit returns soft = det(A)^(1/3)·A^-1, so the corrected vectors
 * equal det(A)^(1/3)·û: constant magnitude (sphere) and exact direction. The
 * absolute-magnitude renormalisation to g is the accelerometer routine's job
 * (Phase 3), not the solver's — this test validates the extracted math only.
 *
 * Also checks the rejection path: coplanar (rank-deficient) input must return -1
 * so the caller keeps the previous calibration.
 */
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "calib/calib_ellipsoid.h"

static int g_checks = 0, g_fails = 0;
#define CHECK(cond, msg)                                                       \
  do {                                                                         \
    g_checks++;                                                                \
    if (cond)                                                                  \
      printf("    ok   %s\n", (msg));                                          \
    else {                                                                     \
      g_fails++;                                                               \
      printf("    FAIL %s   (%s:%d)\n", (msg), __FILE__, __LINE__);            \
    }                                                                          \
  } while (0)

/* Ground truth. A is symmetric => SPD (diagonally dominant, positive diag). */
static const float RADIUS = 9.80665f;
static const float B_TRUE[3] = {0.40f, -0.30f, 0.50f};
static const float A_TRUE[9] = {1.05f,  0.02f,  0.010f, 0.02f, 0.97f,
                                0.015f, 0.010f, 0.015f, 1.03f};

static void mat3_vec(const float m[9], const float v[3], float out[3]) {
  for (int i = 0; i < 3; i++)
    out[i] = m[i * 3 + 0] * v[0] + m[i * 3 + 1] * v[1] + m[i * 3 + 2] * v[2];
}
static float det3(const float m[9]) {
  return m[0] * (m[4] * m[8] - m[5] * m[7]) -
         m[1] * (m[3] * m[8] - m[5] * m[6]) +
         m[2] * (m[3] * m[7] - m[4] * m[6]);
}

/* i-th of N well-spread unit vectors on the sphere (deterministic Fibonacci). */
static void fib_dir(int i, int n, float u[3]) {
  const float ga = 2.39996322972865332f; // golden angle
  float z = 1.0f - 2.0f * ((float)i + 0.5f) / (float)n;
  float r = sqrtf(1.0f - z * z);
  float phi = (float)i * ga;
  u[0] = r * cosf(phi);
  u[1] = r * sinf(phi);
  u[2] = z;
}

/* Accumulate one measured sample into the normal equations (mirrors bmx160). */
static void accumulate(float S[81], float t[9], const float m[3],
                       float radius) {
  float x = m[0] / radius, y = m[1] / radius, z = m[2] / radius;
  float r[9] = {x * x,     y * y, z * z, 2 * y * z, 2 * x * z,
                2 * x * y, 2 * x, 2 * y, 2 * z};
  for (int a = 0; a < 9; a++) {
    t[a] += r[a];
    for (int b = 0; b < 9; b++)
      S[a * 9 + b] += r[a] * r[b];
  }
}

/* Build a measured sample for unit direction u with optional bias-free noise. */
static void synth(const float u[3], float noise, float m[3]) {
  float field[3] = {RADIUS * u[0], RADIUS * u[1], RADIUS * u[2]};
  mat3_vec(A_TRUE, field, m);
  for (int i = 0; i < 3; i++)
    m[i] += B_TRUE[i] + noise;
}

/* Run the full recover-and-verify cycle over N spread orientations. Returns the
 * fit result; on success, fills offset/soft. */
static int recover(int n, float noise_amp, float offset[3], float soft[9]) {
  float S[81] = {0}, t[9] = {0};
  /* tiny deterministic LCG noise so a "noisy" case is reproducible */
  unsigned int lcg = 12345u;
  for (int i = 0; i < n; i++) {
    float u[3];
    fib_dir(i, n, u);
    float nz = 0.0f;
    if (noise_amp > 0.0f) {
      lcg = lcg * 1103515245u + 12345u;
      nz = noise_amp * ((float)((lcg >> 16) & 0xFFFF) / 32768.0f - 1.0f);
    }
    float m[3];
    synth(u, nz, m);
    accumulate(S, t, m, RADIUS);
  }
  return calib_fit_ellipsoid(S, t, offset, soft);
}

/* Verify a successful fit against ground truth. */
static void verify(int n, const float offset[3], const float soft[9],
                   float off_tol, float cos_tol, float mag_tol) {
  /* offset (scaled) * radius == bias */
  for (int i = 0; i < 3; i++) {
    char b[48];
    snprintf(b, sizeof b, "offset[%d]*radius ~= bias", i);
    CHECK(fabsf(offset[i] * RADIUS - B_TRUE[i]) < off_tol, b);
  }
  /* every corrected sample: same magnitude (sphere) + direction == û */
  float mag_mean = 0.0f;
  float mags[256];
  int worst_dir_ok = 1;
  for (int i = 0; i < n; i++) {
    float u[3], m[3];
    fib_dir(i, n, u);
    synth(u, 0.0f, m);
    float xc[3] = {m[0] / RADIUS - offset[0], m[1] / RADIUS - offset[1],
                   m[2] / RADIUS - offset[2]};
    float corr[3];
    mat3_vec(soft, xc, corr);
    float mag =
        sqrtf(corr[0] * corr[0] + corr[1] * corr[1] + corr[2] * corr[2]);
    mags[i] = mag;
    mag_mean += mag;
    float cosang = (corr[0] * u[0] + corr[1] * u[1] + corr[2] * u[2]) / mag;
    if (cosang < cos_tol)
      worst_dir_ok = 0;
  }
  mag_mean /= (float)n;
  CHECK(worst_dir_ok, "corrected direction == true direction (all samples)");
  /* corrected magnitude is constant across the sphere (within mag_tol of mean) */
  int mag_const = 1;
  for (int i = 0; i < n; i++)
    if (fabsf(mags[i] - mag_mean) > mag_tol * mag_mean)
      mag_const = 0;
  CHECK(mag_const, "corrected magnitude constant over the sphere");
  /* and equals the predicted det(A)^(1/3) (the fit's geometric-mean radius) */
  float predicted = cbrtf(det3(A_TRUE));
  CHECK(fabsf(mag_mean - predicted) < 0.02f, "common radius == det(A)^(1/3)");
}

int main(void) {
  printf("test_calib_ellipsoid\n");

  printf("  [1] noiseless recovery (60 spread orientations)\n");
  float off[3], soft[9];
  int rc = recover(60, 0.0f, off, soft);
  CHECK(rc == 0, "fit succeeds");
  if (rc == 0)
    verify(60, off, soft, 0.05f, 0.999f, 0.01f);

  printf("  [2] noisy recovery (~0.03 m/s^2, 120 orientations)\n");
  rc = recover(120, 0.03f, off, soft);
  CHECK(rc == 0, "fit succeeds under noise");
  if (rc == 0)
    verify(120, off, soft, 0.15f, 0.995f, 0.03f);

  printf("  [3] degenerate (coplanar) input is rejected\n");
  {
    float S[81] = {0}, t[9] = {0};
    for (int i = 0; i < 48; i++) {
      float th = 6.2831853f * (float)i / 48.0f;
      float u[3] = {cosf(th), sinf(th), 0.0f}; /* great circle, z==0 */
      float m[3];
      synth(u, 0.0f, m);
      accumulate(S, t, m, RADIUS);
    }
    float o[3], s[9];
    CHECK(calib_fit_ellipsoid(S, t, o, s) != 0, "coplanar fit returns failure");
  }

  printf("\n  %d checks, %d failures\n", g_checks, g_fails);
  return g_fails ? 1 : 0;
}
