/**
 * @file tools/sim_host/tests/test_calib_engine.c
 * @brief Host unit test for the sensor-agnostic calibration engine
 *        (src/calib/calib_engine.c) via a fake provider.
 *
 * Drives calib_engine_run() with a CALIB_FIT_ELLIPSOID target whose read_raw
 * synthesises samples from a known bias + SPD distortion across spread
 * orientations (the same model the magnetometer feeds it on hardware), and
 * checks the engine acquires, fits, and commits the recovered calibration —
 * plus the cancel and too-few-samples rejection paths (no commit, caller keeps
 * the old calibration).
 */
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "calib/calib_engine.h"

static int g_checks = 0, g_fails = 0;
#define CHECK(cond, msg)                                                        \
  do {                                                                          \
    g_checks++;                                                                 \
    if (cond)                                                                   \
      printf("    ok   %s\n", (msg));                                           \
    else {                                                                      \
      g_fails++;                                                                \
      printf("    FAIL %s   (%s:%d)\n", (msg), __FILE__, __LINE__);            \
    }                                                                           \
  } while (0)

#define RADIUS 50.0f
static const float B_TRUE[3] = {3.0f, -2.0f, 4.0f};
static const float A_TRUE[9] = {
    1.05f, 0.02f, 0.010f,
    0.02f, 0.97f, 0.015f,
    0.010f, 0.015f, 1.03f};

static void mat3_vec(const float m[9], const float v[3], float out[3]) {
  for (int i = 0; i < 3; i++)
    out[i] = m[i * 3 + 0] * v[0] + m[i * 3 + 1] * v[1] + m[i * 3 + 2] * v[2];
}
static float det3(const float m[9]) {
  return m[0] * (m[4] * m[8] - m[5] * m[7]) -
         m[1] * (m[3] * m[8] - m[5] * m[6]) +
         m[2] * (m[3] * m[7] - m[4] * m[6]);
}
static void fib_dir(int i, int n, float u[3]) {
  const float ga = 2.39996322972865332f;
  float z = 1.0f - 2.0f * ((float)i + 0.5f) / (float)n;
  float r = sqrtf(1.0f - z * z);
  float phi = (float)i * ga;
  u[0] = r * cosf(phi);
  u[1] = r * sinf(phi);
  u[2] = z;
}
static void synth(const float u[3], float m[3]) {
  float field[3] = {RADIUS * u[0], RADIUS * u[1], RADIUS * u[2]};
  mat3_vec(A_TRUE, field, m);
  for (int i = 0; i < 3; i++)
    m[i] += B_TRUE[i];
}

/* ---- fake provider state ------------------------------------------------- */
#define NDIR 80
static int s_idx;
static bool s_force_cancel;
static int s_commits;
static float s_offset[3], s_mat[9];

static bool fake_read(float v[3], void *ctx) {
  (void)ctx;
  float u[3];
  fib_dir(s_idx % NDIR, NDIR, u);
  s_idx++;
  synth(u, v);
  return true;
}
static bool fake_cancel(void *ctx) {
  (void)ctx;
  return s_force_cancel;
}
static void fake_commit(const float offset[3], const float mat[9], void *ctx) {
  (void)ctx;
  s_commits++;
  for (int i = 0; i < 3; i++)
    s_offset[i] = offset[i];
  for (int i = 0; i < 9; i++)
    s_mat[i] = mat[i];
}

/* ---- fake bias (gyro) provider ------------------------------------------- */
static const float GBIAS[3] = {0.5f, -0.3f, 0.8f};
static bool s_bias_still;     // does the board count as still this run?
static float s_bias_noise;    // +/- amplitude of per-sample noise
static unsigned int s_blcg;

static bool bias_read(float v[3], void *ctx) {
  (void)ctx;
  if (!s_bias_still)
    return false; // board moving -> no accepted samples (engine should time out)
  for (int k = 0; k < 3; k++) {
    s_blcg = s_blcg * 1103515245u + 12345u;
    float nz = s_bias_noise * ((float)((s_blcg >> 16) & 0xFFFF) / 32768.0f - 1.0f);
    v[k] = GBIAS[k] + nz;
  }
  return true;
}

static calib_target_t bias_target(void) {
  calib_target_t t = {
      .name = "gyro",
      .fit = CALIB_FIT_BIAS,
      .min_samples = 50,
      .max_ticks = 500,
      .poll_ms = 20,
      .bias_var_max = 1.0f,
      .read_raw = bias_read,
      .cancelled = fake_cancel,
      .on_progress = NULL,
      .commit = fake_commit,
      .ctx = NULL,
  };
  return t;
}

static calib_target_t base_target(void) {
  calib_target_t t = {
      .name = "fake",
      .fit = CALIB_FIT_ELLIPSOID,
      .radius = RADIUS,
      .min_samples = 50,
      .cov_done = 80.0f,
      .max_ticks = 500,
      .poll_ms = 20,
      .read_raw = fake_read,
      .cancelled = fake_cancel,
      .on_coverage = NULL,
      .commit = fake_commit,
      .ctx = NULL,
  };
  return t;
}

int main(void) {
  printf("test_calib_engine\n");

  printf("  [1] ellipsoid acquire + fit + commit\n");
  s_idx = 0;
  s_force_cancel = false;
  s_commits = 0;
  calib_target_t t = base_target();
  int rc = calib_engine_run(&t);
  CHECK(rc == 0, "engine run succeeds");
  CHECK(s_commits == 1, "commit called exactly once");
  for (int i = 0; i < 3; i++) {
    char b[48];
    snprintf(b, sizeof b, "offset[%d] (raw units) ~= bias", i);
    CHECK(fabsf(s_offset[i] - B_TRUE[i]) < 0.5f, b);
  }
  /* corrected samples land on a common sphere with the right direction */
  {
    float mag_mean = 0.0f, mags[NDIR];
    int dir_ok = 1;
    for (int i = 0; i < NDIR; i++) {
      float u[3], m[3];
      fib_dir(i, NDIR, u);
      synth(u, m);
      float d[3] = {(m[0] - s_offset[0]) / RADIUS, (m[1] - s_offset[1]) / RADIUS,
                    (m[2] - s_offset[2]) / RADIUS};
      float corr[3];
      mat3_vec(s_mat, d, corr);
      float mag = sqrtf(corr[0] * corr[0] + corr[1] * corr[1] + corr[2] * corr[2]);
      mags[i] = mag;
      mag_mean += mag;
      if ((corr[0] * u[0] + corr[1] * u[1] + corr[2] * u[2]) / mag < 0.999f)
        dir_ok = 0;
    }
    mag_mean /= (float)NDIR;
    int mag_const = 1;
    for (int i = 0; i < NDIR; i++)
      if (fabsf(mags[i] - mag_mean) > 0.01f * mag_mean)
        mag_const = 0;
    CHECK(dir_ok, "corrected direction == true direction");
    CHECK(mag_const, "corrected magnitude constant over the sphere");
    CHECK(fabsf(mag_mean - cbrtf(det3(A_TRUE))) < 0.02f, "radius == det(A)^(1/3)");
  }

  printf("  [2] cancel -> no commit\n");
  s_idx = 0;
  s_force_cancel = true;
  s_commits = 0;
  t = base_target();
  rc = calib_engine_run(&t);
  CHECK(rc == -1, "cancelled run returns failure");
  CHECK(s_commits == 0, "no commit on cancel");

  printf("  [3] too few samples -> no commit\n");
  s_idx = 0;
  s_force_cancel = false;
  s_commits = 0;
  t = base_target();
  t.min_samples = 1000; /* unreachable within max_ticks (only ~20 samples) */
  t.max_ticks = 20;
  t.cov_done = 200.0f;    /* never satisfied, so it runs the full (tiny) cap */
  rc = calib_engine_run(&t);
  CHECK(rc == -1, "under-sampled run returns failure");
  CHECK(s_commits == 0, "no commit when too few samples");

  printf("  [4] fit_points + normalize_radius -> |corrected| == radius\n");
  {
    float pts[NDIR][3];
    for (int i = 0; i < NDIR; i++) {
      float u[3];
      fib_dir(i, NDIR, u);
      synth(u, pts[i]);
    }
    t = base_target();
    t.normalize_radius = true;
    s_commits = 0;
    rc = calib_engine_fit_points(&t, (const float (*)[3])pts, NDIR);
    CHECK(rc == 0, "fit_points succeeds");
    CHECK(s_commits == 1, "commit called once");
    /* Apply exactly as the firmware does: a_cal = M * (raw - offset), in raw
     * units (no /radius). normalize_radius makes |a_cal| == radius. */
    float mag_mean = 0.0f, mags[NDIR];
    int dir_ok = 1;
    for (int i = 0; i < NDIR; i++) {
      float u[3];
      fib_dir(i, NDIR, u);
      float d[3] = {pts[i][0] - s_offset[0], pts[i][1] - s_offset[1],
                    pts[i][2] - s_offset[2]};
      float corr[3];
      mat3_vec(s_mat, d, corr);
      float mag = sqrtf(corr[0] * corr[0] + corr[1] * corr[1] + corr[2] * corr[2]);
      mags[i] = mag;
      mag_mean += mag;
      if ((corr[0] * u[0] + corr[1] * u[1] + corr[2] * u[2]) / mag < 0.999f)
        dir_ok = 0;
    }
    mag_mean /= (float)NDIR;
    CHECK(dir_ok, "corrected direction == true direction");
    /* the whole point of normalize_radius: absolute magnitude == radius (g),
     * NOT the bare fit's det(A)^(1/3) */
    CHECK(fabsf(mag_mean - RADIUS) < 0.05f, "corrected magnitude == radius (g)");
    int mag_const = 1;
    for (int i = 0; i < NDIR; i++)
      if (fabsf(mags[i] - RADIUS) > 0.02f * RADIUS)
        mag_const = 0;
    CHECK(mag_const, "corrected magnitude constant == radius");
  }

  printf("  [5] bias (gyro) recovery\n");
  s_force_cancel = false;
  s_bias_still = true;
  s_bias_noise = 0.01f;
  s_blcg = 777;
  s_commits = 0;
  t = bias_target();
  rc = calib_engine_run(&t);
  CHECK(rc == 0, "bias run succeeds");
  CHECK(s_commits == 1, "commit called once");
  for (int i = 0; i < 3; i++) {
    char b[40];
    snprintf(b, sizeof b, "gyro bias[%d] recovered", i);
    CHECK(fabsf(s_offset[i] - GBIAS[i]) < 0.05f, b);
  }

  printf("  [6] never still -> timeout, no commit\n");
  s_bias_still = false;
  s_commits = 0;
  t = bias_target();
  rc = calib_engine_run(&t);
  CHECK(rc == -1, "moving board times out");
  CHECK(s_commits == 0, "no commit when never still");

  printf("  [7] too noisy -> variance reject, no commit\n");
  s_bias_still = true;
  s_bias_noise = 3.0f; // var ~ 3 dps^2 > bias_var_max (1.0)
  s_blcg = 4242;
  s_commits = 0;
  t = bias_target();
  rc = calib_engine_run(&t);
  CHECK(rc == -1, "noisy window rejected");
  CHECK(s_commits == 0, "no commit when variance too high");

  printf("\n  %d checks, %d failures\n", g_checks, g_fails);
  return g_fails ? 1 : 0;
}
