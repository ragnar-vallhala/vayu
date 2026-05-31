/**
 * @file tools/sim_host/tests/test_phase3_slog.c
 * @brief SITL verification suite for the Phase-3 SLOG cluster.
 *
 * Links the real firmware sources (libvayu_sitl_core) and drives their
 * public contracts directly.
 *
 *   @verifies SNS-BUF-002  IMU buffer drop accounting
 *
 * LOG-TXT-002 (telemetry drains vayu_log_queue) is verified by inspection
 * of telemetry_task.c. LOG-SD-002 (SD log wrap counter) is a bench
 * requirement — the 10 MB log files make a wrap impractical to drive in
 * SITL; the counter wiring is verified by inspection of logger.c.
 *
 * Built only under VAYU_SIM (the harness defines it for every TU).
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "sensor/sensor.h"

static int g_checks = 0;
static int g_fails  = 0;

#define CHECK(cond, msg)                                                       \
  do {                                                                         \
    g_checks++;                                                                \
    if (cond) {                                                                \
      printf("    ok   %s\n", (msg));                                          \
    } else {                                                                   \
      g_fails++;                                                               \
      printf("    FAIL %s   (%s:%d)\n", (msg), __FILE__, __LINE__);            \
    }                                                                          \
  } while (0)

/* ----------------------------------------------------------------------------
 * SNS-BUF-002 — overwrites are counted, not silent
 * --------------------------------------------------------------------------*/
static void test_imu_drop_counter(void) {
  printf("  test_imu_drop_counter (SNS-BUF-002)\n");

  imu_buffer_init();

  bmx160_all_reading_t sample;
  memset(&sample, 0, sizeof sample);

  uint32_t start = imu_buffer_drop_count();
  CHECK(start == 0, "drop count starts at zero after init");

  /* A couple of pushes are safely below the ring's usable depth, so with
   * no reader they cannot overwrite anything. */
  imu_buffer_push(&sample);
  imu_buffer_push(&sample);
  CHECK(imu_buffer_drop_count() == start, "no drops below capacity");

  /* Saturate well past capacity with no consumer: overwrites must be
   * counted (exact count depends on ring depth, so just require > 0). */
  for (int i = 0; i < 200; i++) {
    imu_buffer_push(&sample);
  }
  uint32_t saturated = imu_buffer_drop_count();
  CHECK(saturated > start, "overwrites past full are counted");

  /* At steady-state-full, each further push overwrites exactly one
   * unread sample → the counter advances one-for-one. */
  const int more = 10;
  for (int i = 0; i < more; i++) {
    imu_buffer_push(&sample);
  }
  CHECK(imu_buffer_drop_count() == saturated + (uint32_t)more,
        "one drop per push once the ring is full");
}

int main(void) {
  printf("== Phase-3 SLOG SITL verification ==\n");

  test_imu_drop_counter();

  printf("\n%d checks, %d failures\n", g_checks, g_fails);
  return g_fails == 0 ? 0 : 1;
}
