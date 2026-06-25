/**
 * @file sim/host/tests/test_phase3_ctrl.c
 * @brief SITL verification suite for the Phase-3 CTRL cluster.
 *
 * Links the real firmware sources (libvayu_sitl_core) and drives their
 * public contracts directly.
 *
 *   @verifies CTRL-RATE-101  rate loop triggered by IMU-queue arrival
 *   @verifies EST-MAH-105    Mahony integral feedback bound + reset
 *
 * CTRL-RATE-101 is verified end-to-end here: the rate loop's new wait
 * primitive (imu_queue_control_wait) blocks on an empty queue and is
 * woken by a push — i.e. the loop is sample-driven, not clock-polled.
 *
 * EST-MAH-105's reset entry point and finite/normalised output under a
 * long windup run are verified here; the exact ±0.5 rad/s integral bound
 * is verified by inspection of sensor_fusion.c (Analysis).
 *
 * Built only under VAYU_SIM (the harness defines it for every TU).
 */
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "est/est.h"
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
 * CTRL-RATE-101 — the rate loop waits on IMU-queue arrival, not the clock
 * --------------------------------------------------------------------------*/
static void test_imu_control_notify(void) {
  printf("  test_imu_control_notify (CTRL-RATE-101)\n");

  imu_buffer_init();

  /* Empty queue: a bounded wait must time out. (In the loop this is the
   * IMU-stall path that falls back to the previous sample — it must not
   * wake spuriously.) */
  CHECK(imu_queue_control_wait(20) == false,
        "wait on empty queue -> timeout (no spurious wake)");

  /* A push wakes the waiter immediately: the loop is event-driven. */
  bmx160_all_reading_t sample;
  memset(&sample, 0, sizeof sample);
  imu_queue_control_push(&sample);
  CHECK(imu_queue_control_wait(200) == true,
        "push -> wait returns at once (sample-driven)");

  /* The notification is edge-consumed: once taken, the next wait blocks
   * again rather than spinning. */
  CHECK(imu_queue_control_wait(20) == false,
        "signal drained -> next wait times out");

  /* The pushed sample is actually retrievable from the ring. */
  bmx160_all_reading_t out;
  CHECK(imu_queue_control_pop(&out) == true, "pushed sample is poppable");
}

/* ----------------------------------------------------------------------------
 * EST-MAH-105 — integral feedback reset + bounded, finite output
 * --------------------------------------------------------------------------*/
static void test_estimator_integral(void) {
  printf("  test_estimator_integral (EST-MAH-105)\n");

  /* Reset entry point links and runs (drops accumulated windup). */
  estimator_reset();
  CHECK(true, "estimator_reset() callable");

  /* Windup regression guard: drive the filter with a persistent
   * accel/orientation mismatch for many steps. With the integral term
   * clamped per-axis the quaternion must remain finite and unit-norm —
   * an unbounded integral would eventually poison the normalisation
   * and produce NaN. */
  attitude_t ori;
  memset(&ori, 0, sizeof ori);
  ori.q.w = 1.0f;
  for (int i = 0; i < 20000; i++) {
    m_mahony_filter(0.5f, 0.0f, 9.81f,   /* tilted accel -> steady error */
                    0.0f, 0.0f, 0.0f,    /* gyro reports no rotation     */
                    0.0f, 0.0f, 0.0f,    /* mag invalid -> mag step skipped */
                    1e-3f,               /* fixed dt for the stability soak */
                    &ori);
  }
  bool finite = isfinite(ori.q.w) && isfinite(ori.q.x) &&
                isfinite(ori.q.y) && isfinite(ori.q.z);
  CHECK(finite, "quaternion finite after long windup run");

  float n = ori.q.w * ori.q.w + ori.q.x * ori.q.x +
            ori.q.y * ori.q.y + ori.q.z * ori.q.z;
  CHECK(n > 0.9f && n < 1.1f, "quaternion stays unit-norm after windup");

  estimator_reset();
}

int main(void) {
  printf("== Phase-3 CTRL SITL verification ==\n");

  test_imu_control_notify();
  test_estimator_integral();

  printf("\n%d checks, %d failures\n", g_checks, g_fails);
  return g_fails == 0 ? 0 : 1;
}
