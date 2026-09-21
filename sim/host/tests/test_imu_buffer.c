/*
 * Copyright (C) 2026 NAVRobotec Pvt Ltd
 * Author: Ragnar Vallhala
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/* IMU/attitude SPSC rings (firmware/src/sensor/imu_buffer.c).
 *
 * These rings sit between the IMU driver and every consumer of it -- the rate
 * loop, the attitude estimator, the telemetry gate, the calibration engine.
 * They are all OVERWRITE, which is the property worth pinning: a late consumer
 * must cost the OLDEST sample, never the newest, because the control loop
 * wants the freshest gyro reading and nothing else.
 *
 * The push/wait pairs also carry a notification semaphore (CTRL-RATE-101): the
 * rate loop blocks on it instead of clock-polling. A give with no waiter must
 * not be lost, or the first sample after a stall is never noticed -- so the
 * wait-after-push case is checked directly.
 *
 * Driver code (bmx160, bme280, vl53l0x) needs real hardware and is tier-2
 * (on-target gcov) per tools/coverage_gate.py; this ring is the part of
 * firmware/src/sensor that CAN be exercised honestly on the host. */
#include <stdio.h>
#include <string.h>

#include "sensor/imu_buffer.h"
#include "comm/perf_telemetry.h"

/* Usable depth is NOT fixed. spsc_init aligns the buffer to a multiple of
 * elem_size and spends a slot doing it ONLY when the static array's address is
 * not already aligned (extern/vaios/kernel/structure.c:26-44) -- and none of
 * these element types is a power of two in size, so which case you get depends
 * on where the linker happened to put the array. Locally these rings hold
 * SIZE-1; on the CI runner rc_buffer's held SIZE. Both are correct and the
 * difference is harmless for an OVERWRITE mailbox, but the depth cannot be
 * asserted exactly -- so assert the band, and assert the property that
 * actually matters (below) exactly. */

static int g_checks = 0, g_fails = 0;
#define CHECK(cond, msg)                                                       \
  do {                                                                         \
    g_checks++;                                                                \
    if (!(cond)) {                                                             \
      g_fails++;                                                               \
      printf("    FAIL: %s\n", (msg));                                         \
    }                                                                          \
  } while (0)

/* gyro.x carries the sample's identity so overwrite order is checkable. */
static bmx160_all_reading_t sample(float tag) {
  bmx160_all_reading_t s;
  memset(&s, 0, sizeof s);
  s.converted.gyr_raw[0] = tag;
  return s;
}

static imu_calibration_telemetry_t cal_frame(uint8_t tag) {
  imu_calibration_telemetry_t c;
  memset(&c, 0, sizeof c);
  c.buffer[0] = tag;
  c.size = 1;
  return c;
}

static attitude_t att(float tag) {
  attitude_t a;
  memset(&a, 0, sizeof a);
  a.roll = tag;
  return a;
}

int main(void) {
  printf("== imu_buffer ==\n");
  imu_buffer_init();

  bmx160_all_reading_t out;
  attitude_t aout;

  printf("  [1] every ring starts empty\n");
  {
    CHECK(!imu_queue_control_pop(&out), "control empty");
    CHECK(!imu_queue_attitude_pop(&out), "attitude empty");
    CHECK(!imu_queue_telemetry_pop(&out), "telemetry empty");
    CHECK(!imu_queue_calibration_pop(&out), "calibration empty");
    CHECK(!attitude_queue_control_pop(&aout), "attitude-control empty");
    CHECK(!attitude_queue_telemetry_pop(&aout), "attitude-telemetry empty");
  }

  printf("  [2] a sample survives a round trip on each ring\n");
  {
    bmx160_all_reading_t in = sample(1.0f);
    CHECK(imu_queue_control_push(&in), "control push accepted");
    CHECK(imu_queue_control_pop(&out) && out.converted.gyr_raw[0] == 1.0f,
          "control round trip");

    in = sample(2.0f);
    CHECK(imu_queue_attitude_push(&in), "attitude push accepted");
    CHECK(imu_queue_attitude_pop(&out) && out.converted.gyr_raw[0] == 2.0f,
          "attitude round trip");

    in = sample(3.0f);
    CHECK(imu_queue_telemetry_push(&in), "telemetry push accepted");
    CHECK(imu_queue_telemetry_pop(&out) && out.converted.gyr_raw[0] == 3.0f,
          "telemetry round trip");

    in = sample(4.0f);
    CHECK(imu_queue_calibration_push(&in), "calibration push accepted");
    CHECK(imu_queue_calibration_pop(&out) && out.converted.gyr_raw[0] == 4.0f,
          "calibration round trip");

    imu_calibration_telemetry_t cin = cal_frame(5), cout;
    CHECK(imu_queue_calibration_telemetry_push(&cin),
          "calib-telemetry push accepted");
    CHECK(imu_queue_calibration_telemetry_pop(&cout) && cout.buffer[0] == 5,
          "calib-telemetry round trip");

    attitude_t ain = att(6.0f);
    CHECK(attitude_queue_control_push(&ain), "attitude-control push accepted");
    CHECK(attitude_queue_control_pop(&aout) && aout.roll == 6.0f,
          "attitude-control round trip");

    ain = att(7.0f);
    CHECK(attitude_queue_telemetry_push(&ain),
          "attitude-telemetry push accepted");
    CHECK(attitude_queue_telemetry_pop(&aout) && aout.roll == 7.0f,
          "attitude-telemetry round trip");
  }

  printf("  [3] peek returns the head without consuming it\n");
  {
    bmx160_all_reading_t in = sample(42.0f);
    CHECK(imu_queue_control_push(&in), "push for peek");
    CHECK(imu_queue_control_peek(&out) && out.converted.gyr_raw[0] == 42.0f,
          "peek sees the head");
    CHECK(imu_queue_control_peek(&out) && out.converted.gyr_raw[0] == 42.0f,
          "peek did not consume");
    CHECK(imu_queue_control_pop(&out) && out.converted.gyr_raw[0] == 42.0f,
          "pop still gets it");
    CHECK(!imu_queue_control_peek(&out), "peek on an empty ring fails");

    in = sample(43.0f);
    CHECK(imu_queue_telemetry_push(&in), "push for telemetry peek");
    CHECK(imu_queue_telemetry_peek(&out) && out.converted.gyr_raw[0] == 43.0f,
          "telemetry peek");
    CHECK(imu_queue_telemetry_pop(&out), "telemetry drain");

    in = sample(44.0f);
    CHECK(imu_queue_calibration_push(&in), "push for calibration peek");
    CHECK(imu_queue_calibration_peek(&out) && out.converted.gyr_raw[0] == 44.0f,
          "calibration peek");
    CHECK(imu_queue_calibration_pop(&out), "calibration drain");

    imu_calibration_telemetry_t cin = cal_frame(45), cout;
    CHECK(imu_queue_calibration_telemetry_push(&cin),
          "push for calib-tel peek");
    CHECK(imu_queue_calibration_telemetry_peek(&cout) && cout.buffer[0] == 45,
          "calib-telemetry peek");
    CHECK(imu_queue_calibration_telemetry_pop(&cout), "calib-telemetry drain");

    attitude_t ain = att(46.0f);
    CHECK(attitude_queue_telemetry_push(&ain), "push for attitude peek");
    CHECK(attitude_queue_telemetry_peek(&aout) && aout.roll == 46.0f,
          "attitude peek");
    CHECK(attitude_queue_telemetry_pop(&aout), "attitude drain");
  }

  printf("  [4] overrun costs the OLDEST sample, never the newest\n");
  {
    const int n = 3 * IMU_BUFFER_SIZE;
    for (int i = 0; i < n; i++) {
      bmx160_all_reading_t in = sample((float)(200 + i));
      CHECK(imu_queue_control_push(&in), "push past capacity still accepted");
    }
    int count = 0;
    float last = 0.0f;
    while (imu_queue_control_pop(&out)) {
      CHECK(out.converted.gyr_raw[0] != 200.0f,
            "the stalest sample was overwritten");
      last = out.converted.gyr_raw[0];
      count++;
    }
    CHECK(count >= IMU_BUFFER_SIZE - 1 && count <= IMU_BUFFER_SIZE,
          "ring depth is SIZE-1 or SIZE, per spsc_init's alignment");
    CHECK(last == (float)(200 + n - 1), "the freshest sample survived");
  }

  printf("  [5] a give with no waiter is not lost\n");
  {
    /* The rate loop is not running here, so the push below signals nothing.
     * The wait afterwards must still return immediately: otherwise the first
     * sample after a stall goes unnoticed and the loop waits a full timeout. */
    bmx160_all_reading_t in = sample(99.0f);
    CHECK(imu_queue_control_push(&in), "push signals the control semaphore");
    CHECK(imu_queue_control_wait(0),
          "wait after push returns without blocking");
    CHECK(imu_queue_control_pop(&out) && out.converted.gyr_raw[0] == 99.0f,
          "the sample is there");

    in = sample(98.0f);
    CHECK(imu_queue_attitude_push(&in), "push signals the attitude semaphore");
    CHECK(imu_queue_attitude_wait(0), "attitude wait returns without blocking");
    CHECK(imu_queue_attitude_pop(&out) && out.converted.gyr_raw[0] == 98.0f,
          "the sample is there");

    attitude_t ain = att(97.0f);
    CHECK(attitude_queue_control_push(&ain),
          "push signals the outer-loop semaphore");
    CHECK(attitude_queue_control_wait(0),
          "outer-loop wait returns without blocking");
    CHECK(attitude_queue_control_pop(&aout) && aout.roll == 97.0f,
          "the attitude is there");
  }

  printf("  [6] perf rows are reported, and respect the caller's cap\n");
  {
    perf_fifo_row_t rows[16];
    memset(rows, 0, sizeof rows);
    int n = imu_buffer_perf_fifos(rows, 16);
    CHECK(n > 0, "at least one fifo reported");
    CHECK(imu_buffer_perf_fifos(rows, 1) == 1, "a cap of 1 yields 1 row");
    CHECK(imu_buffer_perf_fifos(rows, 0) == 0, "a cap of 0 yields no rows");
  }

  printf("\n  %d checks, %d failures\n", g_checks, g_fails);
  return g_fails ? 1 : 0;
}
