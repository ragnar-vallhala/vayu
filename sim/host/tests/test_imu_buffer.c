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
/* IMU/attitude SPSC rings (firmware/src/hub/hub.c).
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
 * (on-target gcov) per tools/coverage_gate.py; this ring is the part of the
 * sensor path that CAN be exercised honestly on the host. */
#include <stdio.h>
#include <string.h>

#include "hub/hub.h"
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
static imu_sample_t sample(float tag) {
  imu_sample_t s;
  memset(&s, 0, sizeof s);
  s.gyr[0] = tag;
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

  imu_sample_t out;
  attitude_t aout;

  printf("  [1] every ring starts empty\n");
  {
    CHECK(!imu_queue_control_pop(&out), "control empty");
    CHECK(!imu_queue_attitude_pop(&out), "attitude empty");
    CHECK(!imu_queue_telemetry_pop(&out), "telemetry empty");
    CHECK(!attitude_queue_control_pop(&aout), "attitude-control empty");
    CHECK(!attitude_queue_telemetry_pop(&aout), "attitude-telemetry empty");
  }

  printf("  [2] a sample survives a round trip on each ring\n");
  {
    imu_sample_t in = sample(1.0f);
    CHECK(imu_queue_control_push(&in), "control push accepted");
    CHECK(imu_queue_control_pop(&out) && out.gyr[0] == 1.0f,
          "control round trip");

    in = sample(2.0f);
    CHECK(imu_queue_attitude_push(&in), "attitude push accepted");
    CHECK(imu_queue_attitude_pop(&out) && out.gyr[0] == 2.0f,
          "attitude round trip");

    in = sample(3.0f);
    CHECK(imu_queue_telemetry_push(&in), "telemetry push accepted");
    CHECK(imu_queue_telemetry_pop(&out) && out.gyr[0] == 3.0f,
          "telemetry round trip");

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

  printf(
      "  [3] the latest-value read gives the NEWEST, and leaves the stream\n");
  {
    /* There is no head-peek any more, on purpose. Peeking a single-consumer queue
     * handed out the OLDEST unread item, so a reader that wanted "the current
     * value" got one as stale as the real consumer was behind -- which is exactly
     * what angle_controller was doing to vertical.state, ~3 publishes behind at
     * all times. The replacement is a latest-value read, and these checks pin the
     * difference that bug turned on. */
    attitude_t a1 = att(46.0f), a2 = att(47.0f), aout;

    CHECK(!attitude_telemetry_latest(&aout) || true,
          "latest before any push must not crash");

    CHECK(attitude_queue_telemetry_push(&a1), "push 46");
    CHECK(attitude_telemetry_latest(&aout) && aout.roll == 46.0f,
          "latest sees it");
    CHECK(attitude_telemetry_latest(&aout) && aout.roll == 46.0f,
          "latest consumes nothing");

    CHECK(attitude_queue_telemetry_push(&a2), "push 47");
    CHECK(attitude_telemetry_latest(&aout) && aout.roll == 47.0f,
          "latest is the NEWEST, not the oldest unread -- the staleness bug");

    /* And the stream the real consumer reads is untouched and still in order. */
    CHECK(attitude_queue_telemetry_pop(&aout) && aout.roll == 46.0f,
          "stream still FIFO: 46 first");
    CHECK(attitude_queue_telemetry_pop(&aout) && aout.roll == 47.0f, "then 47");
    CHECK(!attitude_queue_telemetry_pop(&aout), "stream drained");

    /* The latest value survives draining: it is a current value, not a queue. */
    CHECK(attitude_telemetry_latest(&aout) && aout.roll == 47.0f,
          "latest outlives the drained stream");
  }

  printf("  [4] overrun costs the OLDEST sample, never the newest\n");
  {
    const int n = 3 * IMU_BUFFER_SIZE;
    for (int i = 0; i < n; i++) {
      imu_sample_t in = sample((float)(200 + i));
      CHECK(imu_queue_control_push(&in), "push past capacity still accepted");
    }
    int count = 0;
    float last = 0.0f;
    while (imu_queue_control_pop(&out)) {
      CHECK(out.gyr[0] != 200.0f, "the stalest sample was overwritten");
      last = out.gyr[0];
      count++;
    }
    /* A bus pipe holds exactly its slot count, so the depth is now exact. It used
     * to be SIZE-1 or SIZE because spsc_init could lose a slot to element
     * alignment -- a quirk of the ring, not a property anything depended on. */
    CHECK(count == IMU_BUFFER_SIZE + 1, "pipe depth is exactly its slot count");
    CHECK(last == (float)(200 + n - 1), "the freshest sample survived");
  }

  printf("  [5] a give with no waiter is not lost\n");
  {
    /* The rate loop is not running here, so the push below signals nothing.
     * The wait afterwards must still return immediately: otherwise the first
     * sample after a stall goes unnoticed and the loop waits a full timeout. */
    imu_sample_t in = sample(99.0f);
    CHECK(imu_queue_control_push(&in), "push signals the control semaphore");
    CHECK(imu_queue_control_wait(0),
          "wait after push returns without blocking");
    CHECK(imu_queue_control_pop(&out) && out.gyr[0] == 99.0f,
          "the sample is there");

    in = sample(98.0f);
    CHECK(imu_queue_attitude_push(&in), "push signals the attitude semaphore");
    CHECK(imu_queue_attitude_wait(0), "attitude wait returns without blocking");
    CHECK(imu_queue_attitude_pop(&out) && out.gyr[0] == 98.0f,
          "the sample is there");

    attitude_t ain = att(97.0f);
    CHECK(attitude_queue_control_push(&ain),
          "push signals the outer-loop semaphore");
    CHECK(attitude_queue_control_wait(0),
          "outer-loop wait returns without blocking");
    CHECK(attitude_queue_control_pop(&aout) && aout.roll == 97.0f,
          "the attitude is there");
  }

  printf("  [6] board trim defaults to level and round-trips\n");
  {
    /* The estimator subtracts this from its published euler. It must default
     * to 0,0 -- a garbage trim on an uncalibrated board would tilt every
     * attitude report and the angle loop would fly the craft to that lie. */
    float tr = 1.0f, tp = 1.0f;
    hub_get_board_trim(&tr, &tp);
    CHECK(tr == 0.0f && tp == 0.0f, "trim starts level");

    hub_set_board_trim(-2.5f, 0.75f);
    hub_get_board_trim(&tr, &tp);
    CHECK(tr == -2.5f && tp == 0.75f, "trim round trip");

    /* NULL-tolerant: the driver publishes both, but a caller may want one. */
    hub_get_board_trim(&tr, NULL);
    CHECK(tr == -2.5f, "a NULL out-param is ignored, not written");

    hub_set_board_trim(0.0f, 0.0f);
  }

  printf("  [7] perf rows are reported, and respect the caller's cap\n");
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
