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
/* Rate-loop system-ID capture (firmware/src/control/sysid.c).
 *
 * The capture buffer is heap, claimed on the first sysid_start of a boot, so
 * the properties worth pinning are the ones that allocation introduced:
 *
 *   - sysid_start REPORTS whether the run began. It returns 0 for a rejected
 *     request and for a failed allocation, and the command handler turns that
 *     into ACK_BAD. A chirp that moves the motors with nowhere to put the
 *     response is all of the risk and none of the data.
 *   - sysid_capture is called from the 1 kHz control loop on EVERY tick,
 *     including before any run has ever allocated the buffer. That path must
 *     not dereference the null pointer.
 *   - what goes in comes back out, in order, through the dump cursor.
 */
#include <stdio.h>

#include "control/sysid.h"

static int g_checks = 0, g_fails = 0;
#define CHECK(cond, msg)                                                       \
  do {                                                                         \
    g_checks++;                                                                \
    if (!(cond)) {                                                             \
      g_fails++;                                                               \
      printf("    FAIL: %s\n", (msg));                                         \
    }                                                                          \
  } while (0)

/* SYSID_DECIM is 2 (file-private), so two capture calls yield one sample. */
#define CAPTURE_PER_SAMPLE 2

static sysid_request_t req(uint8_t axis) {
  sysid_request_t r = {.axis = axis,
                       .mode = SYSID_INJECT_RATE_SP,
                       .f0_hz = 1.0f,
                       .f1_hz = 10.0f,
                       .amp_dps = 20.0f,
                       .duration_s = 2.0f};
  return r;
}

int main(void) {
  printf("== sysid ==\n");

  printf(
      "  [1] capture before any run does not touch the unallocated buffer\n");
  {
    /* The control loop calls this every tick from boot. Nothing has allocated
     * the buffer yet -- if this dereferences it, the rate loop hard-faults. */
    const float u[3] = {0.1f, 0.2f, 0.3f};
    const float g[3] = {1.0f, 2.0f, 3.0f};
    for (int i = 0; i < 8; i++) {
      sysid_capture(u, g);
    }
    CHECK(sysid_capture_count() == 0, "nothing captured while idle");
    CHECK(!sysid_active(), "not active before a start");
  }

  printf("  [2] a bad request is refused and starts nothing\n");
  {
    sysid_request_t bad = req(3); /* axis > 2 */
    CHECK(sysid_start(&bad) == 0, "out-of-range axis rejected");
    CHECK(!sysid_active(), "rejected request left the run inactive");
    CHECK(sysid_start(0) == 0, "null request rejected");
    CHECK(!sysid_active(), "null request left the run inactive");
  }

  printf("  [3] a good request starts, and reports that it started\n");
  {
    sysid_request_t r = req(1); /* pitch */
    CHECK(sysid_start(&r) != 0, "start reports success");
    CHECK(sysid_active(), "run is active");
    CHECK(sysid_capture_axis() == 1, "excited axis recorded");
    CHECK(sysid_capture_count() == 0, "capture count reset for the new run");
    CHECK(sysid_capture_hz() == 500, "capture rate is 500 Hz");
  }

  printf("  [4] what the control loop captures is what the dump returns\n");
  {
    /* Feed known values on the excited axis (pitch = index 1). u is scaled by
     * 1000 and the gyro by 10 on the way in, so pick values that survive the
     * i16 quantisation exactly. */
    const int n = 25;
    for (int i = 0; i < n; i++) {
      const float u[3] = {0.0f, (float)(i + 1) * 0.001f, 0.0f};
      const float g[3] = {0.0f, (float)(i + 1) * 0.1f, 0.0f};
      for (int d = 0; d < CAPTURE_PER_SAMPLE; d++) {
        sysid_capture(u, g);
      }
    }
    CHECK(sysid_capture_count() == n, "one sample per decimation group");

    sysid_dump_request();
    CHECK(sysid_dump_active(), "dump starts with samples pending");

    int seen = 0;
    int ordered = 1;
    while (sysid_dump_active()) {
      uint16_t start = 0xFFFF;
      int16_t u[10], g[10];
      int got = sysid_dump_next(&start, u, g, 10);
      if (got <= 0) {
        break;
      }
      if (start != (uint16_t)seen) {
        ordered = 0;
      }
      for (int i = 0; i < got; i++) {
        /* i-th sample carried u = (i+1)/1000 and gyro = (i+1)/10. */
        const int16_t want_u = (int16_t)(seen + i + 1);
        const int16_t want_g = (int16_t)(seen + i + 1);
        if (u[i] != want_u || g[i] != want_g) {
          ordered = 0;
        }
      }
      seen += got;
    }
    CHECK(seen == n, "the dump returned every captured sample");
    CHECK(ordered, "samples came back in order with their values intact");
    CHECK(!sysid_dump_active(), "dump ends when the cursor is drained");
  }

  printf("  [5] abort stops capture without dropping what was captured\n");
  {
    const int before = sysid_capture_count();
    sysid_abort();
    CHECK(!sysid_active(), "abort cleared the active flag");
    const float u[3] = {0.0f, 9.0f, 0.0f};
    const float g[3] = {0.0f, 9.0f, 0.0f};
    for (int i = 0; i < 8; i++) {
      sysid_capture(u, g);
    }
    CHECK(sysid_capture_count() == before, "no capture after an abort");
  }

  printf("  [6] a second run reuses the buffer and resets the count\n");
  {
    /* The buffer is allocated once and kept; a re-start must not leak a second
     * one, and must not serve the previous run's samples. */
    sysid_request_t r = req(0);
    CHECK(sysid_start(&r) != 0, "second run starts");
    CHECK(sysid_capture_count() == 0, "count reset");
    CHECK(sysid_capture_axis() == 0, "new axis recorded");
    sysid_abort();
  }

  printf("\n  %d checks, %d failures\n", g_checks, g_fails);
  return g_fails ? 1 : 0;
}
