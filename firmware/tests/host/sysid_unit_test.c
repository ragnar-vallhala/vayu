/* sysid_unit_test.c — host unit test for control/sysid.c.
 *
 * On-hardware system-ID chirp injection. The whole excite/capture/abort path is
 * pure (no HAL — motors move only downstream), so it is exercisable on host:
 * idle is silent, an active run injects a tapered chirp on the excited axis
 * only, the run auto-stops at its duration, a limit excursion self-aborts, and
 * the capture buffer fills on the excited axis.
 *
 * Build/run (from firmware/):
 *   gcc -std=c11 -Iinclude tests/host/sysid_unit_test.c src/control/sysid.c \
 *       -lm -o /tmp/st && /tmp/st
 */
#include "control/sysid.h"

#include <math.h>
#include <stdio.h>

static int fails = 0;
static void check(const char *what, int ok) {
  printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) fails++;
}

int main(void) {
  const float zero3[3] = {0, 0, 0};

  /* 1. Idle: not active, injects nothing. */
  printf("Test 1: idle is silent\n");
  {
    float inj[3] = {9, 9, 9};
    check("not active at start", sysid_active() == 0);
    sysid_step(0.001f, zero3, zero3, inj);
    check("inject all zero when idle",
          inj[0] == 0 && inj[1] == 0 && inj[2] == 0);
  }

  /* 2. Start a pitch (axis 1) RATE_SP chirp: active, right mode, injects on the
   *    excited axis only, bounded by the requested amplitude. */
  printf("Test 2: active run injects on the excited axis only\n");
  {
    sysid_request_t req = {.axis = 1,
                           .mode = SYSID_INJECT_RATE_SP,
                           .f0_hz = 1.0f,
                           .f1_hz = 5.0f,
                           .amp_dps = 10.0f,
                           .duration_s = 0.5f};
    sysid_start(&req);
    check("active after start", sysid_active() != 0);
    check("inject mode == RATE_SP", sysid_inject_mode() == SYSID_INJECT_RATE_SP);

    float maxabs = 0, off_axis = 0;
    int nonzero = 0;
    for (int i = 0; i < 200; i++) { /* 0.2s of a 0.5s run */
      float inj[3] = {0, 0, 0};
      sysid_step(0.001f, zero3, zero3, inj);
      if (fabsf(inj[1]) > maxabs) maxabs = fabsf(inj[1]);
      if (inj[1] != 0) nonzero++;
      off_axis += fabsf(inj[0]) + fabsf(inj[2]);
    }
    check("excited axis perturbed", nonzero > 0);
    check("other axes stay zero", off_axis == 0.0f);
    check("amplitude bounded by request", maxabs > 0.0f && maxabs <= 11.0f);
    sysid_abort();
  }

  /* 3. Auto-stop at duration. */
  printf("Test 3: run auto-stops at its duration\n");
  {
    sysid_request_t req = {.axis = 0,
                           .mode = SYSID_INJECT_RATE_SP,
                           .f0_hz = 2.0f,
                           .f1_hz = 2.0f,
                           .amp_dps = 5.0f,
                           .duration_s = 0.1f};
    sysid_start(&req);
    float inj[3];
    for (int i = 0; i < 150; i++) sysid_step(0.001f, zero3, zero3, inj); /* 0.15s */
    check("inactive past duration", sysid_active() == 0);
  }

  /* 4. Self-abort on a limit excursion on the excited axis. */
  printf("Test 4: self-abort on limit excursion\n");
  {
    sysid_request_t req = {.axis = 2,
                           .mode = SYSID_INJECT_RATE_SP,
                           .f0_hz = 1.0f,
                           .f1_hz = 1.0f,
                           .amp_dps = 5.0f,
                           .duration_s = 5.0f};
    sysid_start(&req);
    float inj[3];
    sysid_step(0.001f, zero3, zero3, inj);
    check("active before excursion", sysid_active() != 0);
    /* a wildly out-of-range rate on the excited axis must trip the self-abort */
    float wild_rate[3] = {0, 0, 1.0e6f};
    sysid_step(0.001f, zero3, wild_rate, inj);
    check("aborted after excursion", sysid_active() == 0);
    check("inject returns to zero after abort",
          inj[0] == 0 && inj[1] == 0 && inj[2] == 0);
  }

  /* 5. Capture records the excited axis; abort silences injection. */
  printf("Test 5: capture + manual abort\n");
  {
    sysid_request_t req = {.axis = 1,
                           .mode = SYSID_INJECT_U,
                           .f0_hz = 1.0f,
                           .f1_hz = 3.0f,
                           .amp_dps = 0.3f,
                           .duration_s = 1.0f};
    sysid_start(&req);
    check("inject mode == U", sysid_inject_mode() == SYSID_INJECT_U);
    float inj[3];
    float u[3] = {0.1f, 0.2f, 0.3f}, gyro[3] = {1, 2, 3};
    for (int i = 0; i < 100; i++) {
      sysid_step(0.001f, zero3, zero3, inj);
      sysid_capture(u, gyro);
    }
    check("samples captured", sysid_capture_count() > 0);
    check("capture axis == excited axis", sysid_capture_axis() == 1);
    sysid_abort();
    check("inactive after abort", sysid_active() == 0);
    float inj2[3] = {7, 7, 7};
    sysid_step(0.001f, zero3, zero3, inj2);
    check("inject zero after abort",
          inj2[0] == 0 && inj2[1] == 0 && inj2[2] == 0);
  }

  printf("\n%s (%d failure%s)\n", fails ? "FAILED" : "ALL PASSED", fails,
         fails == 1 ? "" : "s");
  return fails ? 1 : 0;
}
