/**
 * @file sim/host/tests/test_phase0_assert.c
 * @brief Verification of the VAYU_ASSERT release-mode contract (CONV-02).
 *
 *   @verifies CONV-02   VAYU_ASSERT failure handling (R9.2 release branch)
 *
 * This TU is compiled with -DNDEBUG so vayu_assert_fail() takes its
 * release branch: log the site, request SYSTEM_STATE_FAILSAFE, then halt
 * the calling task (infinite loop). To observe the FAILSAFE request
 * without hanging the test, the handler is run on a detached helper
 * thread (it spins there forever and is reaped at process exit) while the
 * main thread watches the recorded state.
 *
 * The handler's three dependencies are stubbed locally so the test links
 * only src/sys/assert.c — no full firmware core, no debug v_panic path.
 *
 * Built only under VAYU_SIM (the harness defines it for every TU).
 */
#define _GNU_SOURCE
#include <pthread.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <time.h>

#include "sys/state.h"
#include "vayu_assert.h"
#include "vayu_status.h"

static int g_checks = 0;
static int g_fails = 0;

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

/* ---- Stubs the release-branch handler depends on -------------------------*/

/* Backing store for the sys/state.h inline accessors. */
volatile sys_state_t _system_current_status = SYSTEM_STATE_INIT;

/* Records the last requested state (the behaviour under test). */
static volatile sys_state_t g_requested_state = SYSTEM_STATE_INIT;

vayu_status_t system_state_set(sys_state_t state) {
  g_requested_state = state;
  _system_current_status = state;
  return VAYU_OK;
}

void vayu_log(const char *fmt, ...) {
  (void)fmt; /* failure site logging is not the behaviour under test */
}

/* ---- Test ---------------------------------------------------------------- */

static void *assert_thread(void *arg) {
  (void)arg;
  /* Direct call (not the macro) so the cond string is irrelevant; this
   * exercises the same handler the macro invokes on a failed assertion. */
  vayu_assert_fail("test_phase0_assert.c", 1, "deliberate-failure");
  return NULL; /* unreachable: vayu_assert_fail is noreturn */
}

static void test_assert_requests_failsafe(void) {
  printf("  test_assert_requests_failsafe (CONV-02, release/NDEBUG)\n");

  _system_current_status = SYSTEM_STATE_INIT;
  g_requested_state = SYSTEM_STATE_INIT;

  pthread_t th;
  pthread_create(&th, NULL, assert_thread, NULL);

  /* The handler runs to its FAILSAFE request quickly, then spins. Poll
   * the recorded state with a bounded wait so a regression can't hang
   * the suite. */
  bool reached = false;
  for (int i = 0; i < 200; i++) { /* up to ~1 s */
    if (g_requested_state == SYSTEM_STATE_FAILSAFE) {
      reached = true;
      break;
    }
    struct timespec ts = {0, 5 * 1000 * 1000}; /* 5 ms */
    nanosleep(&ts, NULL);
  }
  pthread_detach(th); /* leave it spinning; reaped at process exit */

  CHECK(reached, "failed assertion requests SYSTEM_STATE_FAILSAFE");
}

int main(void) {
  printf("== Phase-0 assert SITL verification ==\n");

  test_assert_requests_failsafe();

  printf("\n%d checks, %d failures\n", g_checks, g_fails);
  return g_fails == 0 ? 0 : 1;
}
