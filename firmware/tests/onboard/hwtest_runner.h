#ifndef VAYU_HWTEST_RUNNER_H
#define VAYU_HWTEST_RUNNER_H
/* On-hardware bench test harness (Suite B / C1).
 * Built only into the `hwtest` image (-DVAYU_HW_TEST=ON); never in production.
 * See docs/plans/on-hardware-test-and-coverage.md. */
#include <stdint.h>

typedef enum { HW_FAIL = 0, HW_PASS = 1, HW_SKIP = 2 } hw_status_t;

/* One check's outcome. `value`/`units` carry the measured number (clock MHz,
 * heap bytes, gyro sigma, chip id) so the host records trends, not just P/F. */
typedef struct {
  hw_status_t status;
  float value;
  const char *units; /* <= 7 chars; "" if none */
} hw_result_t;

/* A bench check: a stable <=23-char name + a function returning its result. */
typedef struct {
  const char *name;
  hw_result_t (*fn)(void);
} hw_check_t;

/* Null-terminated registry, defined in hwtest_main.c from the check functions. */
extern const hw_check_t hwtest_registry[];

/* Walk the registry: emit HW_TEST_BEGIN, one HW_TEST_RESULT per check (live over
 * NavLink + a vayu_log line that also lands in the SD text log), then
 * HW_TEST_DONE. Stays in STANDBY; never arms. */
void hwtest_run_all(void);

/* Result constructors. */
static inline hw_result_t hw_pass(float v, const char *u) {
  hw_result_t r = {HW_PASS, v, u};
  return r;
}
static inline hw_result_t hw_fail(float v, const char *u) {
  hw_result_t r = {HW_FAIL, v, u};
  return r;
}
static inline hw_result_t hw_skip(void) {
  hw_result_t r = {HW_SKIP, 0.0f, ""};
  return r;
}

#endif /* VAYU_HWTEST_RUNNER_H */
