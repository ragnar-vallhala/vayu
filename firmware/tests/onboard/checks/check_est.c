/* Bench check: estimator self-test. Reuses the shared EKF branch-coverage
 * scenarios (ekf_selftest_run) as one bench item — same code the host and the
 * -DEKF_SELFTEST boot path run. */
#include "hwtest_runner.h"
#include "est/ekf_selftest.h"
#include <stdbool.h>

static void noop_report(void *ctx, bool pass, const char *name) {
  (void)ctx;
  (void)pass;
  (void)name;
}

/* @verifies EST-EKF-001, EST-EKF-101, EST-EKF-102, EST-EKF-103, EST-EKF-106 */
hw_result_t check_ekf_selftest(void) {
  int fails = ekf_selftest_run(noop_report, 0);
  return fails == 0 ? hw_pass(0.0f, "fails") : hw_fail((float)fails, "fails");
}
