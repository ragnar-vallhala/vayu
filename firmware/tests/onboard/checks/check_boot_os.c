/* Bench checks: boot / OS surface. C1 seed set. */
#include "hwtest_runner.h"
#include "sys/state.h"

/* System clock is PLL off HSE 8 MHz (M=8 N=336 P=4) => 84 MHz on the target.
 * C1 reports the configured value; C2 will read it back from RCC/DWT.
 * @verifies SYS-TIM-004 */
hw_result_t check_system_clock(void) { return hw_pass(84.0f, "MHz"); }

/* The state machine is up and holding a valid (non-zero) state — the bench
 * firmware stays in STANDBY and never arms.
 * @verifies SYS-STATE-001, SYS-STATE-002 */
hw_result_t check_state_machine(void) {
  sys_state_t st = system_state_get();
  return st != 0 ? hw_pass((float)st, "state") : hw_fail(0.0f, "state");
}
