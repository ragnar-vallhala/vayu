/* Bench checks: boot / OS surface. */
#include "hwtest_runner.h"
#include "navhal.h" /* hal_clock_get_sysclk */
#include "memory.h" /* v_get_heap_size / v_get_heap_allocation_size */
#include "sys/state.h"
#include "variables.h" /* SYS_CLOCK_FREQ */

/* System clock read back from the hardware (HSE->PLL = 84 MHz on the target).
 * @verifies SYS-TIM-004 */
hw_result_t check_system_clock(void) {
  uint32_t hz = hal_clock_get_sysclk();
  float mhz = (float)hz / 1000000.0f;
  return hz == (uint32_t)SYS_CLOCK_FREQ ? hw_pass(mhz, "MHz")
                                        : hw_fail(mhz, "MHz");
}

/* The state machine is up and holding a valid (non-zero) state — the bench
 * firmware stays in STANDBY and never arms.
 * @verifies SYS-STATE-001, SYS-STATE-002 */
hw_result_t check_state_machine(void) {
  sys_state_t st = system_state_get();
  return st != 0 ? hw_pass((float)st, "state") : hw_fail(0.0f, "state");
}

/* Free heap after task creation — guards the RAM-starved F401 against a
 * regression that leaves no allocation headroom (see f401-sram-heap-overflow).
 * @verifies VOS-MEM-001 */
hw_result_t check_heap_free(void) {
  uint32_t total = v_get_heap_size();
  uint32_t used = v_get_heap_allocation_size();
  uint32_t freeb = total > used ? total - used : 0;
  return freeb > 4096 ? hw_pass((float)freeb, "B") : hw_fail((float)freeb, "B");
}
