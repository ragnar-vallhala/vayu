#include "navhal.h"
#include "sys/state.h"
#include "task.h"      // For task_exit
#include "variables.h" // For SYS_CLOCK_FREQ
#include "vayu_tasks.h"
#include <stdbool.h>
#include <stddef.h> // For NULL
#include <stdint.h>

void boot_task(void *args) {
  (void)args;

  // Start with a clean status
  system_boot_check_state_init();
  system_state_set(SYSTEM_STATE_INIT);
  // We will accumulate status flags into a local bitmask variable
  // and periodically push it to the global state.
  // The system being alive enough to run this task implies startup checks
  // passed.
  uint32_t boot_status = BOOT_CHECK_STARTUP_CHECK_PASS;
  system_boot_check_state_set((sys_boot_check_state_t)boot_status);

  // 2. System Clock Check
  uint32_t current_sys_clock = hal_clock_get_sysclk();
  bool clock_ok = (current_sys_clock == SYS_CLOCK_FREQ);
  if (clock_ok) {
    boot_status |= BOOT_CHECK_SYSTEM_CLOCK_CHECK_PASS;
  } else {
    boot_status |= BOOT_CHECK_SYSTEM_CLOCK_CHECK_FAIL;
  }
  system_boot_check_state_set((sys_boot_check_state_t)boot_status);

  // 3. SD Card Check
  boot_status |= BOOT_CHECK_SD_CARD_CHECK_PASS;
  system_boot_check_state_set((sys_boot_check_state_t)boot_status);

  // ----------------------------------------------------
  // Evaluate Final Boot Result
  // ----------------------------------------------------
  uint32_t required_passes = BOOT_CHECK_STARTUP_CHECK_PASS |
                             BOOT_CHECK_SYSTEM_CLOCK_CHECK_PASS |
                             BOOT_CHECK_SD_CARD_CHECK_PASS;

  if ((boot_status & required_passes) == required_passes) {
    // All checks passed! Elevate system state out of INIT
    system_state_set(SYSTEM_STATE_STANDBY);
  } else {
    // One or more checks failed
    system_state_set(SYSTEM_STATE_FAILSAFE);
  }

  // The boot sequence is complete. Remove this task from the scheduler.
  task_exit();
}