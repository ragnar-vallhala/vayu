/**
 * @file src/sys/state.c
 * @brief Validated system-state transitions.
 *
 * @implements SYS-SAFE-006, SYS-STATE-001, SYS-STATE-002
 *
 * Backs the public system_state_set() declared in include/sys/state.h
 * with a static allowed-transitions table. Rejected transitions are
 * logged and reported to the caller via VAYU_ERR_INVALID; FAILSAFE
 * remains reachable from every state regardless of the table because
 * safety overrides protocol.
 */
#include "sys/state.h"

#include "storage/fs_owner.h"   /* vayu_log */
#include "vayu_status.h"

#include <stddef.h>

volatile sys_state_t _system_current_status = SYSTEM_STATE_UNINITIALIZED;
volatile sys_boot_check_state_t _system_boot_check_current_status = BOOT_CHECK_NO_CHECK;
volatile sys_imu_health_check_state_t _system_imu_health_check_current_status =
    IMU_HEALTH_NO_CHECK;

/* Allowed normal transitions. Self-loops and FAILSAFE-as-target are
 * handled outside the table (always allowed). PREARM rows are included
 * speculatively — the state exists in the enum but no current task
 * drives that transition; including the rows means the future
 * pre-arm-check phase doesn't need to revisit this table. */
static const sys_state_t k_allowed_transitions[][2] = {
    {SYSTEM_STATE_UNINITIALIZED, SYSTEM_STATE_INIT},
    {SYSTEM_STATE_INIT,          SYSTEM_STATE_STANDBY},
    {SYSTEM_STATE_STANDBY,       SYSTEM_STATE_PREARM},
    {SYSTEM_STATE_STANDBY,       SYSTEM_STATE_ARMED},
    {SYSTEM_STATE_STANDBY,       SYSTEM_STATE_CALIBRATING},
    /* Bench calibration: the FC sits in FAILSAFE whenever there is no RC link
     * (rc_watchdog_step), which is the normal state for a GCS-driven ground
     * calibration. Allow it from there; the task returns to STANDBY on
     * completion (and the watchdog may re-enter FAILSAFE, harmlessly). */
    {SYSTEM_STATE_FAILSAFE,      SYSTEM_STATE_CALIBRATING},
    {SYSTEM_STATE_PREARM,        SYSTEM_STATE_ARMED},
    {SYSTEM_STATE_PREARM,        SYSTEM_STATE_STANDBY},
    {SYSTEM_STATE_ARMED,         SYSTEM_STATE_IN_AIR},
    {SYSTEM_STATE_ARMED,         SYSTEM_STATE_STANDBY},
    {SYSTEM_STATE_IN_AIR,        SYSTEM_STATE_ARMED},
    {SYSTEM_STATE_IN_AIR,        SYSTEM_STATE_STANDBY},
    {SYSTEM_STATE_CALIBRATING,   SYSTEM_STATE_STANDBY},
    {SYSTEM_STATE_FAILSAFE,      SYSTEM_STATE_STANDBY},
};

static const size_t k_allowed_count =
    sizeof(k_allowed_transitions) / sizeof(k_allowed_transitions[0]);

vayu_status_t system_state_set(sys_state_t new_state) {
  sys_state_t cur = _system_current_status;
  if (cur == new_state) {
    return VAYU_OK;
  }
  if (new_state == SYSTEM_STATE_FAILSAFE) {
    _system_current_status = new_state;
    return VAYU_OK;
  }
  for (size_t i = 0; i < k_allowed_count; i++) {
    if (k_allowed_transitions[i][0] == cur &&
        k_allowed_transitions[i][1] == new_state) {
      _system_current_status = new_state;
      return VAYU_OK;
    }
  }
  vayu_log("[STATE] rejected transition 0x%x -> 0x%x",
           (unsigned)cur, (unsigned)new_state);
  return VAYU_ERR_INVALID;
}
