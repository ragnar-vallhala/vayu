#include "sys/state.h"

volatile sys_state_t _system_current_status = SYSTEM_STATE_UNINITIALIZED;
volatile sys_boot_check_state_t _system_boot_check_current_status = BOOT_CHECK_NO_CHECK;
volatile sys_imu_health_check_state_t _system_imu_health_check_current_status =
    IMU_HEALTH_NO_CHECK;
