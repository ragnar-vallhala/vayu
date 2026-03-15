#ifndef SYS_STATE_H
#define SYS_STATE_H
typedef enum {
  SYSTEM_STATE_UNINITIALIZED = 0x1,
  SYSTEM_STATE_INIT = 0x2,
  SYSTEM_STATE_STANDBY = 0x4,
  SYSTEM_STATE_PREARM = 0x8,
  SYSTEM_STATE_ARMED = 0x10,
  SYSTEM_STATE_IN_AIR = 0x20,
  SYSTEM_STATE_FAILSAFE=0x40,
  SYSTEM_STATE_TERMINATED=0x80,
  SYSTEM_STATE_CALIBRATING=0x100,
} sys_state_t;
extern sys_state_t _system_current_status;
inline void system_state_init() { _system_current_status = SYSTEM_STATE_INIT; }
inline void system_state_set(sys_state_t state) {
  _system_current_status = state;
}
inline sys_state_t system_state_get() { return _system_current_status; }

typedef enum {
  BOOT_CHECK_NO_CHECK = 0x1,
  BOOT_CHECK_STARTUP_CHECK_PASS = 0x2,
  BOOT_CHECK_STARTUP_CHECK_FAIL = 0x4,
  BOOT_CHECK_SYSTEM_CLOCK_CHECK_PASS = 0x8,
  BOOT_CHECK_SYSTEM_CLOCK_CHECK_FAIL = 0x10,
  BOOT_CHECK_SD_CARD_CHECK_PASS = 0x20,
  BOOT_CHECK_SD_CARD_CHECK_FAIL = 0x40,
} sys_boot_check_state_t;

extern sys_boot_check_state_t _system_boot_check_current_status;
inline void system_boot_check_state_init() {
  _system_boot_check_current_status = BOOT_CHECK_NO_CHECK;
}
inline void system_boot_check_state_set(sys_boot_check_state_t state) {
  _system_boot_check_current_status = state;
}
inline sys_boot_check_state_t system_boot_check_state_get() {
  return _system_boot_check_current_status;
}

typedef enum {
  IMU_HEALTH_NO_CHECK = 0x1,
  IMU_HEALTH_GYRO_PRESENT = 0x2,
  IMU_HEALTH_GYRO_NOT_PRESENT = 0x4,
  IMU_HEALTH_GYRO_CALIBRATION_VALID = 0x8,
  IMU_HEALTH_GYRO_CALIBRATION_INVALID = 0x10,
  IMU_HEALTH_GYRO_TEMP_VALID = 0x20,
  IMU_HEALTH_GYRO_TEMP_INVALID = 0x40,
  IMU_HEALTH_ACCEL_PRESENT = 0x80,
  IMU_HEALTH_ACCEL_NOT_PRESENT = 0x100,
  IMU_HEALTH_ACCEL_CALIBRATION_VALID = 0x200,
  IMU_HEALTH_ACCEL_CALIBRATION_INVALID = 0x400,
  IMU_HEALTH_ACCEL_TEMP_VALID = 0x800,
  IMU_HEALTH_ACCEL_TEMP_INVALID = 0x1000,
  IMU_HEALTH_MAG_PRESENT = 0x2000,
  IMU_HEALTH_MAG_NOT_PRESENT = 0x4000,
  IMU_HEALTH_MAG_CALIBRATION_VALID = 0x8000,
  IMU_HEALTH_MAG_CALIBRATION_INVALID = 0x10000,
  IMU_HEALTH_MAG_TEMP_VALID = 0x20000,
  IMU_HEALTH_MAG_TEMP_INVALID = 0x40000
} sys_imu_health_check_state_t;

extern sys_imu_health_check_state_t _system_imu_health_check_current_status;
inline void system_imu_health_check_state_init() {
  _system_imu_health_check_current_status = IMU_HEALTH_NO_CHECK;
}
inline void
system_imu_health_check_state_set(sys_imu_health_check_state_t state) {
  _system_imu_health_check_current_status = state;
}
inline sys_imu_health_check_state_t system_imu_health_check_state_get() {
  return _system_imu_health_check_current_status;
}

#endif /* SYS_STATE_H */