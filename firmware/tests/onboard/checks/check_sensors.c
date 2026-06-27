/* Bench checks: sensors on the shared I2C1 bus. Direct bounded reads — the IMU
 * read task is NOT running here, so the bus is free and i2c_manager's 5 ms mutex
 * timeout is the safety net: an absent/dead sensor returns an error fast, never
 * hangs the bench. */
#include "hwtest_runner.h"
#include "sensor/bmx160.h"
#include "sensor/bme280.h"

static int iabs(int v) { return v < 0 ? -v : v; }

/* BMX160 IMU identity over I2C1 (WHO_AM_I @ 0x00 == 0xD8).
 * @verifies SNS-BMX-101 */
hw_result_t check_imu_whoami(void) {
  uint16_t id = bmx160_get_chip_id();
  return id == BMX160_CHIP_ID ? hw_pass((float)id, "id")
                              : hw_fail((float)id, "id");
}

/* BME280 baro identity on the same bus (chip id @ 0xD0 == 0x60).
 * @verifies SNS-BARO-001 */
hw_result_t check_baro_whoami(void) {
  uint8_t id = bme280_get_chip_id();
  return id == BME280_CHIP_ID ? hw_pass((float)id, "id")
                              : hw_fail((float)id, "id");
}

/* Baro init succeeded (present + calibrated + configured). An absent baro
 * degrades rather than faults the FC, so this reports rather than asserts.
 * @verifies SNS-BARO-001 */
hw_result_t check_baro_present(void) {
  return bme280_is_present() ? hw_pass(1.0f, "ok") : hw_fail(0.0f, "ok");
}

/* Gyro reads and the board is roughly still (sum|axis| small) — catches a dead
 * or screaming gyro. Bench is on a static rig.
 * @verifies SNS-IMU-001, SNS-BMX-104 */
hw_result_t check_imu_gyro_still(void) {
  int16_t g[3];
  if (bmx160_read_gyr_raw(g) != NO_ERR) {
    return hw_fail(0.0f, "err");
  }
  int m = iabs(g[0]) + iabs(g[1]) + iabs(g[2]);
  return m < 2000 ? hw_pass((float)m, "lsb") : hw_fail((float)m, "lsb");
}

/* Accel reads and sees gravity (sum|axis| non-trivial) — catches a dead or
 * stuck-at-zero accel. Range-agnostic lower bound.
 * @verifies SNS-IMU-001, SNS-BMX-104 */
hw_result_t check_imu_accel(void) {
  int16_t a[3];
  if (bmx160_read_acc_raw(a) != NO_ERR) {
    return hw_fail(0.0f, "err");
  }
  int m = iabs(a[0]) + iabs(a[1]) + iabs(a[2]);
  return m > 500 ? hw_pass((float)m, "lsb") : hw_fail((float)m, "lsb");
}
