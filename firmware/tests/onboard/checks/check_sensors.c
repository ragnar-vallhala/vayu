/* Bench checks: sensors on the shared I2C1 bus. C1 seed set (WHO_AM_I). */
#include "hwtest_runner.h"
#include "sensor/bmx160.h"
#include "sensor/bme280.h"

/* BMX160 IMU identity over I2C1 (WHO_AM_I @ 0x00 == 0xD8).
 * @verifies SNS-BMX-101 */
hw_result_t check_imu_whoami(void) {
  uint16_t id = bmx160_get_chip_id();
  return id == BMX160_CHIP_ID ? hw_pass((float)id, "id")
                              : hw_fail((float)id, "id");
}

/* BME280 baro identity on the same bus (chip id @ 0xD0 == 0x60). An absent or
 * mis-wired baro degrades rather than faults the FC, so a mismatch is reported,
 * not asserted fatally.
 * @verifies SNS-BARO-001 */
hw_result_t check_baro_whoami(void) {
  uint8_t id = bme280_get_chip_id();
  return id == BME280_CHIP_ID ? hw_pass((float)id, "id")
                              : hw_fail((float)id, "id");
}
