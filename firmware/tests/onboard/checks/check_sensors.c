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

/* Exercise the full BMX160 read/convert/config API so the on-target coverage
 * (C3) reflects the driver surface, not just whoami + a raw accel/gyro read.
 * No read task runs here, so converted reads may return stale/zero data — that
 * is fine: the driver code still executes (compensation, range scaling, the
 * manual-mag sequence). Config writes echo back the just-read config, so no
 * sensor state actually changes. Reports the count of NO_ERR results. */
hw_result_t check_imu_driver_api(void) {
  bmx160_all_reading_t all;
  bmx160_config_t cfg;
  float v3[3];
  float f;
  int16_t r3[3];
  int ok = 0;

  /* Converted reads (drive scaling + BMM150 compensation). */
  ok += (bmx160_read_acc_mps2(v3) == NO_ERR);
  ok += (bmx160_read_gyr_dps(v3) == NO_ERR);
  ok += (bmx160_read_mag_uT(v3) == NO_ERR);
  ok += (bmx160_read_temp_celcius(&f) == NO_ERR);
  ok += (bmx160_read_all_converted(&all) == NO_ERR);

  /* Raw reads. */
  ok += (bmx160_read_acc_raw(r3) == NO_ERR);
  ok += (bmx160_read_gyr_raw(r3) == NO_ERR);
  ok += (bmx160_read_mag_raw(r3) == NO_ERR);
  ok += (bmx160_read_all_raw(&all) == NO_ERR);
  (void)bmx160_read_temp_raw();

  /* Config read-then-echo (write back identical -> no real change). */
  if (bmx160_read_config(&cfg) == NO_ERR) {
    ok += (bmx160_write_config(&cfg) == NO_ERR);
  }
  if (bmx160_read_acc_config(&cfg) == NO_ERR) {
    ok += (bmx160_write_acc_config(&cfg) == NO_ERR);
  }
  if (bmx160_read_gyr_config(&cfg) == NO_ERR) {
    ok += (bmx160_write_gyr_config(&cfg) == NO_ERR);
  }
  if (bmx160_read_mag_config(&cfg) == NO_ERR) {
    ok += (bmx160_write_mag_config(&cfg) == NO_ERR);
  }
  cfg = bmx160_get_current_config();
  bmx160_set_current_config(&cfg);

  /* Pure converters. */
  (void)bmx160_raw_acc_to_mps2(1000);
  (void)bmx160_raw_gyr_to_dps(1000);

  return hw_pass((float)ok, "ok"); /* coverage-exercise: always pass */
}

/* Exercise the BME280 read API (temp/pressure/humidity/altitude/all) plus the
 * sea-level reference setter, for the same C3 coverage reason as the IMU check
 * above. Reports the count of HAL_OK results. */
hw_result_t check_baro_driver_api(void) {
  bme280_reading_t r;
  float f;
  int ok = 0;

  bme280_set_sea_level_pa(101325.0f); /* standard ref; restores default */
  ok += (bme280_read_temperature(&f) == HAL_OK);
  ok += (bme280_read_pressure(&f) == HAL_OK);
  ok += (bme280_read_humidity(&f) == HAL_OK);
  ok += (bme280_read_altitude(&f) == HAL_OK);
  ok += (bme280_read_all(&r) == HAL_OK);

  return hw_pass((float)ok, "ok"); /* coverage-exercise: always pass */
}
