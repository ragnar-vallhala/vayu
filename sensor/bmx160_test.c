#include "bmx160_test.h"
#include "bmx160/bmx160.h"
#include "core/cortex-m4/uart.h"
#include "utils.h"
#include "vaios.h"
#include <stdint.h>

void run_bmx() {
  hal_i2c_status_t status = bmx160_init();
  v_log(LOG_DEBUG, "BMX160 init status: %d", status);
  float temp = 0;
  bmx160_config_t cfg;
  bmx160_all_reading_t data;
  // --- Accelerometer config ---
  cfg.bmx160_acc_us = 0;    // undersampling disabled
  cfg.bmx160_acc_bwp = 1;   // OSR2 (higher filter bandwidth)
  cfg.bmx160_acc_odr = 12;  // 400 Hz (see datasheet ODR table)
  cfg.bmx160_acc_range = 8; // ±8g

  // --- Gyroscope config ---
  cfg.bmx160_gyr_bwp = 0;   // OSR4 (narrow filter bandwidth, high precision)
  cfg.bmx160_gyr_odr = 13;  // 800 Hz
  cfg.bmx160_gyr_range = 1; // ±1000 dps

  // --- Magnetometer config ---
  cfg.bmx160_mag_odr = 6; // 50 Hz

  // --- Write configuration ---
  bmx160_write_config(&cfg);

  while (1) {
    bmx160_read_temp_celcius(&temp);

    // bmx160_read_config(&cfg);
    // v_log(LOG_DEBUG, "ACC_ODR: %d, ACC_BWP: %d, ACC_US: %d, ACC_RANGE: %d",
    //       cfg.bmx160_acc_odr, cfg.bmx160_acc_bwp, cfg.bmx160_acc_us,
    //       cfg.bmx160_acc_range);
    // v_log(LOG_DEBUG, "GYR_ODR: %d, GYR_BWP: %d, GYR_RANGE: %d",
    //       cfg.bmx160_gyr_odr, cfg.bmx160_gyr_bwp, cfg.bmx160_gyr_range);
    // v_log(LOG_DEBUG, "MAG_ODR: %d", cfg.bmx160_mag_odr);

    bmx160_err_type err = bmx160_read_all_converted(&data);

    // if (err == NO_ERR) {
    //   v_log(LOG_DEBUG, "\n>Acc_x: %d", data.raw.acc[0]);
    //   v_log(LOG_DEBUG, "\n>Acc_y: %d", data.raw.acc[1]);
    //   v_log(LOG_DEBUG, "\n>Acc_z: %d", data.raw.acc[2]);
    //
    //   v_log(LOG_DEBUG, "\n>Gyr_x: %d", data.raw.gyr[0]);
    //   v_log(LOG_DEBUG, "\n>Gyr_y: %d", data.raw.gyr[1]);
    //   v_log(LOG_DEBUG, "\n>Gyr_z: %d", data.raw.gyr[2]);
    //
    //   v_log(LOG_DEBUG, "\n>Mag_x: %d", data.raw.mag[0]);
    //   v_log(LOG_DEBUG, "\n>Mag_y: %d", data.raw.mag[1]);
    //   v_log(LOG_DEBUG, "\n>Mag_z: %d", data.raw.mag[2]);
    // } else {
    //   v_log(LOG_DEBUG, "Error Code: %d", err);
    // }

    v_delay(10);
  }
}
