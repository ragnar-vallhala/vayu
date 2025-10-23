#include "bmx160_test.h"
#include "bmx160/bmx160.h"
#include "utils.h"
#include "vaios.h"
#include <stdint.h>
#include "sensor_fusion.h"

void run_bmx()
{
  hal_i2c_status_t status = bmx160_init();
  v_log(LOG_DEBUG, "BMX160 init status: %d", status);
  float temp = 0;
  attitude_t orientation;
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

  while (1)
  {
    bmx160_read_temp_celcius(&temp);

    bmx160_err_type err = bmx160_read_all_converted(&data);
    if (err != NO_ERR)
    {
      v_log(LOG_ERROR, "Error reading BMX160 data: %d", err);
      continue;
    }
    sf_acc_mag(data.converted.acc[0], data.converted.acc[1],
               data.converted.acc[2], data.converted.mag[0],
               data.converted.mag[1], data.converted.mag[2], &orientation);
    v_log(LOG_INFO, "[ATTITUDE] %f, %f, %f",
          orientation.roll,
          orientation.pitch, orientation.yaw);
    v_delay(10);
  }
}
