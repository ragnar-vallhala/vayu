#include "bmx160.h"
#include "utils.h"
#include "vaios.h"

#include "variables.h"
#include <stdint.h>

uint8_t tx_buf[2];
uint8_t rx_buf[12];
static uint8_t status = 0;

hal_i2c_status_t bmx160_init(void) {
  // I2C configuration
  hal_i2c_config_t i2c_config = {.clock_speed = STANDARD_MODE,
                                 .own_address = I2C_MASTER,
                                 .acknowledge = true};
  // Configure GPIO for I2C1 (PB8=SCL, PB9=SDA)
  hal_gpio_set_alternate_function(I2C_PIN_1, GPIO_FUNC_I2C);
  hal_gpio_set_alternate_function(I2C_PIN_2, GPIO_FUNC_I2C);
  hal_gpio_set_output_type(I2C_PIN_1, GPIO_OPEN_DRAIN);
  hal_gpio_set_output_type(I2C_PIN_2, GPIO_OPEN_DRAIN);
  hal_gpio_set_output_speed(I2C_PIN_1, GPIO_VERY_HIGH_SPEED);
  hal_gpio_set_output_speed(I2C_PIN_2, GPIO_VERY_HIGH_SPEED);
  hal_i2c_status_t ts = hal_i2c_init(I2C_BUS, &i2c_config);

  // BMX160 Configuration
  if (!(ts == HAL_I2C_OK || ts == HAL_I2C_ERR_REINIT)) {
    v_log(LOG_ERROR, "Failed to start BMX160, status code: %d", ts);
  } else {
    tx_buf[0] = BMX160_PWR_CONF_REG;
    tx_buf[1] = 0x00;
    hal_i2c_write(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 2);

    // Put accelerometer and gyroscope into normal mode
    tx_buf[0] = BMX160_CMD_REG;
    tx_buf[1] = BMX160_CMD_ACC_NORMAL;
    hal_i2c_write(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 2);
    v_delay(10);

    tx_buf[0] = BMX160_CMD_REG; // register address (0x7E)
    tx_buf[1] = BMX160_CMD_GYR_NORMAL;
    hal_i2c_write(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 2);
    v_delay(100);

    tx_buf[0] = BMX160_CMD_REG; // register address (0x7E)
    tx_buf[1] = BMX160_CMD_MAG_NORMAL;
    hal_i2c_write(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 2);
    v_delay(100);

    v_log(LOG_INFO, "BMX160 started.");
  }

  return status;
}

uint16_t bmx160_get_chip_id(void) {
  uint8_t reg = BMX160_CHIP_ID_ADDR;
  hal_i2c_status_t ret;

  // Read 2 bytes from temperature registers (0x20, 0x21)
  ret = hal_i2c_write_read(I2C_BUS, BMX160_I2C_ADDR, &reg, 1, rx_buf, 1);
  if (ret != HAL_I2C_OK) {
    v_log(LOG_ERROR, "Failed to read BMX160 chip id, status=%d", ret);
    return 0xFFFF;
  }

  int16_t chip_id = (int16_t)(rx_buf[0]);

  return (uint16_t)chip_id; // keep raw for now
}

int16_t bmx160_read_temp_raw(void) {
  if (status != 1) {
    // v_log(LOG_ERROR, "BMX160 not initialized but called
    // bmx160_read_temp_raw"); return 0xFFFF; // return invalid value
  }

  tx_buf[0] = BMX160_TEMPERATURE_1_ADDR;
  hal_i2c_status_t ret;

  // Read 2 bytes from temperature registers (0x20, 0x21)
  ret = hal_i2c_write_read(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 1, rx_buf, 1);
  // Combine MSB and LSB (two's complement, little-endian)
  if (ret != HAL_I2C_OK) {
    v_log(LOG_ERROR, "Failed to read BMX160 temperature, status=%d", ret);
    return 0xFFFF;
  }
  int16_t temp_raw = (int16_t)(rx_buf[0] << 8);

  tx_buf[0] = BMX160_TEMPERATURE_1_ADDR;
  ret = hal_i2c_write_read(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 1, rx_buf, 1);
  // Combine MSB and LSB (two's complement, little-endian)
  if (ret != HAL_I2C_OK) {
    v_log(LOG_ERROR, "Failed to read BMX160 temperature, status=%d", ret);
    return 0xFFFF;
  }
  // Combine MSB and LSB (two's complement, little-endian)
  temp_raw = (int16_t)(temp_raw | rx_buf[1]);

  return temp_raw; // keep raw for now
}

bmx160_err_type bmx160_convert_raw_temp_to_celcius(int16_t raw_temp,
                                                   float *celcius) {
  if (raw_temp == 0x8000)
    return INVALID_DATA_READ;
  v_log(LOG_DEBUG, "RAW TEMP: %X", raw_temp);
  float raw = (float)raw_temp;
  raw /= 512.0f;
  raw += 23;
  *celcius = raw;
  return NO_ERR;
}

bmx160_err_type bmx160_read_temp_celcius(float *celcius) {
  int16_t raw = bmx160_read_temp_raw();
  return bmx160_convert_raw_temp_to_celcius(raw, celcius);
}

bmx160_err_type bmx160_read_acc_raw(int16_t *raw) {
  uint8_t reg = BMX160_ACCX_LOW_ADDR; // Start address of accel X LSB
  hal_i2c_status_t ret;

  // Read 6 bytes: X_L, X_H, Y_L, Y_H, Z_L, Z_H
  ret = hal_i2c_write_read(I2C_BUS, BMX160_I2C_ADDR, &reg, 1, rx_buf, 6);
  if (ret != HAL_I2C_OK) {
    v_log(LOG_ERROR, "Failed to read BMX160 accelerometer, status=%d", ret);
    return ERR0;
  }

  // Combine little-endian pairs into signed 16-bit values
  raw[0] = (int16_t)((rx_buf[1] << 8) | rx_buf[0]); // X
  raw[1] = (int16_t)((rx_buf[3] << 8) | rx_buf[2]); // Y
  raw[2] = (int16_t)((rx_buf[5] << 8) | rx_buf[4]); // Z

  v_log(LOG_DEBUG, "ACC RAW: X=%d, Y=%d, Z=%d", raw[0], raw[1], raw[2]);

  return NO_ERR;
}
bmx160_err_type bmx160_read_gyr_raw(int16_t *raw) {
  uint8_t reg = BMX160_GYRX_LOW_ADDR; // Start address of accel X LSB
  hal_i2c_status_t ret;

  // Read 6 bytes: X_L, X_H, Y_L, Y_H, Z_L, Z_H
  ret = hal_i2c_write_read(I2C_BUS, BMX160_I2C_ADDR, &reg, 1, rx_buf, 6);
  if (ret != HAL_I2C_OK) {
    v_log(LOG_ERROR, "Failed to read BMX160 gyroscope, status=%d", ret);
    return ERR0;
  }

  // Combine little-endian pairs into signed 16-bit values
  raw[0] = (int16_t)((rx_buf[1] << 8) | rx_buf[0]); // X
  raw[1] = (int16_t)((rx_buf[3] << 8) | rx_buf[2]); // Y
  raw[2] = (int16_t)((rx_buf[5] << 8) | rx_buf[4]); // Z

  v_log(LOG_DEBUG, "GYR RAW: X=%d, Y=%d, Z=%d", raw[0], raw[1], raw[2]);

  return NO_ERR;
}
bmx160_err_type bmx160_read_mag_raw(int16_t *raw) {
  uint8_t reg = BMX160_MAGX_LOW_ADDR; // Start address of accel X LSB
  hal_i2c_status_t ret;

  // Read 6 bytes: X_L, X_H, Y_L, Y_H, Z_L, Z_H
  ret = hal_i2c_write_read(I2C_BUS, BMX160_I2C_ADDR, &reg, 1, rx_buf, 6);
  if (ret != HAL_I2C_OK) {
    v_log(LOG_ERROR, "Failed to read BMX160 magnetometer, status=%d", ret);
    return ERR0;
  }

  // Combine little-endian pairs into signed 16-bit values
  raw[0] = (int16_t)((rx_buf[1] << 8) | rx_buf[0]); // X
  raw[1] = (int16_t)((rx_buf[3] << 8) | rx_buf[2]); // Y
  raw[2] = (int16_t)((rx_buf[5] << 8) | rx_buf[4]); // Z

  v_log(LOG_DEBUG, "MAG RAW: X=%d, Y=%d, Z=%d", raw[0], raw[1], raw[2]);

  return NO_ERR;
}
