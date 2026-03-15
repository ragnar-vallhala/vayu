#include "sensor/bmx160.h"
#include "maths/lpf.h"
#include "maths/sensor_fusion.h"
#include "navhal.h"
#include "sensor/imu_buffer.h"
#include "task.h"
#include "utils.h"
#include "vaios.h"
#include "variables.h"
#include <stdint.h>

uint8_t tx_buf[2];
uint8_t rx_buf[14]; // Increased for safer multi-byte reads

// Static helper functions and variables
// Default BMX160 configuration
static bmx160_config_t bmx160_cfg;

// DMA storage for 30 bytes (Mag[6], Hall[2], Gyr[6], Acc[6], Status[4],
// Temp[2])
static uint8_t _bmx_dma_rx_buffer[32] __attribute__((aligned(4)));
static attitude_t _bmx_orientation;
static bmx160_all_reading_t _bmx_data;

static float acc_scale = 0.0f;
static float gyr_scale = 0.0f;

// LPFs for sensors
static lpf_t acc_lpf[3];
static lpf_t gyr_lpf[3];

static bmm150_trim_data_t _mag_trim;

static void bmx160_set_mag_conf();
static bmx160_err_type bmx160_convert_raw_temp_to_celcius(int16_t raw_temp,
                                                          float *celcius);
static float bmx160_range_code_to_g(uint8_t range_code);
static float bmx160_range_code_to_dps(uint8_t range_code);

static bmx160_err_type bmx160_verify_pmu(uint8_t mask, uint8_t expected) {
  uint8_t reg = BMX160_PMU_STAT_ADDR;
  if (hal_i2c_write_read(I2C_BUS, BMX160_I2C_ADDR, &reg, 1, rx_buf, 1) !=
      HAL_I2C_OK) {
    return ERR0;
  }
  if ((rx_buf[0] & mask) == expected) {
    return NO_ERR;
  }
  return ERR1;
}

static void unstick_i2c_bus(void) {
  hal_gpio_setmode(I2C_PIN_1, GPIO_OUTPUT, GPIO_PULLUP);
  hal_gpio_setmode(I2C_PIN_2, GPIO_OUTPUT, GPIO_PULLUP);
  hal_gpio_set_output_type(I2C_PIN_1, GPIO_OPEN_DRAIN);
  hal_gpio_set_output_type(I2C_PIN_2, GPIO_OPEN_DRAIN);

  hal_gpio_digitalwrite(I2C_PIN_2, GPIO_HIGH);
  for (volatile int i = 0; i < 100; i++)
    ;

  for (int i = 0; i < 9; ++i) {
    hal_gpio_digitalwrite(I2C_PIN_1, GPIO_LOW);
    for (volatile int j = 0; j < 200; j++)
      ;
    hal_gpio_digitalwrite(I2C_PIN_1, GPIO_HIGH);
    for (volatile int j = 0; j < 200; j++)
      ;
  }

  hal_gpio_digitalwrite(I2C_PIN_2, GPIO_LOW);
  for (volatile int j = 0; j < 200; j++)
    ;
  hal_gpio_digitalwrite(I2C_PIN_1, GPIO_HIGH);
  for (volatile int j = 0; j < 200; j++)
    ;
  hal_gpio_digitalwrite(I2C_PIN_2, GPIO_HIGH);
  for (volatile int j = 0; j < 200; j++)
    ;
}
hal_i2c_status_t bmx160_init(void) {
  unstick_i2c_bus();
  // I2C configuration
  hal_i2c_config_t i2c_config = {
      .clock_speed = FAST_MODE, .own_address = I2C_MASTER, .acknowledge = true};
  // Configure GPIO for I2C1 (PB8=SCL, PB9=SDA)
  hal_gpio_set_alternate_function(I2C_PIN_1, GPIO_FUNC_I2C);
  hal_gpio_set_alternate_function(I2C_PIN_2, GPIO_FUNC_I2C);
  hal_gpio_set_output_type(I2C_PIN_1, GPIO_OPEN_DRAIN);
  hal_gpio_set_output_type(I2C_PIN_2, GPIO_OPEN_DRAIN);
  hal_gpio_set_output_speed(I2C_PIN_1, GPIO_VERY_HIGH_SPEED);
  hal_gpio_set_output_speed(I2C_PIN_2, GPIO_VERY_HIGH_SPEED);
  hal_i2c_status_t ts = hal_i2c_init(I2C_BUS, &i2c_config);

  if (ts != HAL_I2C_OK && ts != HAL_I2C_ERR_REINIT) {
    return ts;
  }

  // 1. Verify Chip ID
  uint16_t chip_id = bmx160_get_chip_id();
  if (chip_id != BMX160_CHIP_ID) {
    return HAL_I2C_ERR_REINIT;
  }

  // 2. Soft Reset to ensure clean state
  tx_buf[0] = BMX160_CMD_ADDR;
  tx_buf[1] = 0xB6; // softreset command
  hal_i2c_write(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 2);
  v_delay(100);

  // 3. Put accelerometer into normal mode
  int retry = 5;
  while (retry--) {
    tx_buf[0] = BMX160_CMD_ADDR;
    tx_buf[1] = BMX160_CMD_ACC_NORMAL;
    hal_i2c_write(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 2);
    v_delay(20);
    if (bmx160_verify_pmu(BMX160_PMU_STAT_ACC_MASK,
                          BMX160_PMU_STAT_ACC_NORMAL) == NO_ERR) {
      break;
    }
  }

  // 4. Put gyroscope into normal mode
  retry = 5;
  while (retry--) {
    tx_buf[0] = BMX160_CMD_ADDR;
    tx_buf[1] = BMX160_CMD_GYR_NORMAL;
    hal_i2c_write(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 2);
    v_delay(100);
    if (bmx160_verify_pmu(BMX160_PMU_STAT_GYR_MASK,
                          BMX160_PMU_STAT_GYR_NORMAL) == NO_ERR) {
      break;
    }
  }

  // 5. Put magnetometer into normal mode
  retry = 5;
  while (retry--) {
    tx_buf[0] = BMX160_CMD_ADDR;
    tx_buf[1] = BMX160_CMD_MAG_NORMAL;
    hal_i2c_write(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 2);
    v_delay(100);
    if (bmx160_verify_pmu(BMX160_PMU_STAT_MAG_MASK,
                          BMX160_PMU_STAT_MAG_NORMAL) == NO_ERR) {
      break;
    }
  }

  // 6. Configure Magnetometer (BMM150 setup)
  bmx160_set_mag_conf();

  // Config for components
  bmx160_read_config(&bmx160_cfg);

  // Apply ODR and BW settings from variables.h
  bmx160_cfg.bmx160_acc_odr = BMX_ACC_ODR;
  bmx160_cfg.bmx160_acc_bwp = BMX_ACC_BWP;
  bmx160_cfg.bmx160_acc_range = BMX_ACC_RANGE;

  bmx160_cfg.bmx160_gyr_odr = BMX_GYR_ODR;
  bmx160_cfg.bmx160_gyr_bwp = BMX_GYR_BWP;
  bmx160_cfg.bmx160_gyr_range = BMX_GYR_RANGE;

  bmx160_cfg.bmx160_mag_odr = BMX_MAG_ODR;

  bmx160_write_config(&bmx160_cfg);

  // Initialize LPFs
  for (int i = 0; i < 3; i++) {
    lpf_init(&acc_lpf[i], LPF_ACC_ALPHA); // Aggressive filtering for Acc
    lpf_init(&gyr_lpf[i], LPF_GYR_ALPHA);  // Filter Gyro as well
  }
  // Initialize orientation quaternion to identity
  _bmx_orientation.q.w = 1.0f;
  _bmx_orientation.q.x = 0.0f;
  _bmx_orientation.q.y = 0.0f;
  _bmx_orientation.q.z = 0.0f;

  return ts;
}

static bmx160_err_type bmx160_write_bmm150_reg(uint8_t reg, uint8_t data) {
  tx_buf[0] = BMX160_MAG_IF_3_DATA_ADDR;
  tx_buf[1] = data;
  if (hal_i2c_write(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 2) != HAL_I2C_OK)
    return ERR0;
  tx_buf[0] = BMX160_MAG_IF_2_REG_ADDR;
  tx_buf[1] = reg;
  if (hal_i2c_write(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 2) != HAL_I2C_OK)
    return ERR0;
  v_delay(1);
  return NO_ERR;
}

static bmx160_err_type bmx160_read_bmm150_reg(uint8_t reg, uint8_t *data) {
  tx_buf[0] = BMX160_MAG_IF_1_READ_ADDR;
  tx_buf[1] = reg;
  if (hal_i2c_write(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 2) != HAL_I2C_OK)
    return ERR0;
  v_delay(1);
  uint8_t read_reg = 0x04; // MAG_X_LSB in BMX160 is where IF data appears
  if (hal_i2c_write_read(I2C_BUS, BMX160_I2C_ADDR, &read_reg, 1, data, 1) !=
      HAL_I2C_OK)
    return ERR0;
  return NO_ERR;
}

static void bmx160_read_mag_trim_data() {
  // Enter manual mode
  tx_buf[0] = BMX160_MAG_IF_0_CONF_ADDR;
  tx_buf[1] = 0x80;
  hal_i2c_write(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 2);
  v_delay(1);

  uint8_t tmp[2];

  bmx160_read_bmm150_reg(0x5D, (uint8_t *)&_mag_trim.dig_x1);
  bmx160_read_bmm150_reg(0x5E, (uint8_t *)&_mag_trim.dig_y1);
  bmx160_read_bmm150_reg(0x64, (uint8_t *)&_mag_trim.dig_x2);
  bmx160_read_bmm150_reg(0x65, (uint8_t *)&_mag_trim.dig_y2);
  bmx160_read_bmm150_reg(0x71, &_mag_trim.dig_xy1);
  bmx160_read_bmm150_reg(0x70, (uint8_t *)&_mag_trim.dig_xy2);

  bmx160_read_bmm150_reg(0x6A, &tmp[0]);
  bmx160_read_bmm150_reg(0x6B, &tmp[1]);
  _mag_trim.dig_z1 = (uint16_t)(tmp[1] << 8 | tmp[0]);

  bmx160_read_bmm150_reg(0x68, &tmp[0]);
  bmx160_read_bmm150_reg(0x69, &tmp[1]);
  _mag_trim.dig_z2 = (int16_t)(tmp[1] << 8 | tmp[0]);

  bmx160_read_bmm150_reg(0x6E, &tmp[0]);
  bmx160_read_bmm150_reg(0x6F, &tmp[1]);
  _mag_trim.dig_z3 = (int16_t)(tmp[1] << 8 | tmp[0]);

  bmx160_read_bmm150_reg(0x62, &tmp[0]);
  bmx160_read_bmm150_reg(0x63, &tmp[1]);
  _mag_trim.dig_z4 = (int16_t)(tmp[1] << 8 | tmp[0]);

  bmx160_read_bmm150_reg(0x6C, &tmp[0]);
  bmx160_read_bmm150_reg(0x6D, &tmp[1]);
  _mag_trim.dig_xyz1 = (uint16_t)(tmp[1] << 8 | tmp[0]);

  // Exit manual mode (will be done in bmx160_set_mag_conf)
}

static void bmx160_set_mag_conf() {
  // 1. Route secondary I2C interface to Magnetometer (0x6B = 0x20)
  tx_buf[0] = BMX160_IF_CONF_ADDR;
  tx_buf[1] = 0x20;
  hal_i2c_write(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 2);
  v_delay(1);

  // 2. Take BMM150 out of suspend (Manual write 0x01 to BMM150 Reg 0x4B)
  bmx160_write_bmm150_reg(0x4B, 0x01);
  v_delay(50); // Power-up time

  // 3. Read Trim Data
  bmx160_read_mag_trim_data();

  // 4. Configure BMM150 (Regular Preset: 9 reps XY, 15 reps Z)
  bmx160_write_bmm150_reg(0x51, 0x04);
  bmx160_write_bmm150_reg(0x52, 0x0E);

  // 5. Prepare for Data Mode
  // Set BMM150 to Forced Mode (Reg 0x4C = 0x02)
  bmx160_write_bmm150_reg(0x4C, 0x02);

  // 6. Set Mag Read Address to 0x42 (Data X LSB)
  tx_buf[0] = BMX160_MAG_IF_1_READ_ADDR;
  tx_buf[1] = 0x42;
  hal_i2c_write(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 2);
  v_delay(1);

  // 7. Configure ODR (0x44 = 0x05 for 12.5Hz)
  tx_buf[0] = BMX160_MAG_CONF_ADDR;
  tx_buf[1] = 0x05;
  hal_i2c_write(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 2);
  v_delay(1);

  // 8. Enable Auto-mode (manual_en = 0, burst_read = 8 bytes)
  tx_buf[0] = BMX160_MAG_IF_0_CONF_ADDR;
  tx_buf[1] = 0x03;
  hal_i2c_write(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 2);
  v_delay(10);
}

uint16_t bmx160_get_chip_id(void) {
  uint8_t reg = BMX160_CHIP_ID_ADDR;
  hal_i2c_status_t ret;

  ret = hal_i2c_write_read(I2C_BUS, BMX160_I2C_ADDR, &reg, 1, rx_buf, 1);
  if (ret != HAL_I2C_OK) {
    return 0xFFFF;
  }
  int16_t chip_id = (int16_t)(rx_buf[0]);
  return (uint16_t)chip_id;
}

int16_t bmx160_read_temp_raw(void) {
  uint8_t reg = BMX160_TEMPERATURE_0_ADDR;

  if (hal_i2c_write_read(I2C_BUS, BMX160_I2C_ADDR, &reg, 1, rx_buf, 2) !=
      HAL_I2C_OK)
    return 0x8000;

  return (int16_t)((rx_buf[1] << 8) | rx_buf[0]);
}

static float bmm150_compensate_x(int16_t mag_data_x, uint16_t data_rhall) {
  float retval = 0;
  float process_comp_x0;
  float process_comp_x1;
  float process_comp_x2;
  float process_comp_x3;
  float process_comp_x4;

  if (mag_data_x != -4096) {
    if ((data_rhall != 0) && (_mag_trim.dig_xyz1 != 0)) {
      process_comp_x0 =
          (((float)_mag_trim.dig_xyz1) * 16384.0f / (float)data_rhall) -
          16384.0f;
      process_comp_x1 = ((float)_mag_trim.dig_xy2) *
                        (process_comp_x0 * process_comp_x0 / 268435456.0f);
      process_comp_x2 = process_comp_x1 +
                        process_comp_x0 * ((float)_mag_trim.dig_xy1) / 16384.0f;
      process_comp_x3 = ((float)_mag_trim.dig_x2) + 160.0f;
      process_comp_x4 = mag_data_x * ((process_comp_x2 + 256.0f) *
                                      (process_comp_x3 * 0.0000030517578125f));
      retval = process_comp_x4 + ((float)_mag_trim.dig_x1) * 8.0f;
      retval = retval / 16.0f;
    }
  } else {
    retval = 0.0f;
  }
  return retval;
}

static float bmm150_compensate_y(int16_t mag_data_y, uint16_t data_rhall) {
  float retval = 0.0f;
  float process_comp_y0;
  float process_comp_y1;
  float process_comp_y2;
  float process_comp_y3;
  float process_comp_y4;

  if (mag_data_y != -4096) {
    if ((data_rhall != 0) && (_mag_trim.dig_xyz1 != 0)) {
      process_comp_y0 =
          (((float)_mag_trim.dig_xyz1) * 16384.0f / (float)data_rhall) -
          16384.0f;
      process_comp_y1 = ((float)_mag_trim.dig_xy2) *
                        (process_comp_y0 * process_comp_y0 / 268435456.0f);
      process_comp_y2 = process_comp_y1 +
                        process_comp_y0 * ((float)_mag_trim.dig_xy1) / 16384.0f;
      process_comp_y3 = ((float)_mag_trim.dig_y2) + 160.0f;
      process_comp_y4 = mag_data_y * ((process_comp_y2 + 256.0f) *
                                      (process_comp_y3 * 0.0000030517578125f));
      retval = process_comp_y4 + ((float)_mag_trim.dig_y1) * 8.0f;
      retval = retval / 16.0f;
    }
  } else {
    retval = 0.0f;
  }
  return retval;
}

static float bmm150_compensate_z(int16_t mag_data_z, uint16_t data_rhall) {
  float retval = 0.0f;
  float process_comp_z0;
  float process_comp_z1;
  float process_comp_z2;
  float process_comp_z3;
  float process_comp_z4;
  float process_comp_z5;

  if (mag_data_z != -16384) {
    if ((_mag_trim.dig_z2 != 0) && (_mag_trim.dig_z1 != 0) &&
        (_mag_trim.dig_xyz1 != 0) && (data_rhall != 0)) {
      process_comp_z0 = ((float)mag_data_z) - ((float)_mag_trim.dig_z4);
      process_comp_z1 = ((float)data_rhall) - ((float)_mag_trim.dig_xyz1);
      process_comp_z2 = (((float)_mag_trim.dig_z3) * process_comp_z1);
      process_comp_z3 =
          ((float)_mag_trim.dig_z1) * ((float)data_rhall) / 32768.0f;
      process_comp_z4 = ((float)_mag_trim.dig_z2) + process_comp_z3;
      process_comp_z5 = (process_comp_z0 * 131072.0f - process_comp_z2) /
                        (process_comp_z4 * 4.0f);
      retval = process_comp_z5 / 16.0f;
    }
  } else {
    retval = 0.0f;
  }
  return retval;
}

static bmx160_err_type bmx160_convert_raw_temp_to_celcius(int16_t raw_temp,
                                                          float *celcius) {
  if ((uint16_t)raw_temp == 0x8000)
    return INVALID_DATA_READ;
  float raw = (float)raw_temp;
  raw /= 512.0f;
  raw += 23;
  *celcius = raw;
  return NO_ERR;
}

bmx160_err_type bmx160_read_temp_celcius(float *celcius) {
  int16_t raw = bmx160_read_temp_raw();
  bmx160_err_type err = bmx160_convert_raw_temp_to_celcius(raw, celcius);
  if (err == NO_ERR) {
  }
  return ERR0;
}

bmx160_err_type bmx160_read_acc_raw(int16_t *raw) {
  uint8_t reg = BMX160_ACCX_LOW_ADDR; // Start address of accel X LSB
  hal_i2c_status_t ret;

  // Read 6 bytes: X_L, X_H, Y_L, Y_H, Z_L, Z_H
  ret = hal_i2c_write_read(I2C_BUS, BMX160_I2C_ADDR, &reg, 1, rx_buf, 6);
  if (ret != HAL_I2C_OK) {
    return ERR0;
  }

  // Combine little-endian pairs into signed 16-bit values
  raw[0] = (int16_t)((rx_buf[1] << 8) | rx_buf[0]); // X
  raw[1] = (int16_t)((rx_buf[3] << 8) | rx_buf[2]); // Y
  raw[2] = (int16_t)((rx_buf[5] << 8) | rx_buf[4]); // Z
  return NO_ERR;
}
bmx160_err_type bmx160_read_gyr_raw(int16_t *raw) {
  uint8_t reg = BMX160_GYRX_LOW_ADDR; // Start address of accel X LSB
  hal_i2c_status_t ret;

  // Read 6 bytes: X_L, X_H, Y_L, Y_H, Z_L, Z_H
  ret = hal_i2c_write_read(I2C_BUS, BMX160_I2C_ADDR, &reg, 1, rx_buf, 6);
  if (ret != HAL_I2C_OK) {
    return ERR0;
  }

  raw[0] = ((int16_t)(((int8_t)rx_buf[1]) << 8 | rx_buf[0]));
  raw[1] = ((int16_t)(((int8_t)rx_buf[3]) << 8 | rx_buf[2]));
  raw[2] = ((int16_t)(((int8_t)rx_buf[5]) << 8 | rx_buf[4]));

  return NO_ERR;
}

bmx160_err_type bmx160_read_mag_raw(int16_t *raw) {
  uint8_t reg = BMX160_MAGX_LOW_ADDR; // Start address of accel X LSB
  hal_i2c_status_t ret;

  // Read 6 bytes: X_L, X_H, Y_L, Y_H, Z_L, Z_H
  ret = hal_i2c_write_read(I2C_BUS, BMX160_I2C_ADDR, &reg, 1, rx_buf, 6);
  if (ret != HAL_I2C_OK) {
    return ERR0;
  }

  // Combine little-endian pairs into signed 16-bit values
  raw[0] = (int16_t)((rx_buf[1] << 8) | rx_buf[0]); // X
  raw[1] = (int16_t)((rx_buf[3] << 8) | rx_buf[2]); // Y
  raw[2] = (int16_t)((rx_buf[5] << 8) | rx_buf[4]); // Z
  return NO_ERR;
}

bmx160_err_type bmx160_read_all_raw(bmx160_all_reading_t *raw) {
  int16_t data[3];
  bmx160_err_type status = bmx160_read_acc_raw(data);
  if (status != NO_ERR)
    return status;
  raw->raw.acc[0] = data[0];
  raw->raw.acc[1] = data[1];
  raw->raw.acc[2] = data[2];

  status = bmx160_read_gyr_raw(data);
  if (status != NO_ERR)
    return status;
  raw->raw.gyr[0] = data[0];
  raw->raw.gyr[1] = data[1];
  raw->raw.gyr[2] = data[2];

  status = bmx160_read_mag_raw(data);
  if (status != NO_ERR)
    return status;
  raw->raw.mag[0] = data[0];
  raw->raw.mag[1] = data[1];
  raw->raw.mag[2] = data[2];

  return NO_ERR;
}

bmx160_err_type bmx160_read_acc_mps2(float *data) {
  int16_t raw[3];
  bmx160_err_type err = bmx160_read_acc_raw(raw); // <-- raw must be [3]
  if (err != NO_ERR)
    return err;

  data[0] = bmx160_raw_acc_to_mps2(raw[0]);
  data[1] = bmx160_raw_acc_to_mps2(raw[1]);
  data[2] = bmx160_raw_acc_to_mps2(raw[2]);

  return NO_ERR;
}

bmx160_err_type bmx160_read_gyr_dps(float *data) {
  int16_t raw[3];
  bmx160_err_type err = bmx160_read_gyr_raw(raw);
  if (err != NO_ERR)
    return err;

  data[0] = bmx160_raw_gyr_to_dps(raw[0]);
  data[1] = bmx160_raw_gyr_to_dps(raw[1]);
  data[2] = bmx160_raw_gyr_to_dps(raw[2]);

  return NO_ERR;
}

bmx160_err_type bmx160_read_mag_uT(float *data) {
  int16_t raw[3];
  bmx160_err_type err = bmx160_read_mag_raw(raw);
  if (err != NO_ERR)
    return err;

  data[0] = bmx160_raw_mag_to_uT(raw[0]);
  data[1] = bmx160_raw_mag_to_uT(raw[1]);
  data[2] = bmx160_raw_mag_to_uT(raw[2]);

  return NO_ERR;
}

bmx160_err_type bmx160_read_all_converted(bmx160_all_reading_t *data) {
  float calculated[3];
  bmx160_err_type err = bmx160_read_acc_mps2(calculated);
  if (err != NO_ERR)
    return err;
  data->converted.acc[0] = calculated[0];
  data->converted.acc[1] = calculated[1];
  data->converted.acc[2] = calculated[2];

  err = bmx160_read_gyr_dps(calculated);
  if (err != NO_ERR)
    return err;
  data->converted.gyr[0] = calculated[0];
  data->converted.gyr[1] = calculated[1];
  data->converted.gyr[2] = calculated[2];

  err = bmx160_read_mag_uT(calculated);
  if (err != NO_ERR)
    return err;
  data->converted.mag[0] = calculated[0];
  data->converted.mag[1] = calculated[1];
  data->converted.mag[2] = calculated[2];
  return NO_ERR;
}

#define GET_ACC_US(x) ((x >> 7U) & 1U)     // bit <7>
#define GET_ACC_BWP(x) ((x >> 4U) & 7U)    // bits <6:4>
#define GET_ACC_ODR(x) ((x >> 0U) & 15U)   // bits <3:0>
#define GET_ACC_RANGE(x) ((x >> 0U) & 15U) // bits <3:0>

#define GET_GYR_ODR(x) ((x >> 0U) & 15U)  // bits <3:0>
#define GET_GYR_BWP(x) ((x >> 4U) & 3U)   // bits <5:4>
#define GET_GYR_RANGE(x) ((x >> 0U) & 7U) // bits <2:0>

#define GET_MAG_ODR(x) ((x >> 0U) & 15U) // bits <3:0>

bmx160_err_type bmx160_read_acc_config(bmx160_config_t *config) {
  // Reading ACC conf
  tx_buf[0] = BMX160_ACC_CONF_ADDR;
  if (hal_i2c_write_read(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 1, rx_buf, 1) ==
      HAL_I2C_OK) {
    config->bmx160_acc_us = GET_ACC_US(rx_buf[0]);
    config->bmx160_acc_bwp = GET_ACC_BWP(rx_buf[0]);
    config->bmx160_acc_odr = GET_ACC_ODR(rx_buf[0]);
  } else
    return ERR0;

  tx_buf[0] = BMX160_ACC_RANGE_ADDR;
  if (hal_i2c_write_read(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 1, rx_buf, 1) ==
      HAL_I2C_OK) {
    config->bmx160_acc_range = GET_ACC_RANGE(rx_buf[0]);
  } else
    return ERR0;
  return NO_ERR;
}

bmx160_err_type bmx160_read_gyr_config(bmx160_config_t *config) {
  // Reading GYR conf
  tx_buf[0] = BMX160_GYR_CONF_ADDR;
  if (hal_i2c_write_read(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 1, rx_buf, 1) ==
      HAL_I2C_OK) {
    config->bmx160_gyr_bwp = GET_GYR_BWP(rx_buf[0]);
    config->bmx160_gyr_odr = GET_GYR_ODR(rx_buf[0]);
  } else
    return ERR0;

  tx_buf[0] = BMX160_GYR_RANGE_ADDR;
  if (hal_i2c_write_read(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 1, rx_buf, 1) ==
      HAL_I2C_OK) {
    config->bmx160_gyr_range = GET_GYR_RANGE(rx_buf[0]);
  } else
    return ERR0;
  return NO_ERR;
}

bmx160_err_type bmx160_read_mag_config(bmx160_config_t *config) {
  // Reading MAG conf
  tx_buf[0] = BMX160_MAG_CONF_ADDR;
  if (hal_i2c_write_read(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 1, rx_buf, 1) ==
      HAL_I2C_OK) {
    config->bmx160_mag_odr = GET_MAG_ODR(rx_buf[0]);
  } else
    return ERR0;
  return NO_ERR;
}

bmx160_err_type bmx160_read_config(bmx160_config_t *config) {
  if (bmx160_read_acc_config(config) != NO_ERR)
    return ERR0;

  if (bmx160_read_gyr_config(config) != NO_ERR)
    return ERR0;

  if (bmx160_read_mag_config(config) != NO_ERR)
    return ERR0;
  bmx160_set_current_config(config);
  return NO_ERR;
}

bmx160_err_type bmx160_write_config(bmx160_config_t *config) {
  if (bmx160_write_acc_config(config) != NO_ERR)
    return ERR0;
  if (bmx160_write_gyr_config(config) != NO_ERR)
    return ERR0;
  if (bmx160_write_mag_config(config) != NO_ERR)
    return ERR0;
  bmx160_set_current_config(config);

  // Pre-calculate scales
  float g_range = bmx160_range_code_to_g(config->bmx160_acc_range);
  acc_scale = g_range * 9.80665f / 32768.0f;

  float dps_range = bmx160_range_code_to_dps(config->bmx160_gyr_range);
  gyr_scale = dps_range / 32768.0f;

  return NO_ERR;
}

static uint8_t bmx160_get_acc_conf(bmx160_config_t *config) {
  uint8_t val =
      (((config->bmx160_acc_us & 1U) << 7U) |
       ((config->bmx160_acc_bwp & 7U) << 4U) | (config->bmx160_acc_odr & 15U));
  return val;
}

static uint8_t bmx160_get_acc_range(bmx160_config_t *config) {
  uint8_t val = (config->bmx160_acc_range & 15U);
  return val;
}

static uint8_t bmx160_get_gyr_conf(bmx160_config_t *config) {
  uint8_t val =
      (((config->bmx160_gyr_bwp & 3U) << 4U) | (config->bmx160_gyr_odr & 15U));
  return val;
}

static uint8_t bmx160_get_gyr_range(bmx160_config_t *config) {
  uint8_t val = (config->bmx160_gyr_range & 7U);
  return val;
}

static uint8_t bmx160_get_mag_conf(bmx160_config_t *config) {
  uint8_t val = (config->bmx160_mag_odr & 15U);
  return val;
}

bmx160_err_type bmx160_write_acc_config(bmx160_config_t *config) {
  uint8_t val;

  // --- ACC_CONF ---
  val = bmx160_get_acc_conf(config);
  tx_buf[0] = BMX160_ACC_CONF_ADDR;
  tx_buf[1] = val;
  if (hal_i2c_write(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 2) != HAL_I2C_OK)
    return ERR0;

  // --- ACC_RANGE ---
  val = bmx160_get_acc_range(config);
  tx_buf[0] = BMX160_ACC_RANGE_ADDR;
  tx_buf[1] = val;
  if (hal_i2c_write(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 2) != HAL_I2C_OK)
    return ERR0;

  return NO_ERR;
}

bmx160_err_type bmx160_write_gyr_config(bmx160_config_t *config) {
  uint8_t val;

  // --- GYR_CONF ---
  val = bmx160_get_gyr_conf(config);
  tx_buf[0] = BMX160_GYR_CONF_ADDR;
  tx_buf[1] = val;
  if (hal_i2c_write(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 2) != HAL_I2C_OK)
    return ERR0;

  // --- GYR_RANGE ---
  val = bmx160_get_gyr_range(config);
  tx_buf[0] = BMX160_GYR_RANGE_ADDR;
  tx_buf[1] = val;
  if (hal_i2c_write(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 2) != HAL_I2C_OK)
    return ERR0;

  return NO_ERR;
}

bmx160_err_type bmx160_write_mag_config(bmx160_config_t *config) {
  uint8_t val;

  // --- MAG_CONF ---
  val = bmx160_get_mag_conf(config);
  tx_buf[0] = BMX160_MAG_CONF_ADDR;
  tx_buf[1] = val;
  if (hal_i2c_write(I2C_BUS, BMX160_I2C_ADDR, tx_buf, 2) != HAL_I2C_OK)
    return ERR0;

  return NO_ERR;
}
static float bmx160_range_code_to_g(uint8_t range_code) {
  switch (range_code) {
  case 3:
    return 2.0f; // ±2g
  case 5:
    return 4.0f; // ±4g
  case 8:
    return 8.0f; // ±8g
  case 12:
    return 16.0f; // ±16g
  default:
    return 2.0f; // fallback to ±2g
  }
}

float bmx160_raw_acc_to_mps2(int16_t raw) {
  // Convert raw → g → m/s² using pre-calculated scale
  return (float)raw * acc_scale;
}
static float bmx160_range_code_to_dps(uint8_t range_code) {
  switch (range_code) {
  case 0:
    return 2000.0f;
  case 1:
    return 1000.0f;
  case 2:
    return 500.0f;
  case 3:
    return 250.0f;
  case 4:
    return 125.0f;
  default:
    return 2000.0f; // fallback
  }
}

float bmx160_raw_gyr_to_dps(int16_t raw) {
  // Convert raw → dps using pre-calculated scale
  return (float)raw * gyr_scale;
}

float bmx160_raw_mag_to_uT(int16_t raw) {
  // Deprecated: use axis-specific compensation
  return (float)raw;
}

bmx160_config_t bmx160_get_current_config(void) { return bmx160_cfg; }
void bmx160_set_current_config(bmx160_config_t *cfg) { bmx160_cfg = *cfg; }

extern uint32_t bmx160_task_id;

// Run from ISR
void wake_imu_read_task(void) { task_unblock(bmx160_task_id); }

void bmx160_initiate_read(void *args) {
  // Trigger DMA read for 30 bytes (MagX_LSB 0x04 to Temp_MSB 0x21)
  dma_config_t i2c_dma_cfg = {
      .controller = DMA_CONTROLLER_1,
      .stream = 0,
      .channel = 1,
      .direction = DMA_DIR_P2M,
      .src_addr = (uint32_t)(0x40005400 + 0x10), // I2C1_BASE + DR Offset
      .dst_addr = (uint32_t)_bmx_dma_rx_buffer,
      .data_count = 30,
      .src_inc = 0,
      .dst_inc = 1,
      .data_width = DMA_DATA_WIDTH_8,
      .priority = DMA_PRIORITY_VERY_HIGH,
      .circular = 0};
  while (1) {
    hal_i2c_read_regs_dma(I2C1, BMX160_I2C_ADDR, 0x04, &i2c_dma_cfg,
                          bmx160_dma_callback);
    task_block();
  }
}

void bmx160_dma_callback(void) {
  // 1. Extract mag (0-5)
  // X/Y are 13-bit, Z is 15-bit. Status bits are in the LSB.
  int16_t mx = (int16_t)(((uint16_t)_bmx_dma_rx_buffer[1] << 8) |
                         (_bmx_dma_rx_buffer[0] & 0xF8)) >>
               3;
  int16_t my = (int16_t)(((uint16_t)_bmx_dma_rx_buffer[3] << 8) |
                         (_bmx_dma_rx_buffer[2] & 0xF8)) >>
               3;
  int16_t mz = (int16_t)(((uint16_t)_bmx_dma_rx_buffer[5] << 8) |
                         (_bmx_dma_rx_buffer[4] & 0xFE)) >>
               1;
  // RHALL is at bytes 6, 7
  uint16_t rhall = (uint16_t)(((uint16_t)_bmx_dma_rx_buffer[7] << 8) |
                              (_bmx_dma_rx_buffer[6] & 0xFE)) >>
                   2;

  // 2. Extract gyr (8-13)
  int16_t gx =
      (int16_t)(((uint16_t)_bmx_dma_rx_buffer[9] << 8) | _bmx_dma_rx_buffer[8]);
  int16_t gy = (int16_t)(((uint16_t)_bmx_dma_rx_buffer[11] << 8) |
                         _bmx_dma_rx_buffer[10]);
  int16_t gz = (int16_t)(((uint16_t)_bmx_dma_rx_buffer[13] << 8) |
                         _bmx_dma_rx_buffer[12]);

  // 3. Extract acc (14-19)
  int16_t ax = (int16_t)(((uint16_t)_bmx_dma_rx_buffer[15] << 8) |
                         _bmx_dma_rx_buffer[14]);
  int16_t ay = (int16_t)(((uint16_t)_bmx_dma_rx_buffer[17] << 8) |
                         _bmx_dma_rx_buffer[16]);
  int16_t az = (int16_t)(((uint16_t)_bmx_dma_rx_buffer[19] << 8) |
                         _bmx_dma_rx_buffer[18]);

  // 4. Extract temperature (28-29) - Register 0x20, 0x21
  int16_t raw_temp = (int16_t)(((uint16_t)_bmx_dma_rx_buffer[29] << 8) |
                               _bmx_dma_rx_buffer[28]);

  // Store raw results
  _bmx_data.raw.acc[0] = ax;
  _bmx_data.raw.acc[1] = ay;
  _bmx_data.raw.acc[2] = az;
  _bmx_data.raw.gyr[0] = gx;
  _bmx_data.raw.gyr[1] = gy;
  _bmx_data.raw.gyr[2] = gz;
  _bmx_data.raw.mag[0] = mx;
  _bmx_data.raw.mag[1] = my;
  _bmx_data.raw.mag[2] = mz;
  _bmx_data.raw.rhall = rhall;
  _bmx_data.raw.temp = raw_temp;

  // Convert to units (for local attitude fusion and telemetry)
  _bmx_data.converted.acc[0] = bmx160_raw_acc_to_mps2(ax);
  _bmx_data.converted.acc[1] = bmx160_raw_acc_to_mps2(ay);
  _bmx_data.converted.acc[2] = bmx160_raw_acc_to_mps2(az);

  _bmx_data.converted.gyr[0] = bmx160_raw_gyr_to_dps(gx);
  _bmx_data.converted.gyr[1] = bmx160_raw_gyr_to_dps(gy);
  _bmx_data.converted.gyr[2] = bmx160_raw_gyr_to_dps(gz);

  _bmx_data.converted.mag[0] = bmm150_compensate_x(mx, rhall);
  _bmx_data.converted.mag[1] = bmm150_compensate_y(my, rhall);
  _bmx_data.converted.mag[2] = bmm150_compensate_z(mz, rhall);

  bmx160_convert_raw_temp_to_celcius(raw_temp, &_bmx_data.converted.temp);

  // Apply LPF to converted values
  for (int i = 0; i < 3; i++) {
    _bmx_data.converted.acc[i] =
        lpf_apply(&acc_lpf[i], _bmx_data.converted.acc[i]);
    _bmx_data.converted.gyr[i] =
        lpf_apply(&gyr_lpf[i], _bmx_data.converted.gyr[i]);
  }

  // Push to ring buffer for 100Hz averaging (now with converted and filtered
  // values)
  imu_buffer_push(&_bmx_data);

  // Sensor Fusion
  // 1kHz sampling rate (from main.c registration)
  const float dt = 0.001f;
  if (SF_FILTER_USED == SF_MAHONY) {
    m_mahony_filter(_bmx_data.converted.acc[0], _bmx_data.converted.acc[1],
                    _bmx_data.converted.acc[2], _bmx_data.converted.gyr[0],
                    _bmx_data.converted.gyr[1], _bmx_data.converted.gyr[2],
                    _bmx_data.converted.mag[0], _bmx_data.converted.mag[1],
                    _bmx_data.converted.mag[2], dt, &_bmx_orientation);
  } else {
    m_complementary_filter(
        _bmx_data.converted.acc[0], _bmx_data.converted.acc[1],
        _bmx_data.converted.acc[2], _bmx_data.converted.gyr[0],
        _bmx_data.converted.gyr[1], _bmx_data.converted.gyr[2],
        _bmx_data.converted.mag[0], _bmx_data.converted.mag[1],
        _bmx_data.converted.mag[2], dt, &_bmx_orientation);
  }
}

void bmx160_get_attitude(attitude_t *att) {
  if (att != NULL) {
    *att = _bmx_orientation;
  }
}
