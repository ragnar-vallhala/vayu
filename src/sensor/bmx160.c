#include "sensor/bmx160.h"
#include "comm/serializer.h"
#include "drivers/i2c_manager.h"
#include "ipc.h"
#include "maths/lpf.h"
#include "maths/sensor_fusion.h"
#include "memory.h"
#include "sensor/imu_buffer.h"
#include "sys/state.h"
#include "task.h"
#include "utils.h"
#include "utils/utils.h"
#include "vaios.h"
#include "variables.h"
#include "vayu_tasks.h"
#include "vfs.h"
#include <math.h>
#include <stdint.h>

extern float sqrtf(float x);
extern float fabsf(float x);

#define FABS_F(x) ((x) < 0.0f ? -(x) : (x))
#define SQRT_F(x) sqrtf(x)
static int in_init = 1;
#define IS_FINITE(x) ((x) - (x) == 0.0f)
extern channel_t g_telemetry_channel;
uint8_t tx_buf[2];
uint8_t rx_buf[14]; // Increased for safer multi-byte reads

static float last_mag[3] = {0}; // last valid mag readings
static int stable_count = 0;    // is platform stable

static MutexHandle_t bmx160_attitude_mutex; // Mutex for attitude data
static SemaphoreHandle_t bmx160_dma_sema;   // Semaphore for DMA completion
static SemaphoreHandle_t bmx160_timer_sema; // Semaphore for Timer wake-up

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
static bmx160_calibration_t bmx160_calib = {.acc_offset = {0.0f, 0.0f, 0.0f},
                                            .acc_scale = {1.0f, 1.0f, 1.0f},
                                            .gyr_offset = {0.0f, 0.0f, 0.0f},
                                            .mag_offset = {0.0f, 0.0f, 0.0f},
                                            .mag_scale = {1.0f, 1.0f, 1.0f}};

// LPFs for sensors
static lpf_t acc_lpf[3];
static lpf_t gyr_lpf[3];

static bmm150_trim_data_t _mag_trim;
static int i2c_error_count = 0;

// Helper functions
static bmx160_err_type bmx160_set_mag_conf();
static bmx160_err_type bmx160_convert_raw_temp_to_celcius(int16_t raw_temp,
                                                          float *celcius);
static float bmx160_range_code_to_g(uint8_t range_code);
static float bmx160_range_code_to_dps(uint8_t range_code);
static void bmx160_process_data(void);
static bmx160_err_type bmx160_wait_mag_manual_op(void);

static bmx160_err_type bmx160_wait_mag_manual_op(void) {
  uint8_t status_reg = 0x1B; // STATUS register
  uint8_t status;
  int timeout = 100;
  do {
    if (i2c_manager_write_read(BMX160_I2C_ADDR, &status_reg, 1, &status, 1) !=
        HAL_I2C_OK)
      return ERR0;
    if (!(status & 0x04)) // mag_man_op bit clear = operation done
      return NO_ERR;
    v_delay(1);
  } while (--timeout > 0);

  // Phase 2: wait for mag_man_op to go LOW (operation complete)
  timeout = 100;
  do {
    if (i2c_manager_write_read(BMX160_I2C_ADDR, &status_reg, 1, &status, 1) !=
        HAL_I2C_OK)
      return ERR0;
    if (!(status & 0x04))
      return NO_ERR;
    v_delay(1);
  } while (--timeout > 0);

  vayu_log("BMX160: Mag manual op timeout!");
  return ERR1;
}

static bmx160_err_type bmx160_verify_pmu(uint8_t mask, uint8_t expected) {
  uint8_t reg = BMX160_PMU_STAT_ADDR;

  if (i2c_manager_write_read(BMX160_I2C_ADDR, &reg, 1, rx_buf, 1) !=
      HAL_I2C_OK) {

    return ERR0;
  }
  uint8_t stat = rx_buf[0];

  if ((stat & mask) == expected) {
    return NO_ERR;
  }
  return ERR1;
}

hal_i2c_status_t bmx160_init(void) {

  // Create I2C bus semaphore early. Ensure it starts "given"
  in_init = 1; // Explicitly set it here as well

  // Read calibration from sd card
  vfs_fd_t file = vfs_open(CALIBRATION_FILE_PATH, VFS_O_RDONLY);
  if (file < 0) {
    vayu_log("[CALIB] Failed to open calibration file.");
  } else {
    vfs_read(file, &bmx160_calib, sizeof(bmx160_calibration_t));
    vfs_close(file);
  }
  // Create attitude mutex
  if (bmx160_attitude_mutex == NULL) {
    bmx160_attitude_mutex = v_mutex_create();
  }
  // Reading PTR
  // 1. Verify Chip ID
  uint16_t chip_id = bmx160_get_chip_id();
  if (chip_id != BMX160_CHIP_ID) {
    return HAL_I2C_ERR_REINIT;
  }

  // 2. Soft Reset to ensure clean state
  tx_buf[0] = BMX160_CMD_ADDR;
  tx_buf[1] = 0xB6; // softreset command
  i2c_manager_write(BMX160_I2C_ADDR, tx_buf, 2);
  v_delay(100);

  // 3. Put accelerometer into normal mode
  int retry = 5;
  while (retry--) {
    tx_buf[0] = BMX160_CMD_ADDR;
    tx_buf[1] = BMX160_CMD_ACC_NORMAL;
    i2c_manager_write(BMX160_I2C_ADDR, tx_buf, 2);
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
    i2c_manager_write(BMX160_I2C_ADDR, tx_buf, 2);
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
    i2c_manager_write(BMX160_I2C_ADDR, tx_buf, 2);
    v_delay(100);
    if (bmx160_verify_pmu(BMX160_PMU_STAT_MAG_MASK,
                          BMX160_PMU_STAT_MAG_NORMAL) == NO_ERR) {
      break;
    }
  }

  // LPF init and Scale calc BEFORE Mag init to be safe
  // Initialize LPFs
  for (int i = 0; i < 3; i++) {
    lpf_init(&acc_lpf[i], LPF_ACC_ALPHA); // Aggressive filtering for Acc
    lpf_init(&gyr_lpf[i], LPF_GYR_ALPHA); // Filter Gyro as well
  }

  // Pre-calculate scales (calculating here too just in case config write fails
  // elsewhere)
  float g_range = bmx160_range_code_to_g(bmx160_cfg.bmx160_acc_range);
  acc_scale = g_range * 9.80665f / 32768.0f;
  float dps_range = bmx160_range_code_to_dps(bmx160_cfg.bmx160_gyr_range);
  gyr_scale = dps_range / 32768.0f;

  // 6. Configure Magnetometer (BMM150 setup)
  if (bmx160_set_mag_conf() != NO_ERR) {
    return HAL_I2C_ERR_REINIT;
  }

  // Initialize orientation quaternion to identity
  _bmx_orientation.q.w = 1.0f;
  _bmx_orientation.q.x = 0.0f;
  _bmx_orientation.q.y = 0.0f;
  _bmx_orientation.q.z = 0.0f;

  // Create semaphore for DMA synchronization
  if (bmx160_dma_sema == NULL) {
    bmx160_dma_sema = v_semaphore_create_binary();
  }
  if (bmx160_dma_sema == NULL) {
    return HAL_I2C_ERR_REINIT;
  }

  // Create semaphore for Timer wake-up
  if (bmx160_timer_sema == NULL) {
    bmx160_timer_sema = v_semaphore_create_binary();
  }
  if (bmx160_timer_sema == NULL) {
    return HAL_I2C_ERR_REINIT;
  }

  in_init = 0; // Success! Disable blocking bypass
  return HAL_I2C_OK;
}

static bmx160_err_type bmx160_write_bmm150_reg(uint8_t reg, uint8_t data) {
  tx_buf[0] = BMX160_MAG_IF_3_DATA_ADDR;
  tx_buf[1] = data;

  if (i2c_manager_write(BMX160_I2C_ADDR, tx_buf, 2) != HAL_I2C_OK) {
    return ERR0;
  }

  v_delay(2); // Wait for BMX -> BMM write

  tx_buf[0] = BMX160_MAG_IF_2_REG_ADDR;
  tx_buf[1] = reg;

  if (i2c_manager_write(BMX160_I2C_ADDR, tx_buf, 2) != HAL_I2C_OK) {
    return ERR0;
  }

  if (bmx160_wait_mag_manual_op() != NO_ERR)
    return ERR1;

  return NO_ERR;
}

static bmx160_err_type bmx160_read_bmm150_reg(uint8_t reg, uint8_t *data) {
  tx_buf[0] = BMX160_MAG_IF_1_READ_ADDR;
  tx_buf[1] = reg;

  if (i2c_manager_write(BMX160_I2C_ADDR, tx_buf, 2) != HAL_I2C_OK) {
    return ERR0;
  }

  if (bmx160_wait_mag_manual_op() != NO_ERR)
    return ERR1;

  uint8_t read_reg = 0x04; // MAG_X_LSB in BMX160 is where IF data appears

  if (i2c_manager_write_read(BMX160_I2C_ADDR, &read_reg, 1, data, 1) !=
      HAL_I2C_OK) {
    return ERR0;
  }
  return NO_ERR;
}

static void bmx160_read_mag_trim_data(void) {
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
}

static bmx160_err_type bmx160_set_mag_conf() {
  // 1. Route secondary I2C interface to Magnetometer (0x6B = 0x20)
  tx_buf[0] = BMX160_IF_CONF_ADDR;
  tx_buf[1] = 0x20;
  i2c_manager_write(BMX160_I2C_ADDR, tx_buf, 2);
  v_delay(1);

  // 2. Enter Manual Mode to allow BMM150 writes/reads
  tx_buf[0] = BMX160_MAG_IF_0_CONF_ADDR;
  tx_buf[1] = 0x80; // manual_en = 1
  i2c_manager_write(BMX160_I2C_ADDR, tx_buf, 2);
  v_delay(1);

  // 3. Take BMM150 out of suspend (Manual write 0x01 to BMM150 Reg 0x4B)
  bmx160_write_bmm150_reg(0x4B, 0x01);
  v_delay(100); // Wait for Power-up

  // Read dummy
  uint8_t dummy = 0;
  bmx160_read_bmm150_reg(0x5D, &dummy);
  v_delay(10);
  // 4. Read Trim Data (Manual mode is already active)
  bmx160_read_mag_trim_data();

  // 4. Configure BMM150 (Regular Preset: 9 reps XY, 15 reps Z)
  bmx160_write_bmm150_reg(0x51, 0x04);
  bmx160_write_bmm150_reg(0x52, 0x0E);

  // 5. Prepare for Data Mode
  // Set BMM150 to Forced Mode (Reg 0x4C = 0x02)
  bmx160_write_bmm150_reg(0x4C, 0x00);

  // 6. Set Mag Read Address to 0x42 (Data X LSB)
  tx_buf[0] = BMX160_MAG_IF_1_READ_ADDR;
  tx_buf[1] = 0x42;
  i2c_manager_write(BMX160_I2C_ADDR, tx_buf, 2);
  v_delay(1);

  // 7. Configure ODR (0x44 = 0x08 for 100Hz)
  tx_buf[0] = BMX160_MAG_CONF_ADDR;
  tx_buf[1] = 0x08;
  i2c_manager_write(BMX160_I2C_ADDR, tx_buf, 2);
  v_delay(1);

  // 8. Verify BMM150 Chip ID (Register 0x40) - MUST BE DONE IN MANUAL MODE
  uint8_t mag_id = 0;
  bmx160_read_bmm150_reg(0x40, &mag_id);
  if (mag_id != 0x32) {
    return ERR0;
  }

  // 9. Enable Auto-mode (manual_en = 0, burst_read = 8 bytes)
  tx_buf[0] = BMX160_MAG_IF_0_CONF_ADDR;
  tx_buf[1] = 0x03;
  i2c_manager_write(BMX160_I2C_ADDR, tx_buf, 2);
  v_delay(10);

  return NO_ERR;
}

uint16_t bmx160_get_chip_id(void) {
  uint8_t reg = BMX160_CHIP_ID_ADDR;
  hal_i2c_status_t ret;

  ret = i2c_manager_write_read(BMX160_I2C_ADDR, &reg, 1, rx_buf, 1);
  if (ret != HAL_I2C_OK) {
    return 0xFFFF;
  }
  int16_t chip_id = (int16_t)(rx_buf[0]);
  return (uint16_t)chip_id;
}

int16_t bmx160_read_temp_raw(void) { return _bmx_data.raw.temp; }

static float bmm150_compensate_x(int16_t mag_data_x, uint16_t data_rhall) {
  if (data_rhall < 50)
    return 0.0f;
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
                                      (process_comp_x3 / 8192.0f));
      retval = process_comp_x4 + ((float)_mag_trim.dig_x1) * 8.0f;
      retval = retval / 16.0f;
    }
  } else {
    retval = 0.0f;
  }
  return retval;
}

static float bmm150_compensate_y(int16_t mag_data_y, uint16_t data_rhall) {
  if (data_rhall < 50)
    return 0.0f;
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
                                      (process_comp_y3 / 8192.0f));
      retval = process_comp_y4 + ((float)_mag_trim.dig_y1) * 8.0f;
      retval = retval / 16.0f;
    }
  } else {
    retval = 0.0f;
  }
  return retval;
}

static float bmm150_compensate_z(int16_t mag_data_z, uint16_t data_rhall) {
  if (data_rhall < 50)
    return 0.0f;
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
  if (celcius == NULL)
    return ERR0;
  *celcius = _bmx_data.converted.temp;
  return NO_ERR;
}

bmx160_err_type bmx160_read_acc_raw(int16_t *raw) {
  if (raw == NULL)
    return ERR0;
  raw[0] = _bmx_data.raw.acc[0];
  raw[1] = _bmx_data.raw.acc[1];
  raw[2] = _bmx_data.raw.acc[2];
  return NO_ERR;
}
bmx160_err_type bmx160_read_gyr_raw(int16_t *raw) {
  if (raw == NULL)
    return ERR0;
  raw[0] = _bmx_data.raw.gyr[0];
  raw[1] = _bmx_data.raw.gyr[1];
  raw[2] = _bmx_data.raw.gyr[2];
  return NO_ERR;
}

bmx160_err_type bmx160_read_mag_raw(int16_t *raw) {
  if (raw == NULL)
    return ERR0;
  raw[0] = _bmx_data.raw.mag[0];
  raw[1] = _bmx_data.raw.mag[1];
  raw[2] = _bmx_data.raw.mag[2];
  return NO_ERR;
}

bmx160_err_type bmx160_read_all_raw(bmx160_all_reading_t *raw) {
  if (raw == NULL)
    return ERR0;
  *raw = _bmx_data;
  return NO_ERR;
}

bmx160_err_type bmx160_read_acc_mps2(float *data) {
  if (data == NULL)
    return ERR0;
  data[0] = _bmx_data.converted.acc[0];
  data[1] = _bmx_data.converted.acc[1];
  data[2] = _bmx_data.converted.acc[2];
  return NO_ERR;
}

bmx160_err_type bmx160_read_gyr_dps(float *data) {
  if (data == NULL)
    return ERR0;
  data[0] = _bmx_data.converted.gyr[0];
  data[1] = _bmx_data.converted.gyr[1];
  data[2] = _bmx_data.converted.gyr[2];
  return NO_ERR;
}

bmx160_err_type bmx160_read_mag_uT(float *data) {
  if (data == NULL)
    return ERR0;
  data[0] = _bmx_data.converted.mag[0];
  data[1] = _bmx_data.converted.mag[1];
  data[2] = _bmx_data.converted.mag[2];
  return NO_ERR;
}

bmx160_err_type bmx160_read_all_converted(bmx160_all_reading_t *data) {
  if (data == NULL)
    return ERR0;

  *data = _bmx_data;
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
  if (i2c_manager_write_read(BMX160_I2C_ADDR, tx_buf, 1, rx_buf, 1) ==
      HAL_I2C_OK) {
    config->bmx160_acc_us = GET_ACC_US(rx_buf[0]);
    config->bmx160_acc_bwp = GET_ACC_BWP(rx_buf[0]);
    config->bmx160_acc_odr = GET_ACC_ODR(rx_buf[0]);
  } else
    return ERR0;

  tx_buf[0] = BMX160_ACC_RANGE_ADDR;
  if (i2c_manager_write_read(BMX160_I2C_ADDR, tx_buf, 1, rx_buf, 1) ==
      HAL_I2C_OK) {
    config->bmx160_acc_range = GET_ACC_RANGE(rx_buf[0]);
  } else
    return ERR0;
  return NO_ERR;
}

bmx160_err_type bmx160_read_gyr_config(bmx160_config_t *config) {
  // Reading GYR conf
  tx_buf[0] = BMX160_GYR_CONF_ADDR;
  if (i2c_manager_write_read(BMX160_I2C_ADDR, tx_buf, 1, rx_buf, 1) ==
      HAL_I2C_OK) {
    config->bmx160_gyr_bwp = GET_GYR_BWP(rx_buf[0]);
    config->bmx160_gyr_odr = GET_GYR_ODR(rx_buf[0]);
  } else
    return ERR0;

  tx_buf[0] = BMX160_GYR_RANGE_ADDR;
  if (i2c_manager_write_read(BMX160_I2C_ADDR, tx_buf, 1, rx_buf, 1) ==
      HAL_I2C_OK) {
    config->bmx160_gyr_range = GET_GYR_RANGE(rx_buf[0]);
  } else
    return ERR0;
  return NO_ERR;
}

bmx160_err_type bmx160_read_mag_config(bmx160_config_t *config) {
  // Reading MAG conf
  tx_buf[0] = BMX160_MAG_CONF_ADDR;
  if (i2c_manager_write_read(BMX160_I2C_ADDR, tx_buf, 1, rx_buf, 1) ==
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
  if (i2c_manager_write(BMX160_I2C_ADDR, tx_buf, 2) != HAL_I2C_OK)
    return ERR0;

  // --- ACC_RANGE ---
  val = bmx160_get_acc_range(config);
  tx_buf[0] = BMX160_ACC_RANGE_ADDR;
  tx_buf[1] = val;
  if (i2c_manager_write(BMX160_I2C_ADDR, tx_buf, 2) != HAL_I2C_OK)
    return ERR0;

  return NO_ERR;
}

bmx160_err_type bmx160_write_gyr_config(bmx160_config_t *config) {
  uint8_t val;

  // --- GYR_CONF ---
  val = bmx160_get_gyr_conf(config);
  tx_buf[0] = BMX160_GYR_CONF_ADDR;
  tx_buf[1] = val;
  if (i2c_manager_write(BMX160_I2C_ADDR, tx_buf, 2) != HAL_I2C_OK)
    return ERR0;

  // --- GYR_RANGE ---
  val = bmx160_get_gyr_range(config);
  tx_buf[0] = BMX160_GYR_RANGE_ADDR;
  tx_buf[1] = val;
  if (i2c_manager_write(BMX160_I2C_ADDR, tx_buf, 2) != HAL_I2C_OK)
    return ERR0;

  return NO_ERR;
}

bmx160_err_type bmx160_write_mag_config(bmx160_config_t *config) {
  uint8_t val;

  // --- MAG_CONF ---
  val = bmx160_get_mag_conf(config);
  tx_buf[0] = BMX160_MAG_CONF_ADDR;
  tx_buf[1] = val;
  if (i2c_manager_write(BMX160_I2C_ADDR, tx_buf, 2) != HAL_I2C_OK)
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

// Removed: bmx160_raw_mag_to_uT was deprecated and broken.

bmx160_config_t bmx160_get_current_config(void) { return bmx160_cfg; }
void bmx160_set_current_config(bmx160_config_t *cfg) { bmx160_cfg = *cfg; }

extern uint32_t bmx160_task_id;

// Run from ISR
void wake_imu_read_task(void) {
  int higher_priority_task_woken = 0;
  v_semaphore_give_from_isr(bmx160_timer_sema, &higher_priority_task_woken);
  if (higher_priority_task_woken) {
    task_yield();
  }
}
void bmx160_initiate_read(void *args) {
  while (1) {

    // 1kHz trigger
    v_semaphore_take(bmx160_timer_sema, 1000000);
    // direct_dma_print((const uint8_t *)"Called", 6);
    // Start async read (manager handles I2C locking internally)
    hal_i2c_status_t hal_ret =
        i2c_manager_read_async(BMX160_I2C_ADDR, 0x04, 30, bmx160_dma_callback);

    if (hal_ret == HAL_I2C_OK) {

      // Wait for DMA completion
      if (v_semaphore_take(bmx160_dma_sema,
                           MS_TO_TICKS(I2C_MANAGER_DMA_TIMEOUT)) == VA_PASS) {
        i2c_error_count = 0;
        bmx160_process_data();
      } else {
        // DMA timeout
        i2c_error_count++;
      }

    } else {
      // Queue full / manager busy
      i2c_error_count++;
    }

    // Error handling
    if (i2c_error_count > 10) {
      // Optionally: trigger bus reset or manager reset later
      i2c_error_count = 0;
    }
  }
}

void bmx160_dma_callback(void *args) {
  if (args == NULL) {
    vayu_log("BMX160 DMA Callback: args is NULL");
    return;
  }
  // Copy data from manager buffer to local buffer
  v_memcpy(_bmx_dma_rx_buffer, args, 30);

  int higher_priority_task_woken = 0;

  // ONLY signal DMA completion
  v_semaphore_give_from_isr(bmx160_dma_sema, &higher_priority_task_woken);

  if (higher_priority_task_woken) {
    task_yield();
  }
}

static uint32_t _last_read_time = 0;
static uint32_t _read_count = 0;

void bmx160_process_data(void) {
  _read_count++;
  if (_read_count % 2000 == 0) {
    vayu_log("IMU data processing frequency: %f Hz",
             (1000.0f * 2000.0f) / (v_get_ticks() - _last_read_time));
    _last_read_time = v_get_ticks();
    _read_count = 0;
  }
  // static int diag_printed = 0;
  // 1. Extract mag (0-5)
  // X/Y are 13-bit, Z is 15-bit. Status bits are in the LSB.
  // We assemble as signed 16-bit and then arithmetic shift to preserve sign.
  uint8_t _is_mag_invalid = 0;
  int16_t mx =
      (int16_t)(((uint16_t)_bmx_dma_rx_buffer[1] << 8) | _bmx_dma_rx_buffer[0]);
  mx >>= 3;

  int16_t my =
      (int16_t)(((uint16_t)_bmx_dma_rx_buffer[3] << 8) | _bmx_dma_rx_buffer[2]);
  my >>= 3;

  int16_t mz =
      (int16_t)(((uint16_t)_bmx_dma_rx_buffer[5] << 8) | _bmx_dma_rx_buffer[4]);
  mz >>= 1;

  // RHALL is at bytes 6, 7
  uint16_t rhall = (uint16_t)(((uint16_t)_bmx_dma_rx_buffer[7] << 8) |
                              (_bmx_dma_rx_buffer[6] & 0xFE)) >>
                   2;
  if (rhall < 50 || rhall > 30000) {
    _is_mag_invalid = 1;
  }
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
  _bmx_data.converted.acc_raw[0] = bmx160_raw_acc_to_mps2(ax);
  _bmx_data.converted.acc_raw[1] = bmx160_raw_acc_to_mps2(ay);
  _bmx_data.converted.acc_raw[2] = bmx160_raw_acc_to_mps2(az);

  // Initial populate (will be calibrated/filtered later)
  _bmx_data.converted.acc[0] = _bmx_data.converted.acc_raw[0];
  _bmx_data.converted.acc[1] = _bmx_data.converted.acc_raw[1];
  _bmx_data.converted.acc[2] = _bmx_data.converted.acc_raw[2];

  _bmx_data.converted.gyr_raw[0] = bmx160_raw_gyr_to_dps(gx);
  _bmx_data.converted.gyr_raw[1] = bmx160_raw_gyr_to_dps(gy);
  _bmx_data.converted.gyr_raw[2] = bmx160_raw_gyr_to_dps(gz);

  _bmx_data.converted.gyr[0] = _bmx_data.converted.gyr_raw[0];
  _bmx_data.converted.gyr[1] = _bmx_data.converted.gyr_raw[1];
  _bmx_data.converted.gyr[2] = _bmx_data.converted.gyr_raw[2];

  // LPF
  for (int i = 0; i < 3; i++) {
    _bmx_data.converted.gyr[i] =
        lpf_apply(&gyr_lpf[i], _bmx_data.converted.gyr[i]);
  }

  // Apply gyro bias estimator
  float acc_mag =
      SQRT_F(_bmx_data.converted.acc[0] * _bmx_data.converted.acc[0] +
             _bmx_data.converted.acc[1] * _bmx_data.converted.acc[1] +
             _bmx_data.converted.acc[2] * _bmx_data.converted.acc[2]);

  float gyro_norm =
      SQRT_F(_bmx_data.converted.gyr[0] * _bmx_data.converted.gyr[0] +
             _bmx_data.converted.gyr[1] * _bmx_data.converted.gyr[1] +
             _bmx_data.converted.gyr[2] * _bmx_data.converted.gyr[2]);
  if (system_state_get() != SYSTEM_STATE_CALIBRATING) {
    if (FABS_F(acc_mag - 9.81f) < 0.2f && gyro_norm < 0.2f) {
      stable_count++;
    } else {
      stable_count = 0;
    }
    if (stable_count > 200) {
      for (int i = 0; i < 3; i++) {
        bmx160_calib.gyr_offset[i] =
            (1.0f - GYRO_BIAS_ALPHA) * bmx160_calib.gyr_offset[i] +
            GYRO_BIAS_ALPHA * _bmx_data.converted.gyr[i];
        if (FABS_F(bmx160_calib.gyr_offset[i]) > 5.0f) {
          bmx160_calib.gyr_offset[i] = 0;
        }
      }
    }
  }

  for (int i = 0; i < 3; i++)
    _bmx_data.converted.gyr[i] -= bmx160_calib.gyr_offset[i];

  // Align BMM150 axes to BMX160 body frame: [-Y, X, Z]

  float mag_x = -bmm150_compensate_y(my, rhall);
  float mag_y = bmm150_compensate_x(mx, rhall);
  float mag_z = bmm150_compensate_z(mz, rhall);

  // Store compensated but unscaled data for calibration
  _bmx_data.converted.mag_compensated[0] = mag_x;
  _bmx_data.converted.mag_compensated[1] = mag_y;
  _bmx_data.converted.mag_compensated[2] = mag_z;

  uint8_t mag_fusion_valid = 1;
  if (!IS_FINITE(mag_x) || !IS_FINITE(mag_y) || !IS_FINITE(mag_z) ||
      _is_mag_invalid) {
    mag_fusion_valid = 0;
  }

  _bmx_data.converted.mag[0] =
      (mag_x - bmx160_calib.mag_offset[0]) * bmx160_calib.mag_scale[0];
  _bmx_data.converted.mag[1] =
      (mag_y - bmx160_calib.mag_offset[1]) * bmx160_calib.mag_scale[1];
  _bmx_data.converted.mag[2] =
      (mag_z - bmx160_calib.mag_offset[2]) * bmx160_calib.mag_scale[2];

  // Magnitude and Disturbance Checks
  if (mag_fusion_valid) {
    float mag_norm =
        SQRT_F(_bmx_data.converted.mag[0] * _bmx_data.converted.mag[0] +
               _bmx_data.converted.mag[1] * _bmx_data.converted.mag[1] +
               _bmx_data.converted.mag[2] * _bmx_data.converted.mag[2]);

    // Check magnitude (20-80 uT expected for Earth's field)
    if (mag_norm < 20.0f || mag_norm > 80.0f) {
      mag_fusion_valid = 0;
    }

    // Disturbance detection (dot product with previous valid reading)
    if (mag_fusion_valid &&
        (last_mag[0] != 0.0f || last_mag[1] != 0.0f || last_mag[2] != 0.0f)) {
      // Calculate dot product of current reading and last valid reading
      float dot = _bmx_data.converted.mag[0] * last_mag[0] +
                  _bmx_data.converted.mag[1] * last_mag[1] +
                  _bmx_data.converted.mag[2] * last_mag[2];

      // Threshold: if cos(theta) * mag_norm_curr * mag_norm_prev is too small
      // relative to expected squared magnitude, it's a disturbance.
      // Since mag_norm is around 50uT, squared is 2500.
      // We'll use a threshold based on angle change.
      // dot / (norm_curr * norm_prev) = cos(theta)
      // For 1kHz (actually ~20Hz update), the change should be tiny.
      // A dot product less than 0.9 * norm^2 is a massive jump.
      float norm_curr_sq = mag_norm * mag_norm;
      if (dot < 0.75f * norm_curr_sq) { // Significant directional jump
        mag_fusion_valid = 0;
      }
    }
  }

  // Magnetometer Normalization for Fusion
  if (mag_fusion_valid) {
    float mag_norm =
        SQRT_F(_bmx_data.converted.mag[0] * _bmx_data.converted.mag[0] +
               _bmx_data.converted.mag[1] * _bmx_data.converted.mag[1] +
               _bmx_data.converted.mag[2] * _bmx_data.converted.mag[2]);
    if (mag_norm > 0.001f) {
      _bmx_data.converted.mag[0] /= mag_norm;
      _bmx_data.converted.mag[1] /= mag_norm;
      _bmx_data.converted.mag[2] /= mag_norm;

      // Update last_mag with the *calibrated* vector (unnormalized for next dot
      // product)
      last_mag[0] = _bmx_data.converted.mag[0] * mag_norm;
      last_mag[1] = _bmx_data.converted.mag[1] * mag_norm;
      last_mag[2] = _bmx_data.converted.mag[2] * mag_norm;
    } else {
      mag_fusion_valid = 0;
    }
  }

  if (!mag_fusion_valid) {
    // Skip magnetometer in fusion by passing zero vector to Mahony
    _bmx_data.converted.mag[0] = 0.0f;
    _bmx_data.converted.mag[1] = 0.0f;
    _bmx_data.converted.mag[2] = 0.0f;
  }
  bmx160_convert_raw_temp_to_celcius(raw_temp, &_bmx_data.converted.temp);

  // Apply LPF to accelerometer (gyro was already filtered before bias
  // correction)
  for (int i = 0; i < 3; i++) {
    // Apply calibration: (raw_converted - bias) * scale
    _bmx_data.converted.acc[i] =
        (_bmx_data.converted.acc[i] - bmx160_calib.acc_offset[i]) *
        bmx160_calib.acc_scale[i];
    _bmx_data.converted.acc[i] =
        lpf_apply(&acc_lpf[i], _bmx_data.converted.acc[i]);
  }

  // Push to ring buffer for 100Hz averaging (now with converted and filtered
  // values)
  imu_buffer_push(&_bmx_data);
  imu_distribution_queue_push(&_bmx_data);

  // Sensor Fusion
  if (system_state_get() == SYSTEM_STATE_CALIBRATING) {
    return;
  }
  // 1kHz sampling rate (from main.c registration)
  if (bmx160_attitude_mutex != NULL) {
    v_mutex_lock(bmx160_attitude_mutex, MS_TO_TICKS(5));
  }
  if (SF_FILTER_USED == SF_MAHONY) {
    m_mahony_filter(_bmx_data.converted.acc[0], _bmx_data.converted.acc[1],
                    _bmx_data.converted.acc[2], _bmx_data.converted.gyr[0],
                    _bmx_data.converted.gyr[1], _bmx_data.converted.gyr[2],
                    _bmx_data.converted.mag[0], _bmx_data.converted.mag[1],
                    _bmx_data.converted.mag[2], &_bmx_orientation);
  } else {
    m_complementary_filter(
        _bmx_data.converted.acc[0], _bmx_data.converted.acc[1],
        _bmx_data.converted.acc[2], _bmx_data.converted.gyr[0],
        _bmx_data.converted.gyr[1], _bmx_data.converted.gyr[2],
        _bmx_data.converted.mag[0], _bmx_data.converted.mag[1],
        _bmx_data.converted.mag[2], &_bmx_orientation);
  }
  if (bmx160_attitude_mutex != NULL) {
    v_mutex_unlock(bmx160_attitude_mutex);
  }
}

void bmx160_get_attitude(attitude_t *att) {
  if (att != NULL && bmx160_attitude_mutex != NULL) {
    v_mutex_lock(bmx160_attitude_mutex,
                 MS_TO_TICKS(I2C_MANAGER_SEMAPHORE_TIMEOUT));
    *att = _bmx_orientation;
    v_mutex_unlock(bmx160_attitude_mutex);
  }
}

static int wait_for_orientation(calib_update_type_t orient, float *accel_out) {
  vayu_log("[CALIB] Waiting for orientation: %d", orient);

  // Send instruction to GCS
  uint8_t payload[7];
  payload[0] = SYSTEM_ORIGIN_CALIBRATION;
  payload[1] = 0x01;
  payload[2] = (uint8_t)orient;

  float zero = 0.0f;
  v_memcpy(&payload[3], &zero, 4);
  send_packet(&g_telemetry_channel, PACKET_TYPE_SYSTEM_STATUS, payload, 7);

  v_delay(CALIBRATION_WAIT_USER_TIME_PRE_CALIBRATION);

  const int target_samples = CALIBRATION_SAMPLE_COUNT;

  float sum[3] = {0.0f, 0.0f, 0.0f};
  int count = 0;

  const float g = 9.81f;
  const float thr = 3.0f;

  while (count < target_samples) {
    float raw[3];

    // Use DMA data from _bmx_data.converted.acc_raw
    raw[0] = _bmx_data.converted.acc_raw[0];
    raw[1] = _bmx_data.converted.acc_raw[1];
    raw[2] = _bmx_data.converted.acc_raw[2];

    int match = 0;

    switch (orient) {
    case CALIB_UPDATE_UPRIGHT:
      match = (raw[2] > g - thr);
      break;

    case CALIB_UPDATE_UPSIDE_DOWN:
      match = (raw[2] < -g + thr);
      break;

    case CALIB_UPDATE_NOSE_UP:
      match = (raw[0] > g - thr);
      break;

    case CALIB_UPDATE_NOSE_DOWN:
      match = (raw[0] < -g + thr);
      break;

    case CALIB_UPDATE_RIGHT_DOWN:
      match = (raw[1] > g - thr);
      break;

    case CALIB_UPDATE_LEFT_DOWN:
      match = (raw[1] < -g + thr);
      break;

    default:
      match = 0;
      break;
    }

    if (match) {
      // Accumulate
      sum[0] += raw[0];
      sum[1] += raw[1];
      sum[2] += raw[2];
      count++;

      // Progress update (every 5%)
      if (count % (target_samples / 20) == 0) {
        payload[2] = CALIB_UPDATE_PROGRESS;
        float progress = (100.0f * count) / target_samples;
        v_memcpy(&payload[3], &progress, 4);

        send_packet(&g_telemetry_channel, PACKET_TYPE_SYSTEM_STATUS, payload,
                    7);
      }

    } else {
      // Reset if orientation disturbed
      count = 0;
      sum[0] = 0.0f;
      sum[1] = 0.0f;
      sum[2] = 0.0f;
    }
    v_delay(20);
  }

  // Compute average
  float inv = 1.0f / (float)target_samples;

  accel_out[0] = sum[0] * inv;
  accel_out[1] = sum[1] * inv;
  accel_out[2] = sum[2] * inv;

  vayu_log("[CALIB] Orientation %d done: %.3f %.3f %.3f", orient, accel_out[0],
           accel_out[1], accel_out[2]);

  return 1;
}

void calibration_task(void *args) {
  calibration_args_t *cal_args = (calibration_args_t *)args;
  float imu_id = 1.0f; // Default to Accel if nothing specified
  if (cal_args)
    imu_id = cal_args->imu_id;

  system_state_set(SYSTEM_STATE_CALIBRATING);
  v_delay(500);
  vayu_log("[CALIB] IMU ID: %.1f, Type: %.1f", imu_id, cal_args->type);
  if (imu_id == 1.0f) { // ACCEL
    calib_update_type_t orients[] = {
        CALIB_UPDATE_UPRIGHT,    CALIB_UPDATE_UPSIDE_DOWN,
        CALIB_UPDATE_NOSE_UP,    CALIB_UPDATE_NOSE_DOWN,
        CALIB_UPDATE_RIGHT_DOWN, CALIB_UPDATE_LEFT_DOWN};
    float averages[6][3];

    for (int i = 0; i < 6; i++) {
      wait_for_orientation(orients[i], averages[i]);
      vayu_log("[CALIB] Pose %d recorded.", i);
      v_delay(
          CALIBRATION_WAIT_USER_TIME_PRE_CALIBRATION); // Wait for user to move
    }

    // Calibration calculation
    if (cal_args->type == 1.0f) { // FULL CALIBRATION (Bias + Scale)
      vayu_log("[CALIB] Calculating Full Accel Calibration...");

      // X-axis: Nose Up (2) and Nose Down (3)
      float max_x = averages[2][0];
      float min_x = averages[3][0];
      bmx160_calib.acc_offset[0] = (max_x + min_x) / 2.0f;
      bmx160_calib.acc_scale[0] = (2.0f * 9.80665f) / (max_x - min_x);

      // Y-axis: Right Down (4) and Left Down (5)
      float max_y = averages[4][1];
      float min_y = averages[5][1];
      bmx160_calib.acc_offset[1] = (max_y + min_y) / 2.0f;
      bmx160_calib.acc_scale[1] = (2.0f * 9.80665f) / (max_y - min_y);

      // Z-axis: Upright (0) and Upside Down (1)
      float max_z = averages[0][2];
      float min_z = averages[1][2];
      bmx160_calib.acc_offset[2] = (max_z + min_z) / 2.0f;
      bmx160_calib.acc_scale[2] = (2.0f * 9.80665f) / (max_z - min_z);

      vayu_log("[CALIB] Accel Bias: %.3f, %.3f, %.3f",
               bmx160_calib.acc_offset[0], bmx160_calib.acc_offset[1],
               bmx160_calib.acc_offset[2]);
      vayu_log("[CALIB] Accel Scale: %.3f, %.3f, %.3f",
               bmx160_calib.acc_scale[0], bmx160_calib.acc_scale[1],
               bmx160_calib.acc_scale[2]);
    } else { // BIAS ONLY
      vayu_log("[CALIB] Calculating Bias-Only Accel Calibration...");
      bmx160_calib.acc_offset[0] = (averages[2][0] + averages[3][0]) / 2.0f;
      bmx160_calib.acc_offset[1] = (averages[4][1] + averages[5][1]) / 2.0f;
      bmx160_calib.acc_offset[2] = (averages[0][2] + averages[1][2]) / 2.0f;

      // Keep scales as they are (default 1.0)
      vayu_log("[CALIB] Accel biases: %.3f, %.3f, %.3f",
               bmx160_calib.acc_offset[0], bmx160_calib.acc_offset[1],
               bmx160_calib.acc_offset[2]);
    }

  } else if (imu_id == 2.0f) {    // GYRO
    if (cal_args->type == 1.0f) { // FULL CALIBRATION (6-point Bias Check)
      vayu_log("[CALIB] Starting Full Gyro Calibration (6-point bias)...");
      calib_update_type_t orients[] = {
          CALIB_UPDATE_UPRIGHT,    CALIB_UPDATE_UPSIDE_DOWN,
          CALIB_UPDATE_NOSE_UP,    CALIB_UPDATE_NOSE_DOWN,
          CALIB_UPDATE_RIGHT_DOWN, CALIB_UPDATE_LEFT_DOWN};

      float gsum[3] = {0, 0, 0};
      float avg_buf[3];

      for (int i = 0; i < 6; i++) {
        wait_for_orientation(orients[i], avg_buf);
        vayu_log("[CALIB] Gyro orientation %d recorded.", i);

        // Collect 200 samples for bias in this orientation
        for (int s = 0; s < 200; s++) {
          gsum[0] += _bmx_data.converted.gyr_raw[0];
          gsum[1] += _bmx_data.converted.gyr_raw[1];
          gsum[2] += _bmx_data.converted.gyr_raw[2];
          v_delay(5);
        }
        v_delay(500);
      }

      // Average across all 6 orientations (1200 samples total)
      bmx160_calib.gyr_offset[0] = gsum[0] / 1200.0f;
      bmx160_calib.gyr_offset[1] = gsum[1] / 1200.0f;
      bmx160_calib.gyr_offset[2] = gsum[2] / 1200.0f;

      vayu_log("[CALIB] Gyro Final Bias: %.3f, %.3f, %.3f",
               bmx160_calib.gyr_offset[0], bmx160_calib.gyr_offset[1],
               bmx160_calib.gyr_offset[2]);

    } else { // BIAS ONLY (Stationary Upright)
      vayu_log("[CALIB] Collecting Gyro data (Bias-Only)...");
      float avg[3];
      wait_for_orientation(CALIB_UPDATE_UPRIGHT, avg);

      float gsum[3] = {0, 0, 0};
      for (int i = 0; i < 500; i++) {
        gsum[0] += _bmx_data.converted.gyr_raw[0];
        gsum[1] += _bmx_data.converted.gyr_raw[1];
        gsum[2] += _bmx_data.converted.gyr_raw[2];
        v_delay(5);
      }
      bmx160_calib.gyr_offset[0] = gsum[0] / 500.0f;
      bmx160_calib.gyr_offset[1] = gsum[1] / 500.0f;
      bmx160_calib.gyr_offset[2] = gsum[2] / 500.0f;
    }
  } else if (imu_id == 3.0f) { // MAGNETOMETER
    vayu_log(
        "[CALIB] Starting Magnetometer Quick Calibration (Free-Rotation)...");

    // Prompt user to rotate
    uint8_t payload[7];
    payload[0] = SYSTEM_ORIGIN_CALIBRATION;
    payload[1] = 0x01; // nArgs
    payload[2] = CALIB_UPDATE_FREE_ROT;
    float zero = 0.0f;
    v_memcpy(&payload[3], &zero, 4);
    send_packet(&g_telemetry_channel, PACKET_TYPE_SYSTEM_STATUS, payload, 7);

    v_delay(1000); // Give user time to see it

    float mag_max[3] = {-10000.0f, -10000.0f, -10000.0f};
    float mag_min[3] = {10000.0f, 10000.0f, 10000.0f};

    const int calibration_time_ms = 20000; // 20 seconds
    const int loop_delay_ms = 20;
    const int iterations = calibration_time_ms / loop_delay_ms;

    for (int i = 0; i < iterations; i++) {
      // Use DMA data from _bmx_data.converted.mag_compensated
      if (_bmx_data.raw.rhall >= 50 && _bmx_data.raw.rhall <= 30000) {
        float mx_c = _bmx_data.converted.mag_compensated[0];
        float my_c = _bmx_data.converted.mag_compensated[1];
        float mz_c = _bmx_data.converted.mag_compensated[2];

        if (IS_FINITE(mx_c) && IS_FINITE(my_c) && IS_FINITE(mz_c)) {
          // Alignment is already handled in process_data: [-Y, X, Z]
          float cur_mag[3] = {mx_c, my_c, mz_c};

          for (int axis = 0; axis < 3; axis++) {
            if (cur_mag[axis] > mag_max[axis])
              mag_max[axis] = cur_mag[axis];
            if (cur_mag[axis] < mag_min[axis])
              mag_min[axis] = cur_mag[axis];
          }
        }
      }

      // Progress update every 5%
      if (i % (iterations / 20) == 0) {
        payload[2] = CALIB_UPDATE_PROGRESS;
        float progress = (100.0f * i) / iterations;
        v_memcpy(&payload[3], &progress, 4);
        send_packet(&g_telemetry_channel, PACKET_TYPE_SYSTEM_STATUS, payload,
                    7);
      }

      v_delay(loop_delay_ms);
    }

    vayu_log("[CALIB] Mag Bounds: X[%.1f, %.1f], Y[%.1f, %.1f], Z[%.1f, %.1f]",
             mag_min[0], mag_max[0], mag_min[1], mag_max[1], mag_min[2],
             mag_max[2]);

    // Calculate Hard Iron (Bias)
    for (int i = 0; i < 3; i++) {
      bmx160_calib.mag_offset[i] = (mag_max[i] + mag_min[i]) * 0.5f;
    }

    // Calculate per-axis scale
    float scale[3];
    float avg_scale = 0.0f;
    for (int i = 0; i < 3; i++) {
      scale[i] = (mag_max[i] - mag_min[i]) * 0.5f;
      avg_scale += scale[i];
    }
    avg_scale /= 3.0f;

    for (int i = 0; i < 3; i++) {
      if (scale[i] > 0.001f) {
        bmx160_calib.mag_scale[i] = avg_scale / scale[i];
      } else {
        bmx160_calib.mag_scale[i] = 1.0f;
      }
    }

    vayu_log("[CALIB] Mag Bias: %.3f, %.3f, %.3f", bmx160_calib.mag_offset[0],
             bmx160_calib.mag_offset[1], bmx160_calib.mag_offset[2]);
    vayu_log("[CALIB] Mag Scale: %.3f, %.3f, %.3f", bmx160_calib.mag_scale[0],
             bmx160_calib.mag_scale[1], bmx160_calib.mag_scale[2]);
  }

  vayu_log("[BMX] Calibration Done. Biases applied.");
  v_delay(10);

  // Send Final Success Packet
  uint8_t payload[7];
  payload[0] = SYSTEM_ORIGIN_CALIBRATION;
  payload[1] = 0x01;
  payload[2] = CALIB_UPDATE_PROGRESS;
  float final_p = 100.0f;
  v_memcpy(&payload[3], &final_p, 4);
  send_packet(&g_telemetry_channel, PACKET_TYPE_SYSTEM_STATUS, payload, 7);

  i2c_error_count = 0;
  vfs_fd_t file =
      vfs_open(CALIBRATION_FILE_PATH, VFS_O_WRONLY | VFS_O_CREAT | VFS_O_TRUNC);
  if (file < 0) {
    vayu_log("[CALIB] Failed to open calibration file.");
    return;
  }
  uint8_t res = vfs_write(file, &bmx160_calib, sizeof(bmx160_calibration_t));
  vayu_log("[CALIB] Calibration file written. Result: %d", res);
  vfs_close(file);
  // bmx160_init();
  // v_delay(50);
  // wake_imu_read_task();
  // v_delay(10);
  system_state_set(SYSTEM_STATE_STANDBY);

  if (cal_args)
    v_free(cal_args);
  task_exit();
}
