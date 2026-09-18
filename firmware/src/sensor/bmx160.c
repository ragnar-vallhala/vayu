#include "sensor/bmx160.h"
#include "storage/imu_hs_log.h"
#include "calib/calib_engine.h"
#include "comm/comm.h"
#include "sensor/bme280.h"
#include "sensor/i2c_manager.h"
#include "sensor/vl53l0x.h"
#include "navhal.h"
#include "ipc.h"
#include "est/est.h"
#include "memory.h"
#include "sensor/imu_buffer.h"
#include "sys/state.h"
#include "task.h"
#include "utils.h"
#include "storage/fs_owner.h"
#include "vaios.h"
#include "variables.h"
#include "vayu_tasks.h"
#include "vfs.h"
#include "maths/maths_interface.h"
#include <stdint.h>

extern float m_sqrt(float x);
extern float m_fabsf(float x);

#define FABS_F(x) ((x) < 0.0f ? -(x) : (x))
#define SQRT_F(x) m_sqrt(x)

/* Gyro-bias "still" detector thresholds, pre-squared at compile time so the
 * per-sample check compares squared magnitudes (no sqrt, no per-loop compute).
 * Stillness = |acc| within ACC_STILL_TOL_G of 1g AND |gyro| < GYRO_STILL_DPS. */
#define ACC_STILL_G 9.81f
#define ACC_STILL_TOL_G 0.2f
#define GYRO_STILL_DPS 0.2f
#define ACC_STILL_LO_SQ                                                        \
  ((ACC_STILL_G - ACC_STILL_TOL_G) *                                           \
   (ACC_STILL_G - ACC_STILL_TOL_G)) /* 9.61^2 */
#define ACC_STILL_HI_SQ                                                        \
  ((ACC_STILL_G + ACC_STILL_TOL_G) *                                           \
   (ACC_STILL_G + ACC_STILL_TOL_G))                     /* 10.01^2 */
#define GYRO_STILL_SQ (GYRO_STILL_DPS * GYRO_STILL_DPS) /* 0.04 */
static int in_init = 1;
#define IS_FINITE(x) (m_isfinite(x))
uint8_t tx_buf[2];
uint8_t rx_buf[14]; // sized for safe multi-byte reads

static float last_mag[3] = {0}; // last valid mag readings
static int stable_count = 0;    // is platform stable

static SemaphoreHandle_t bmx160_ready_sema; // Semaphore for data ready
/* Given at IMU_SAMPLE_FREQ_HZ by the HIGH_FREQ_TIMER (bmx160_fast_tick_isr);
 * the read task waits on it before each FAST (accel/gyro) read to pace the IMU
 * to a fixed rate instead of free-running at I2C speed. */
static SemaphoreHandle_t bmx160_fast_tick_sema = NULL;

static bmx160_config_t bmx160_cfg;
// DMA storage buffers
static uint8_t _bmx_dma_rx_buffer_double[32] __attribute__((aligned(4)));
/* Attitude/fusion state lives in the attitude task (attitude_task.c); this
 * driver only acquires, converts, timestamps and fans out samples. */
static bmx160_all_reading_t _bmx_data;

/* Set by the MAG DMA callback, consumed (and cleared) by bmx160_process_data:
 * the magnetometer (BMM150) only refreshes ~1/13 of the FAST cadence, so the
 * expensive compensation/normalization runs only when there's new mag data —
 * the converted mag fields hold their last value otherwise (identical to
 * recomputing from the unchanged bytes). */
static volatile uint8_t _mag_fresh = 0;
/* Set by the TEMP DMA callback; consumed (and cleared) by bmx160_process_data.
 * Temperature shares the mag decimation (~1/13 of FAST), so the conversion runs
 * only on a fresh TEMP read — converted.temp holds its last value otherwise. */
static volatile uint8_t _temp_fresh = 0;

// Split-rate state machine
typedef enum {
  IMU_OP_FAST, // Gyro + Accel
  IMU_OP_MAG,  // Magnetometer
  IMU_OP_TEMP, // Temperature
  IMU_OP_BARO, // BME280 baro/humidity (shares this single-owner bus loop)
  IMU_OP_TOF   // VL53L0X ToF rangefinder (ditto)
} imu_op_t;

/* Read the BME280 every Nth TEMP slot. TEMP runs ~1/13 of FAST (2 kHz) ~= 150
 * Hz, so /10 ~= 15 Hz — matched to the BME280's 62.5 ms normal-mode cadence. */
#define BARO_READ_DECIM 10

/* Read the VL53L0X every Nth TEMP slot. 150/7 ~= 21 Hz, comfortably under the
 * device's ~30 Hz continuous cadence. 7 is coprime with BARO_READ_DECIM so the
 * two ride-along slots almost never land on the same TEMP tick (when they do,
 * the baro wins and the ToF read simply waits one cycle). */
#define TOF_READ_DECIM 7

extern hal_i2c_config_t i2c_config;
static volatile imu_op_t _next_op = IMU_OP_FAST;
static volatile imu_op_t _last_op = IMU_OP_FAST;

static volatile int isr_count = 0;
static volatile int task_count = 0;

// Forward declarations for split-rate callbacks
static void bmx160_dma_callback_fast(void *args);
static void bmx160_dma_callback_mag(void *args);
static void bmx160_dma_callback_temp(void *args);
static void bmx160_dma_callback_baro(void *args);
static void bmx160_dma_callback_tof(void *args);

static float acc_scale = 0.0f;
static float gyr_scale = 0.0f;
static bmx160_calibration_t bmx160_calib = {
    .acc_offset = {0.0f, 0.0f, 0.0f},
    .acc_soft_iron = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f},
    .gyr_offset = {0.0f, 0.0f, 0.0f},
    .mag_offset = {0.0f, 0.0f, 0.0f},
    .mag_soft_iron = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f},
    .board_trim = {0.0f, 0.0f}};

/* Board-level / trim accessor (see bmx160.h). Read live so a fresh board-level
 * calibration takes effect without a reboot; two float reads, no lock (the
 * estimator reads while the calibration task may write — a torn read is
 * harmless, same R8.6 rationale as the gyro-bias offset). */
void bmx160_get_board_trim(float *roll_deg, float *pitch_deg) {
  if (roll_deg)
    *roll_deg = bmx160_calib.board_trim[0];
  if (pitch_deg)
    *pitch_deg = bmx160_calib.board_trim[1];
}

/* Set by bmx160_calib_request_cancel() (CMD_CANCEL_CALIBRATION), polled and
 * cleared by calibration_task. volatile: written from the comm task, read from
 * the calibration task. */
static volatile int _calib_cancel = 0;

/* True while a calibration routine owns the IMU stream. The sample diversion in
 * bmx160_process_data() keys off THIS, not the system state: the RC watchdog can
 * slam the FC into FAILSAFE at any instant (no RC link is normal during a bench
 * calibration), and gating on `state == CALIBRATING` would silently cut the
 * calibration task off from its samples mid-pose and hang it forever. volatile:
 * written by the calibration task, read by the sensor task. */
static volatile int _calib_active = 0;

/** @noreq Calibration-cancel flag setter; dispatched by COMM-CMD-001. */
void bmx160_calib_request_cancel(void) { _calib_cancel = 1; }

// LPFs for sensors
static lpf_t acc_lpf[3];
static lpf_t gyr_lpf[3];

static bmm150_trim_data_t _mag_trim;
static int i2c_error_count = 0;

// Helper functions
static bmx160_err_type bmx160_set_mag_conf(void);
static bmx160_err_type bmx160_convert_raw_temp_to_celcius(int16_t raw_temp,
                                                          float *celcius);
static float bmx160_range_code_to_g(uint8_t range_code);
static float bmx160_range_code_to_dps(uint8_t range_code);
static void bmx160_process_data(void);
static bmx160_err_type bmx160_wait_mag_manual_op(void);

/** @noreq BMM150 manual-op completion poll; low-level mag-init helper. */
static bmx160_err_type bmx160_wait_mag_manual_op(void) {
  uint8_t status_reg = 0x1B; // STATUS register
  uint8_t status;
  int timeout = 100;
  do {
    if (i2c_manager_write_read(BMX160_I2C_ADDR, &status_reg, 1, &status, 1) !=
        HAL_OK)
      return ERR0;
    if (!(status & 0x04)) // mag_man_op bit clear = operation done
      return NO_ERR;
    v_delay(1);
  } while (--timeout > 0);

  // Phase 2: wait for mag_man_op to go LOW (operation complete)
  timeout = 100;
  do {
    if (i2c_manager_write_read(BMX160_I2C_ADDR, &status_reg, 1, &status, 1) !=
        HAL_OK)
      return ERR0;
    if (!(status & 0x04))
      return NO_ERR;
    v_delay(1);
  } while (--timeout > 0);

  vayu_log("BMX160: Mag manual op timeout!");
  return ERR1;
}

/** @implements SNS-BMX-101 */
static bmx160_err_type bmx160_verify_pmu(uint8_t mask, uint8_t expected) {
  uint8_t reg = BMX160_PMU_STAT_ADDR;

  if (i2c_manager_write_read(BMX160_I2C_ADDR, &reg, 1, rx_buf, 1) != HAL_OK) {

    return ERR0;
  }
  uint8_t stat = rx_buf[0];

  if ((stat & mask) == expected) {
    return NO_ERR;
  }
  return ERR1;
}

/** @implements SNS-BMX-101, SNS-CAL-001 */
hal_status_t bmx160_init(void) {

  // Create I2C bus semaphore early. Ensure it starts "given"
  in_init = 1; // Explicitly set it here as well

  // Read calibration from sd card. Validate the versioned header before
  // applying the payload; on any mismatch (e.g. a pre-v2 headerless file) keep
  // the compiled-in identity defaults rather than loading garbage.
  vfs_fd_t file = vfs_open(CALIBRATION_FILE_PATH, VFS_O_RDONLY);
  if (file < 0) {
    vayu_log("[CALIB] Failed to open calibration file.");
  } else {
    calib_file_header_t hdr;
    bmx160_calibration_t loaded;
    int hn = vfs_read(file, &hdr, sizeof(hdr));
    int pn = vfs_read(file, &loaded, sizeof(loaded));
    vfs_close(file);
    if (hn == (int)sizeof(hdr) && pn == (int)sizeof(loaded) &&
        hdr.magic == CALIB_FILE_MAGIC && hdr.version == CALIB_FILE_VERSION &&
        hdr.payload_size == (uint16_t)sizeof(bmx160_calibration_t)) {
      bmx160_calib = loaded;
      vayu_log("[CALIB] Calibration loaded (v%u).", (unsigned)hdr.version);
    } else {
      vayu_log(
          "[CALIB] Calibration file invalid/old; using identity defaults.");
    }
  }
  // 1. Verify Chip ID
  uint16_t chip_id = bmx160_get_chip_id();
  if (chip_id != BMX160_CHIP_ID) {
    return HAL_ERR_NOT_INITIALIZED;
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

  /* Configure the accelerometer and the gyro.*/
  bmx160_cfg.bmx160_acc_odr = BMX_ACC_ODR;
  bmx160_cfg.bmx160_acc_bwp = BMX_ACC_BWP;
  bmx160_cfg.bmx160_acc_us = BMX_ACC_US;
  bmx160_cfg.bmx160_acc_range = BMX_ACC_RANGE;
  bmx160_cfg.bmx160_gyr_odr = BMX_GYR_ODR;
  bmx160_cfg.bmx160_gyr_bwp = BMX_GYR_BWP;
  bmx160_cfg.bmx160_gyr_range = BMX_GYR_RANGE;
  if (bmx160_write_acc_config(&bmx160_cfg) != NO_ERR ||
      bmx160_write_gyr_config(&bmx160_cfg) != NO_ERR) {
    return HAL_ERR_NOT_INITIALIZED; // flying on an unknown IMU config is worse
  }

  // Scales follow from the range codes just written.
  float g_range = bmx160_range_code_to_g((uint8_t)bmx160_cfg.bmx160_acc_range);
  acc_scale = g_range * 9.80665f / 32768.0f;
  float dps_range =
      bmx160_range_code_to_dps((uint8_t)bmx160_cfg.bmx160_gyr_range);
  gyr_scale = dps_range / 32768.0f;
  imu_hs_log_set_scale(gyr_scale, acc_scale);

  // 6. Configure Magnetometer (BMM150 setup)
  if (bmx160_set_mag_conf() != NO_ERR) {
    return HAL_ERR_NOT_INITIALIZED;
  }

  // Orientation quaternion is initialized by the attitude task.

  // Create semaphore for data ready
  if (bmx160_ready_sema == NULL) {
    bmx160_ready_sema = v_semaphore_create_counting(10, 0);
  }
  if (bmx160_ready_sema == NULL) {
    return HAL_ERR_NOT_INITIALIZED;
  }

  // FAST-read pacing tick (given by bmx160_fast_tick_isr at IMU_SAMPLE_FREQ_HZ)
  if (bmx160_fast_tick_sema == NULL) {
    bmx160_fast_tick_sema = v_semaphore_create_binary();
  }

  in_init = 0; // Success! Disable blocking bypass

  _next_op = IMU_OP_FAST;
  _last_op = IMU_OP_FAST;

  // Kick off the loop from task context by giving the semaphore
  v_semaphore_give(bmx160_ready_sema);

  return HAL_OK;
}

/** @noreq BMM150 indirect register write via the BMX160 mag interface. */
static bmx160_err_type bmx160_write_bmm150_reg(uint8_t reg, uint8_t data) {
  tx_buf[0] = BMX160_MAG_IF_3_DATA_ADDR;
  tx_buf[1] = data;

  if (i2c_manager_write(BMX160_I2C_ADDR, tx_buf, 2) != HAL_OK) {
    return ERR0;
  }

  v_delay(2); // Wait for BMX -> BMM write

  tx_buf[0] = BMX160_MAG_IF_2_REG_ADDR;
  tx_buf[1] = reg;

  if (i2c_manager_write(BMX160_I2C_ADDR, tx_buf, 2) != HAL_OK) {
    return ERR0;
  }

  if (bmx160_wait_mag_manual_op() != NO_ERR)
    return ERR1;

  return NO_ERR;
}

/** @noreq BMM150 indirect register read via the BMX160 mag interface. */
static bmx160_err_type bmx160_read_bmm150_reg(uint8_t reg, uint8_t *data) {
  /* Every error path below returns WITHOUT writing *data, and all 19 call
   * sites ignore the return -- so an I2C hiccup during init left the BMM150
   * trim table built from stack garbage, and `tmp[1] << 8` in
   * bmx160_read_mag_trim_data was undefined behaviour on an uninitialised
   * value. Land a defined 0 first: it is what the callers already assume. */
  *data = 0;
  tx_buf[0] = BMX160_MAG_IF_1_READ_ADDR;
  tx_buf[1] = reg;

  if (i2c_manager_write(BMX160_I2C_ADDR, tx_buf, 2) != HAL_OK) {
    return ERR0;
  }

  if (bmx160_wait_mag_manual_op() != NO_ERR)
    return ERR1;

  uint8_t read_reg = 0x04; // MAG_X_LSB in BMX160 is where IF data appears

  if (i2c_manager_write_read(BMX160_I2C_ADDR, &read_reg, 1, data, 1) !=
      HAL_OK) {
    return ERR0;
  }
  return NO_ERR;
}

/** @implements SNS-MAG-101 */
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

/** @implements SNS-BMX-101 */
static bmx160_err_type bmx160_set_mag_conf(void) {
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

  // 8b. CRITICAL: the chip-ID read above repointed MAG_IF_1 to 0x40. Restore
  // the auto-read address to 0x42 (BMM150 DATA_X_LSB) before enabling auto
  // mode, otherwise the burst read starts at the chip-ID register and every
  // mag axis is shifted 2 bytes — mag X then reads the constant chip ID (0x32)
  // and that axis is frozen for the life of the build (long-standing mag-axis
  // bug). Step 6 set this, but the chip-ID check clobbered it.
  tx_buf[0] = BMX160_MAG_IF_1_READ_ADDR;
  tx_buf[1] = 0x42;
  i2c_manager_write(BMX160_I2C_ADDR, tx_buf, 2);
  v_delay(1);

  // 9. Enable Auto-mode (manual_en = 0, burst_read = 8 bytes)
  tx_buf[0] = BMX160_MAG_IF_0_CONF_ADDR;
  tx_buf[1] = 0x03;
  i2c_manager_write(BMX160_I2C_ADDR, tx_buf, 2);
  v_delay(10);

  return NO_ERR;
}

/** @noreq Single-register CHIP_ID read; init verify carries SNS-BMX-101. */
uint16_t bmx160_get_chip_id(void) {
  uint8_t reg = BMX160_CHIP_ID_ADDR;
  hal_status_t ret;

  ret = i2c_manager_write_read(BMX160_I2C_ADDR, &reg, 1, rx_buf, 1);
  if (ret != HAL_OK) {
    return 0xFFFF;
  }
  int16_t chip_id = (int16_t)(rx_buf[0]);
  return (uint16_t)chip_id;
}

/** @noreq Trivial raw accessor. */
int16_t bmx160_read_temp_raw(void) { return _bmx_data.raw.temp; }

/** @implements SNS-MAG-101 */
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

/** @implements SNS-MAG-101 */
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

/** @implements SNS-MAG-101 */
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

/** @noreq Temperature unit conversion; no temperature requirement. */
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

/** @noreq Trivial converted-value accessor. */
bmx160_err_type bmx160_read_temp_celcius(float *celcius) {
  if (celcius == NULL)
    return ERR0;
  *celcius = _bmx_data.converted.temp;
  return NO_ERR;
}

/** @noreq Trivial raw accessor. */
bmx160_err_type bmx160_read_acc_raw(int16_t *raw) {
  if (raw == NULL)
    return ERR0;
  raw[0] = _bmx_data.raw.acc[0];
  raw[1] = _bmx_data.raw.acc[1];
  raw[2] = _bmx_data.raw.acc[2];
  return NO_ERR;
}
/** @noreq Trivial raw accessor. */
bmx160_err_type bmx160_read_gyr_raw(int16_t *raw) {
  if (raw == NULL)
    return ERR0;
  raw[0] = _bmx_data.raw.gyr[0];
  raw[1] = _bmx_data.raw.gyr[1];
  raw[2] = _bmx_data.raw.gyr[2];
  return NO_ERR;
}

/** @noreq Trivial raw accessor. */
bmx160_err_type bmx160_read_mag_raw(int16_t *raw) {
  if (raw == NULL)
    return ERR0;
  raw[0] = _bmx_data.raw.mag[0];
  raw[1] = _bmx_data.raw.mag[1];
  raw[2] = _bmx_data.raw.mag[2];
  return NO_ERR;
}

/** @noreq Trivial raw accessor. */
bmx160_err_type bmx160_read_all_raw(bmx160_all_reading_t *raw) {
  if (raw == NULL)
    return ERR0;
  *raw = _bmx_data;
  return NO_ERR;
}

/** @noreq Trivial converted-value accessor. */
bmx160_err_type bmx160_read_acc_mps2(float *data) {
  if (data == NULL)
    return ERR0;
  data[0] = _bmx_data.converted.acc[0];
  data[1] = _bmx_data.converted.acc[1];
  data[2] = _bmx_data.converted.acc[2];
  return NO_ERR;
}

/** @noreq Trivial converted-value accessor. */
bmx160_err_type bmx160_read_gyr_dps(float *data) {
  if (data == NULL)
    return ERR0;
  data[0] = _bmx_data.converted.gyr[0];
  data[1] = _bmx_data.converted.gyr[1];
  data[2] = _bmx_data.converted.gyr[2];
  return NO_ERR;
}

/** @noreq Trivial converted-value accessor. */
bmx160_err_type bmx160_read_mag_uT(float *data) {
  if (data == NULL)
    return ERR0;
  data[0] = _bmx_data.converted.mag[0];
  data[1] = _bmx_data.converted.mag[1];
  data[2] = _bmx_data.converted.mag[2];
  return NO_ERR;
}

/** @noreq Trivial converted-value accessor. */
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

/** @noreq Config-register readback helper. */
bmx160_err_type bmx160_read_acc_config(bmx160_config_t *config) {
  // Reading ACC conf
  tx_buf[0] = BMX160_ACC_CONF_ADDR;
  if (i2c_manager_write_read(BMX160_I2C_ADDR, tx_buf, 1, rx_buf, 1) == HAL_OK) {
    config->bmx160_acc_us = GET_ACC_US(rx_buf[0]);
    config->bmx160_acc_bwp = GET_ACC_BWP(rx_buf[0]);
    config->bmx160_acc_odr = GET_ACC_ODR(rx_buf[0]);
  } else
    return ERR0;

  tx_buf[0] = BMX160_ACC_RANGE_ADDR;
  if (i2c_manager_write_read(BMX160_I2C_ADDR, tx_buf, 1, rx_buf, 1) == HAL_OK) {
    config->bmx160_acc_range = GET_ACC_RANGE(rx_buf[0]);
  } else
    return ERR0;
  return NO_ERR;
}

/** @noreq Config-register readback helper. */
bmx160_err_type bmx160_read_gyr_config(bmx160_config_t *config) {
  // Reading GYR conf
  tx_buf[0] = BMX160_GYR_CONF_ADDR;
  if (i2c_manager_write_read(BMX160_I2C_ADDR, tx_buf, 1, rx_buf, 1) == HAL_OK) {
    config->bmx160_gyr_bwp = GET_GYR_BWP(rx_buf[0]);
    config->bmx160_gyr_odr = GET_GYR_ODR(rx_buf[0]);
  } else
    return ERR0;

  tx_buf[0] = BMX160_GYR_RANGE_ADDR;
  if (i2c_manager_write_read(BMX160_I2C_ADDR, tx_buf, 1, rx_buf, 1) == HAL_OK) {
    config->bmx160_gyr_range = GET_GYR_RANGE(rx_buf[0]);
  } else
    return ERR0;
  return NO_ERR;
}

/** @noreq Config-register readback helper. */
bmx160_err_type bmx160_read_mag_config(bmx160_config_t *config) {
  // Reading MAG conf
  tx_buf[0] = BMX160_MAG_CONF_ADDR;
  if (i2c_manager_write_read(BMX160_I2C_ADDR, tx_buf, 1, rx_buf, 1) == HAL_OK) {
    config->bmx160_mag_odr = GET_MAG_ODR(rx_buf[0]);
  } else
    return ERR0;
  return NO_ERR;
}

/** @noreq Config-register readback helper. */
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

/** @implements SNS-BMX-102, SNS-BMX-103 */
bmx160_err_type bmx160_write_config(bmx160_config_t *config) {
  if (bmx160_write_acc_config(config) != NO_ERR)
    return ERR0;
  if (bmx160_write_gyr_config(config) != NO_ERR)
    return ERR0;
  if (bmx160_write_mag_config(config) != NO_ERR)
    return ERR0;
  bmx160_set_current_config(config);

  // Pre-calculate scales
  float g_range = bmx160_range_code_to_g((uint8_t)config->bmx160_acc_range);
  acc_scale = g_range * 9.80665f / 32768.0f;

  float dps_range = bmx160_range_code_to_dps((uint8_t)config->bmx160_gyr_range);
  gyr_scale = dps_range / 32768.0f;
  imu_hs_log_set_scale(gyr_scale, acc_scale);

  return NO_ERR;
}

/* The register bytes the configuration in variables.h has to pack down to,
 * from the BMI160 datasheet. This is where the g-value-vs-register-code trap
 * shows up: with the old enum, ACC_RANGE packed 16 & 15 == 0 -- a reserved
 * value -- and the sensor silently kept whatever range it already had. Failing
 * the build is the only way that stays caught, since nothing host-compiles
 * this driver. */
_Static_assert((((BMX_ACC_US & 1U) << 7U) | ((BMX_ACC_BWP & 7U) << 4U) |
                (BMX_ACC_ODR & 15U)) == 0x2C,
               "ACC_CONF: 1600 Hz ODR, normal bandwidth, no undersampling");
_Static_assert((BMX_ACC_RANGE & 15U) == 0x0C, "ACC_RANGE: +-16 g");
_Static_assert((((BMX_GYR_BWP & 3U) << 4U) | (BMX_GYR_ODR & 15U)) == 0x2C,
               "GYR_CONF: 1600 Hz ODR, normal bandwidth");
_Static_assert((BMX_GYR_RANGE & 7U) == 0x00, "GYR_RANGE: +-2000 dps");

/** @noreq Register-field packing helper. */
static uint8_t bmx160_get_acc_conf(bmx160_config_t *config) {
  uint8_t val = (uint8_t)(((config->bmx160_acc_us & 1U) << 7U) |
                          ((config->bmx160_acc_bwp & 7U) << 4U) |
                          (config->bmx160_acc_odr & 15U));
  return val;
}

/** @noreq Register-field packing helper. */
static uint8_t bmx160_get_acc_range(bmx160_config_t *config) {
  uint8_t val = (config->bmx160_acc_range & 15U);
  return val;
}

/** @noreq Register-field packing helper. */
static uint8_t bmx160_get_gyr_conf(bmx160_config_t *config) {
  uint8_t val = (uint8_t)(((config->bmx160_gyr_bwp & 3U) << 4U) |
                          (config->bmx160_gyr_odr & 15U));
  return val;
}

/** @noreq Register-field packing helper. */
static uint8_t bmx160_get_gyr_range(bmx160_config_t *config) {
  uint8_t val = (config->bmx160_gyr_range & 7U);
  return val;
}

/** @noreq Register-field packing helper. */
static uint8_t bmx160_get_mag_conf(bmx160_config_t *config) {
  uint8_t val = (config->bmx160_mag_odr & 15U);
  return val;
}

/** @implements SNS-BMX-102 */
bmx160_err_type bmx160_write_acc_config(bmx160_config_t *config) {
  uint8_t val;

  // --- ACC_CONF ---
  val = bmx160_get_acc_conf(config);
  tx_buf[0] = BMX160_ACC_CONF_ADDR;
  tx_buf[1] = val;
  if (i2c_manager_write(BMX160_I2C_ADDR, tx_buf, 2) != HAL_OK)
    return ERR0;

  // --- ACC_RANGE ---
  val = bmx160_get_acc_range(config);
  tx_buf[0] = BMX160_ACC_RANGE_ADDR;
  tx_buf[1] = val;
  if (i2c_manager_write(BMX160_I2C_ADDR, tx_buf, 2) != HAL_OK)
    return ERR0;

  return NO_ERR;
}

/** @implements SNS-BMX-102 */
bmx160_err_type bmx160_write_gyr_config(bmx160_config_t *config) {
  uint8_t val;

  // --- GYR_CONF ---
  val = bmx160_get_gyr_conf(config);
  tx_buf[0] = BMX160_GYR_CONF_ADDR;
  tx_buf[1] = val;
  if (i2c_manager_write(BMX160_I2C_ADDR, tx_buf, 2) != HAL_OK)
    return ERR0;

  // --- GYR_RANGE ---
  val = bmx160_get_gyr_range(config);
  tx_buf[0] = BMX160_GYR_RANGE_ADDR;
  tx_buf[1] = val;
  if (i2c_manager_write(BMX160_I2C_ADDR, tx_buf, 2) != HAL_OK)
    return ERR0;

  return NO_ERR;
}

/** @implements SNS-BMX-102 */
bmx160_err_type bmx160_write_mag_config(bmx160_config_t *config) {
  uint8_t val;

  // --- MAG_CONF ---
  val = bmx160_get_mag_conf(config);
  tx_buf[0] = BMX160_MAG_CONF_ADDR;
  tx_buf[1] = val;
  if (i2c_manager_write(BMX160_I2C_ADDR, tx_buf, 2) != HAL_OK)
    return ERR0;

  return NO_ERR;
}
/** @noreq Range-code -> full-scale (g) lookup. */
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

/** @implements SNS-BMX-103 */
float bmx160_raw_acc_to_mps2(int16_t raw) {
  // Convert raw → g → m/s² using pre-calculated scale
  return (float)raw * acc_scale;
}
/** @noreq Range-code -> full-scale (dps) lookup. */
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

/** @implements SNS-BMX-103 */
float bmx160_raw_gyr_to_dps(int16_t raw) {
  // Convert raw → dps using pre-calculated scale
  return (float)raw * gyr_scale;
}

/** @noreq Trivial config-cache accessor. */
bmx160_config_t bmx160_get_current_config(void) { return bmx160_cfg; }
/** @noreq Trivial config-cache accessor. */
void bmx160_set_current_config(bmx160_config_t *cfg) { bmx160_cfg = *cfg; }

// Run from ISR
#define MAX(a, b) ((a) > (b) ? (a) : (b))

/** @implements SNS-BMX-104, SNS-BMX-105 */
void bmx160_initiate_read(void *args) {
  (void)args;
  static uint32_t last_tick = 0;
  while (1) {
    if (v_semaphore_take(bmx160_ready_sema, MS_TO_TICKS(2)) == VA_PASS) {
      last_tick = v_get_ticks();

      // 1. Process data from the op that just COMPLETED
      if (_last_op == IMU_OP_FAST) {
        bmx160_process_data();
      }

      // 2. Start the NEXT op in the chain
      hal_status_t ret = HAL_OK;
      _last_op = _next_op;

      switch (_next_op) {
      case IMU_OP_FAST:
        /* Pace accel/gyro acquisition to IMU_SAMPLE_FREQ_HZ. Drain any tick
         * that arrived while we were busy (e.g. during the previous DMA wait),
         * then block for the NEXT fresh tick so a full period elapses — without
         * the drain the pending tick would be consumed immediately and the read
         * would free-run at I2C speed. CPU is free while blocked; the 2 ms cap
         * keeps the chain alive if the tick source ever stalls. */
        v_semaphore_take(bmx160_fast_tick_sema, 0); /* clear stale */
        v_semaphore_take(bmx160_fast_tick_sema, MS_TO_TICKS(2));
        ret = i2c_manager_read_async(BMX160_I2C_ADDR, 0x0C, 12,
                                     bmx160_dma_callback_fast);
        break;
      case IMU_OP_MAG:
        ret = i2c_manager_read_async(BMX160_I2C_ADDR, 0x04, 8,
                                     bmx160_dma_callback_mag);
        break;
      case IMU_OP_TEMP:
        ret = i2c_manager_read_async(BMX160_I2C_ADDR, 0x20, 2,
                                     bmx160_dma_callback_temp);
        break;
      case IMU_OP_BARO:
        /* Read the BME280's 8 data bytes (0xF7..0xFE) through the SAME
         * single-owner DMA path. No contention because only this loop ever
         * drives the bus at runtime. */
        ret = i2c_manager_read_async(BME280_I2C_ADDR, BME280_REG_DATA,
                                     BME280_DATA_LEN, bmx160_dma_callback_baro);
        break;
      case IMU_OP_TOF:
        /* Read the VL53L0X's 12-byte result block (0x14..0x1F) through the SAME
         * single-owner DMA path. The device free-runs in continuous mode, so
         * this is a pure read — no trigger, and no interrupt-clear write (the
         * async path cannot write; see vl53l0x.h). */
        ret = i2c_manager_read_async(VL53L0X_I2C_ADDR, VL53L0X_REG_RESULT_RANGE,
                                     VL53L0X_DATA_LEN, bmx160_dma_callback_tof);
        break;
      }

      if (ret != HAL_OK) {
        vayu_log("DMA FAIL -> HARD RECOVERY");
        i2c_manager_unstick();
        _next_op = IMU_OP_FAST;
        // No fake wake-up: wait for watchdog or next valid event
      }

      task_count++;
      if (task_count % 1000 == 0) {
      }
    } else {
      if (v_get_ticks() - last_tick > 50) {
        // FULL RECOVERY
        i2c_manager_unstick();
        init_i2c_manager(&i2c_config);

        _next_op = IMU_OP_FAST;
        _last_op = IMU_OP_FAST;

        // CRITICAL: START DMA MANUALLY
        hal_status_t ret = i2c_manager_read_async(BMX160_I2C_ADDR, 0x0C, 12,
                                                  bmx160_dma_callback_fast);

        if (ret != HAL_OK) {
          vayu_log("RESTART FAILED");
        }
        task_count = 0;
        isr_count = 0;
        last_tick = v_get_ticks();
      }
    }
  }
}

/** @implements SNS-BMX-104, SNS-BMX-105 */
void bmx160_dma_callback_fast(void *args) {
  static uint32_t slow_counter = 0;
  isr_count++;
  if (args != NULL) {
    // Copy Gyro/Accel (12 bytes) to [8-19]
    v_memcpy(&_bmx_dma_rx_buffer_double[8], args, 12);
  }

  slow_counter++;
  if (slow_counter % 13 == 0) {
    _next_op = IMU_OP_MAG;
  } else {
    _next_op = IMU_OP_FAST;
  }

  int higher_priority_task_woken = 0;
  v_semaphore_give_from_isr(bmx160_ready_sema, &higher_priority_task_woken);
  if (higher_priority_task_woken) {
    task_yield();
  }
}

/** @implements SNS-BMX-104, SNS-BMX-105 */
static void bmx160_dma_callback_mag(void *args) {
  if (args != NULL) {
    v_memcpy(&_bmx_dma_rx_buffer_double[0], args, 8);
    _mag_fresh = 1; /* new mag data -> process_data will recompute the field */
  }
  isr_count++;
  _next_op = IMU_OP_TEMP;

  int higher_priority_task_woken = 0;
  v_semaphore_give_from_isr(bmx160_ready_sema, &higher_priority_task_woken);
  if (higher_priority_task_woken) {
    task_yield();
  }
}

/** @implements SNS-BMX-104, SNS-BMX-105 */
static void bmx160_dma_callback_temp(void *args) {
  static uint32_t baro_counter = 0;
  static uint32_t tof_counter = 0;
  if (args != NULL) {
    v_memcpy(&_bmx_dma_rx_buffer_double[28], args, 2);
    _temp_fresh = 1; /* new temperature -> process_data will reconvert it */
  }
  isr_count++;
  /* Every Nth TEMP slot, read the BME280 instead of returning to FAST — but
   * only if the baro is present (else its DMA read would NACK and trip the
   * IMU recovery path). The baro callback returns the chain to FAST. */
  baro_counter++;
  tof_counter++;
  if (bme280_is_present() && (baro_counter % BARO_READ_DECIM == 0)) {
    _next_op = IMU_OP_BARO;
  } else if (vl53l0x_is_present() && (tof_counter % TOF_READ_DECIM == 0)) {
    _next_op = IMU_OP_TOF;
  } else {
    _next_op = IMU_OP_FAST;
  }

  int higher_priority_task_woken = 0;
  v_semaphore_give_from_isr(bmx160_ready_sema, &higher_priority_task_woken);
  if (higher_priority_task_woken) {
    task_yield();
  }
}

/* BME280 data-register burst complete: hand the 8 raw bytes to the baro driver
 * (cheap copy + flag; compensation runs in bme280_read_task) and return the
 * acquisition chain to FAST. Mirrors the mag/temp callbacks. */
/** @implements SNS-BMX-105 */
static void bmx160_dma_callback_baro(void *args) {
  if (args != NULL) {
    bme280_ingest_raw((const uint8_t *)args);
  }
  isr_count++;
  _next_op = IMU_OP_FAST;

  int higher_priority_task_woken = 0;
  v_semaphore_give_from_isr(bmx160_ready_sema, &higher_priority_task_woken);
  if (higher_priority_task_woken) {
    task_yield();
  }
}

/* VL53L0X result-block burst complete: hand the 12 raw bytes to the ToF driver
 * (cheap copy + flag; decode runs in vl53l0x_read_task) and return the
 * acquisition chain to FAST. Mirrors the baro callback. */
/** @noreq ISR glue for the ToF ride-along slot. */
static void bmx160_dma_callback_tof(void *args) {
  if (args != NULL) {
    vl53l0x_ingest_raw((const uint8_t *)args);
  }
  isr_count++;
  _next_op = IMU_OP_FAST;

  int higher_priority_task_woken = 0;
  v_semaphore_give_from_isr(bmx160_ready_sema, &higher_priority_task_woken);
  if (higher_priority_task_woken) {
    task_yield();
  }
}

// Keep the old callback for compatibility if needed, but it's now unused
/** @noreq Legacy compatibility wrapper (unused). */
void bmx160_dma_callback(void *args) { bmx160_dma_callback_fast(args); }

/* HIGH_FREQ_TIMER tick (registered at IMU_FAST_PERIOD_US): releases the read
 * task to start one FAST (accel/gyro) read, pacing the IMU to
 * IMU_SAMPLE_FREQ_HZ instead of free-running at I2C speed. Binary sema, so
 * ticks that arrive while the task is mid-cycle coalesce (caps, never queues). */
/** @implements SNS-IMU-001 */
void bmx160_fast_tick_isr(void) {
  if (bmx160_fast_tick_sema == NULL)
    return;
  int higher_priority_task_woken = 0;
  v_semaphore_give_from_isr(bmx160_fast_tick_sema, &higher_priority_task_woken);
  if (higher_priority_task_woken)
    task_yield();
}

static uint32_t _last_read_time = 0;
static uint32_t _read_count = 0;
/*
 * According to test done on 03/04/2026, the IMU loop cycles are around 20128 at
 * 84MHz So the effective frequency is around 4.2kHz Time taken is around 237
 * microseconds
 */
/* BMM150 compensation + calibration + disturbance check + unit-normalization.
 * Split out of bmx160_process_data so it can run only when fresh mag data
 * arrived (see _mag_fresh). Writes the converted mag[] and mag_fusion[] fields
 * and updates last_mag for the next disturbance comparison. */
/** @implements SNS-MAG-002, SNS-MAG-101 */
static void bmx160_process_mag(int16_t mx, int16_t my, int16_t mz,
                               uint16_t rhall, uint8_t is_mag_invalid) {
  // Align BMM150 axes to BMX160 body frame: [-Y, X, Z]
  float mag_x = -bmm150_compensate_y(my, rhall);
  float mag_y = -bmm150_compensate_x(mx, rhall);
  float mag_z = -bmm150_compensate_z(mz, rhall);

  // Store compensated but unscaled data for calibration
  _bmx_data.converted.mag_compensated[0] = mag_x;
  _bmx_data.converted.mag_compensated[1] = mag_y;
  _bmx_data.converted.mag_compensated[2] = mag_z;

  uint8_t mag_fusion_valid = 1;
  if (!IS_FINITE(mag_x) || !IS_FINITE(mag_y) || !IS_FINITE(mag_z) ||
      is_mag_invalid) {
    mag_fusion_valid = 0;
  }

  // Calibrated magnetometer in microtesla. This is the value the getters and
  // telemetry report — it is NOT normalized; fusion uses mag_fusion[] below.
  // Full hard-iron (offset) + soft-iron (3x3) correction: v = M * (m - bias).
  float md0 = mag_x - bmx160_calib.mag_offset[0];
  float md1 = mag_y - bmx160_calib.mag_offset[1];
  float md2 = mag_z - bmx160_calib.mag_offset[2];
  const float *Msi = bmx160_calib.mag_soft_iron;
  _bmx_data.converted.mag[0] = Msi[0] * md0 + Msi[1] * md1 + Msi[2] * md2;
  _bmx_data.converted.mag[1] = Msi[3] * md0 + Msi[4] * md1 + Msi[5] * md2;
  _bmx_data.converted.mag[2] = Msi[6] * md0 + Msi[7] * md1 + Msi[8] * md2;

  // Magnitude and Disturbance Checks (on the calibrated uT vector)
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
        (m_fabsf(last_mag[0]) > 0.0f || m_fabsf(last_mag[1]) > 0.0f ||
         m_fabsf(last_mag[2]) > 0.0f)) {
      // Calculate dot product of current reading and last valid reading
      float dot = _bmx_data.converted.mag[0] * last_mag[0] +
                  _bmx_data.converted.mag[1] * last_mag[1] +
                  _bmx_data.converted.mag[2] * last_mag[2];

      // A dot product below 0.75 * norm^2 is a large directional jump
      // (mag updates at ~20 Hz, so consecutive readings should be close).
      float norm_curr_sq = mag_norm * mag_norm;
      if (dot < 0.75f * norm_curr_sq) { // Significant directional jump
        mag_fusion_valid = 0;
      }
    }
  }

  // Build the unit-normalized vector the estimator consumes, leaving the
  // calibrated uT in converted.mag[] intact for the getters/telemetry.
  if (mag_fusion_valid) {
    float mag_norm =
        SQRT_F(_bmx_data.converted.mag[0] * _bmx_data.converted.mag[0] +
               _bmx_data.converted.mag[1] * _bmx_data.converted.mag[1] +
               _bmx_data.converted.mag[2] * _bmx_data.converted.mag[2]);
    if (mag_norm > 0.001f) {
      _bmx_data.converted.mag_fusion[0] = _bmx_data.converted.mag[0] / mag_norm;
      _bmx_data.converted.mag_fusion[1] = _bmx_data.converted.mag[1] / mag_norm;
      _bmx_data.converted.mag_fusion[2] = _bmx_data.converted.mag[2] / mag_norm;

      // Remember the calibrated uT vector (unnormalized) for the next dot
      // product disturbance check.
      last_mag[0] = _bmx_data.converted.mag[0];
      last_mag[1] = _bmx_data.converted.mag[1];
      last_mag[2] = _bmx_data.converted.mag[2];
    } else {
      mag_fusion_valid = 0;
    }
  }

  if (!mag_fusion_valid) {
    // Skip the magnetometer in fusion by passing a zero vector to the
    // estimator. The calibrated uT in converted.mag[] is preserved.
    _bmx_data.converted.mag_fusion[0] = 0.0f;
    _bmx_data.converted.mag_fusion[1] = 0.0f;
    _bmx_data.converted.mag_fusion[2] = 0.0f;
  }
}

/** @implements SNS-IMU-001, SNS-IMU-002, SNS-BMX-106, SNS-LPF-101, SNS-CAL-002 */
void bmx160_process_data(void) {
  /* Stamp the sample at acquisition (DWT cycles). All downstream dt is derived
   * from deltas of this stamp, not a DWT read at consume time — see
   * vayu_dt_from_cycles(). */
  uint32_t _sample_cyc = hal_cycle_counter_get();
  _read_count++;
  if (_read_count % 1000 == 0) {
    _last_read_time = v_get_ticks();
    _read_count = 0;
  }
  // 1. Extract mag (0-5)
  // X/Y are 13-bit, Z is 15-bit. Status bits are in the LSB.
  // We assemble as signed 16-bit and then arithmetic shift to preserve sign.
  uint8_t _is_mag_invalid = 0;
  int16_t mx = (int16_t)(((uint16_t)_bmx_dma_rx_buffer_double[1] << 8) |
                         _bmx_dma_rx_buffer_double[0]);
  mx >>= 3;

  int16_t my = (int16_t)(((uint16_t)_bmx_dma_rx_buffer_double[3] << 8) |
                         _bmx_dma_rx_buffer_double[2]);
  my >>= 3;

  int16_t mz = (int16_t)(((uint16_t)_bmx_dma_rx_buffer_double[5] << 8) |
                         _bmx_dma_rx_buffer_double[4]);
  mz >>= 1;

  // RHALL is at bytes 6, 7
  uint16_t rhall = (uint16_t)(((uint16_t)_bmx_dma_rx_buffer_double[7] << 8) |
                              (_bmx_dma_rx_buffer_double[6] & 0xFE)) >>
                   2;
  if (rhall < 50 || rhall > 30000) {
    _is_mag_invalid = 1;
  }
  // 2. Extract gyr (8-13)
  int16_t gx = (int16_t)(((uint16_t)_bmx_dma_rx_buffer_double[9] << 8) |
                         _bmx_dma_rx_buffer_double[8]);
  int16_t gy = (int16_t)(((uint16_t)_bmx_dma_rx_buffer_double[11] << 8) |
                         _bmx_dma_rx_buffer_double[10]);
  int16_t gz = (int16_t)(((uint16_t)_bmx_dma_rx_buffer_double[13] << 8) |
                         _bmx_dma_rx_buffer_double[12]);

  // 3. Extract acc (14-19)
  int16_t ax = (int16_t)(((uint16_t)_bmx_dma_rx_buffer_double[15] << 8) |
                         _bmx_dma_rx_buffer_double[14]);
  int16_t ay = (int16_t)(((uint16_t)_bmx_dma_rx_buffer_double[17] << 8) |
                         _bmx_dma_rx_buffer_double[16]);
  int16_t az = (int16_t)(((uint16_t)_bmx_dma_rx_buffer_double[19] << 8) |
                         _bmx_dma_rx_buffer_double[18]);

  int16_t raw_temp = (int16_t)(((uint16_t)_bmx_dma_rx_buffer_double[29] << 8) |
                               _bmx_dma_rx_buffer_double[28]);

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

  /* High-speed SD stream, tapped HERE: sensor-native counts, before the LPF,
   * the bias correction and the axis remap below. Pre-filter is the entire
   * point -- the recording exists to show what the filters should be removing.
   * RAM-only, ~12 bytes of memcpy, no-op unless armed. */
  imu_hs_log_sample(_bmx_data.raw.gyr, _bmx_data.raw.acc, _sample_cyc);

  // Convert to units (for local attitude fusion and telemetry)
  _bmx_data.converted.acc_raw[0] = -bmx160_raw_acc_to_mps2(ax);
  _bmx_data.converted.acc_raw[1] = bmx160_raw_acc_to_mps2(ay);
  _bmx_data.converted.acc_raw[2] = -bmx160_raw_acc_to_mps2(az);

  // Initial populate (will be calibrated/filtered later)
  _bmx_data.converted.acc[0] = _bmx_data.converted.acc_raw[0];
  _bmx_data.converted.acc[1] = _bmx_data.converted.acc_raw[1];
  _bmx_data.converted.acc[2] = _bmx_data.converted.acc_raw[2];

  _bmx_data.converted.gyr_raw[0] = -bmx160_raw_gyr_to_dps(gx);
  _bmx_data.converted.gyr_raw[1] = bmx160_raw_gyr_to_dps(gy);
  _bmx_data.converted.gyr_raw[2] = -bmx160_raw_gyr_to_dps(gz);

  _bmx_data.converted.gyr[0] = _bmx_data.converted.gyr_raw[0];
  _bmx_data.converted.gyr[1] = _bmx_data.converted.gyr_raw[1];
  _bmx_data.converted.gyr[2] = _bmx_data.converted.gyr_raw[2];

  // LPF
  for (int i = 0; i < 3; i++) {
    _bmx_data.converted.gyr[i] =
        lpf_apply(&gyr_lpf[i], _bmx_data.converted.gyr[i]);
  }

  // Apply gyro bias estimator. Compare squared magnitudes against squared
  // thresholds so the per-sample stillness check needs no sqrt: the magnitude
  // band |acc_mag - 9.81| < 0.2 (i.e. acc_mag in (9.61, 10.01)) and
  // gyro_norm < 0.2 are monotonic in the squared value, so this is exact.
  // Thresholds are the compile-time GYRO_STILL_SQ / ACC_STILL_*_SQ constants.
  float acc_sq = _bmx_data.converted.acc[0] * _bmx_data.converted.acc[0] +
                 _bmx_data.converted.acc[1] * _bmx_data.converted.acc[1] +
                 _bmx_data.converted.acc[2] * _bmx_data.converted.acc[2];
  float gyro_sq = _bmx_data.converted.gyr[0] * _bmx_data.converted.gyr[0] +
                  _bmx_data.converted.gyr[1] * _bmx_data.converted.gyr[1] +
                  _bmx_data.converted.gyr[2] * _bmx_data.converted.gyr[2];
  if (system_state_get() != SYSTEM_STATE_CALIBRATING) {
    if (acc_sq > ACC_STILL_LO_SQ && acc_sq < ACC_STILL_HI_SQ &&
        gyro_sq < GYRO_STILL_SQ) {
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

  /* Magnetometer compensation + normalization is the most expensive part of
   * the sample path (3x BMM150 compensation + 2 sqrt + disturbance dot/div),
   * but the BMM150 only refreshes ~1/13 of the FAST cadence. Run it only when
   * a MAG read just landed; otherwise the converted mag fields keep their last
   * value — identical to recomputing from the unchanged bytes. */
  if (_mag_fresh) {
    bmx160_process_mag(mx, my, mz, rhall, _is_mag_invalid);
    _mag_fresh = 0;
  }
  // Temperature drifts slowly and only refreshes on a TEMP read (~1/13 of the
  // FAST cadence); convert only when fresh, else keep the last value.
  if (_temp_fresh) {
    bmx160_convert_raw_temp_to_celcius(raw_temp, &_bmx_data.converted.temp);
    _temp_fresh = 0;
  }

  // Apply accelerometer calibration: a_cal = M_acc * (a_raw - bias), where M_acc
  // is the full 3x3 (scale + cross-axis misalignment). Same shape as the mag
  // soft-iron apply above. Then LPF (gyro was already filtered before its bias
  // correction). Compute into locals first — each output axis depends on all
  // three inputs, so we can't overwrite in place.
  {
    float a0 = _bmx_data.converted.acc[0] - bmx160_calib.acc_offset[0];
    float a1 = _bmx_data.converted.acc[1] - bmx160_calib.acc_offset[1];
    float a2 = _bmx_data.converted.acc[2] - bmx160_calib.acc_offset[2];
    const float *Asi = bmx160_calib.acc_soft_iron;
    _bmx_data.converted.acc[0] = Asi[0] * a0 + Asi[1] * a1 + Asi[2] * a2;
    _bmx_data.converted.acc[1] = Asi[3] * a0 + Asi[4] * a1 + Asi[5] * a2;
    _bmx_data.converted.acc[2] = Asi[6] * a0 + Asi[7] * a1 + Asi[8] * a2;
    for (int i = 0; i < 3; i++)
      _bmx_data.converted.acc[i] =
          lpf_apply(&acc_lpf[i], _bmx_data.converted.acc[i]);
  }

  // Tag the converted sample with its acquisition cycle stamp and fan it out.
  _bmx_data.converted.timestamp = _sample_cyc;

  imu_queue_telemetry_push(&_bmx_data);
  imu_queue_control_push(&_bmx_data);

  // During calibration the sample goes to the calibration consumer; no
  // attitude estimation runs. Keyed off _calib_active rather than the system
  // state: the RC watchdog may flip us to FAILSAFE mid-calibration (no RC link
  // on the bench), and we must keep feeding the calibration task regardless or
  // the calibration sample readers starve and hang.
  if (_calib_active) {
    imu_queue_calibration_push(&_bmx_data);
    return;
  }

  /* EST-MAH-002: feed the sample's validity flag into the estimator
   * health tracker. _is_mag_invalid (rhall ∉ [50, 30000]) is the
   * validity surface available today; SNS-IMU-002 will eventually
   * include I2C-read and over-saturation faults too. Stays in the driver:
   * it's a sample-acquisition concern. */
  estimator_mark_sample(!_is_mag_invalid);

  /* Attitude estimation runs in its own task now (attitude_task.c): hand off
   * the timestamped sample and let it run fusion + the safety step. */
  imu_queue_attitude_push(&_bmx_data);
}

/* Ellipsoid fit (offset + full 3x3) + its float linear-algebra helpers
 * moved to the shared, sensor-agnostic src/calib/calib_ellipsoid.c
 * (calib_fit_ellipsoid). */

/* Push a calibration telemetry packet [origin][nargs=1][code][float]. */
/** @noreq Calibration telemetry packet helper. */
static void calib_telemetry(uint8_t code, float value) {
  imu_calibration_telemetry_t t;
  t.buffer[0] = SYSTEM_ORIGIN_CALIBRATION;
  t.buffer[1] = 0x01;
  t.buffer[2] = code;
  v_memcpy(&t.buffer[3], &value, 4);
  t.size = 7;
  imu_queue_calibration_telemetry_push(&t);
}

/* Per-axis mag coverage (step 8): three floats in buffer[3],[7],[11], size 15 —
 * the layout navlink_tx_calibration unpacks into CALIBRATION_STATUS.coverage[3].
 * The GCS renders it as "COVERAGE: X.. Y.. Z..". */
/** @implements SYS-CAL-003 */
static void calib_coverage(float cx, float cy, float cz) {
  imu_calibration_telemetry_t t;
  t.buffer[0] = SYSTEM_ORIGIN_CALIBRATION;
  t.buffer[1] = 0x01;
  t.buffer[2] = CALIB_UPDATE_MAG_AXIS_COVERAGE;
  v_memcpy(&t.buffer[3], &cx, 4);
  v_memcpy(&t.buffer[7], &cy, 4);
  v_memcpy(&t.buffer[11], &cz, 4);
  t.size = 15;
  imu_queue_calibration_telemetry_push(&t);
}

/* ---- calib_engine descriptor callbacks (shared cancel + mag target) ------- */

/* Operator-cancel poll for the calib engine (every target uses this). */
/** @noreq Calib-engine cancel-poll callback (glue). */
static bool calib_cancelled(void *ctx) {
  (void)ctx;
  return _calib_cancel != 0;
}

/* Mag: pop one compensated sample (uT), gated on a sane rhall + finite values.
 * False = nothing usable this tick (the engine skips it). */
/** @implements SNS-CAL-103 */
static bool mag_read_raw(float v[3], void *ctx) {
  (void)ctx;
  bmx160_all_reading_t s;
  if (!imu_queue_calibration_pop(&s))
    return false;
  if (s.raw.rhall < 50 || s.raw.rhall > 30000)
    return false;
  float mx = s.converted.mag_compensated[0];
  float my = s.converted.mag_compensated[1];
  float mz = s.converted.mag_compensated[2];
  if (!IS_FINITE(mx) || !IS_FINITE(my) || !IS_FINITE(mz))
    return false;
  v[0] = mx;
  v[1] = my;
  v[2] = mz;
  return true;
}

/** @noreq Coverage-callback forwarder (glue). */
static void mag_on_coverage(float cx, float cy, float cz, void *ctx) {
  (void)ctx;
  calib_coverage(cx, cy, cz);
}

/* Commit a successful mag fit: hard iron in uT, soft iron used as-is. */
/** @implements SNS-CAL-103 */
static void mag_commit(const float offset[3], const float mat[9], void *ctx) {
  (void)ctx;
  for (int i = 0; i < 3; i++)
    bmx160_calib.mag_offset[i] = offset[i];
  for (int i = 0; i < 9; i++)
    bmx160_calib.mag_soft_iron[i] = mat[i];
  vayu_log("[CALIB] Mag Bias: %.2f, %.2f, %.2f", bmx160_calib.mag_offset[0],
           bmx160_calib.mag_offset[1], bmx160_calib.mag_offset[2]);
  vayu_log("[CALIB] Mag SoftIron diag: %.3f, %.3f, %.3f",
           bmx160_calib.mag_soft_iron[0], bmx160_calib.mag_soft_iron[4],
           bmx160_calib.mag_soft_iron[8]);
}

/* Commit a successful accel fit: bias (m/s^2) + full 3x3 (scale+misalignment). */
/** @implements SNS-CAL-101 */
static void acc_commit(const float offset[3], const float mat[9], void *ctx) {
  (void)ctx;
  for (int i = 0; i < 3; i++)
    bmx160_calib.acc_offset[i] = offset[i];
  for (int i = 0; i < 9; i++)
    bmx160_calib.acc_soft_iron[i] = mat[i];
  vayu_log("[CALIB] Accel Bias: %.3f, %.3f, %.3f", bmx160_calib.acc_offset[0],
           bmx160_calib.acc_offset[1], bmx160_calib.acc_offset[2]);
  vayu_log("[CALIB] Accel SoftIron diag: %.3f, %.3f, %.3f",
           bmx160_calib.acc_soft_iron[0], bmx160_calib.acc_soft_iron[4],
           bmx160_calib.acc_soft_iron[8]);
}

/* A captured hold is one of two structural kinds: a FACE (one body axis points
 * along gravity) or an EDGE/corner (gravity shared between axes, which is what
 * makes the 3x3 off-diagonal/misalignment terms observable). */
typedef enum { POSE_KIND_FACE, POSE_KIND_EDGE } pose_kind_t;
/* Coverage-gate outcomes: 0 banks the hold, negatives say why it was refused. */
enum { POSE_OK = 0, POSE_REJ_SHAPE = -1, POSE_REJ_DUP = -2 };

/* Dominant signed body axis of a direction: index 0..2 of the largest |component|,
 * with *sign set to +1/-1. Identifies which of the six signed axes a face lands on
 * without assuming any particular board mounting.
 * @implements SNS-CAL-104 */
static int dom_axis(const float v[3], int *sign) {
  int k = 0;
  if (m_fabsf(v[1]) > m_fabsf(v[k]))
    k = 1;
  if (m_fabsf(v[2]) > m_fabsf(v[k]))
    k = 2;
  *sign = (v[k] >= 0.0f) ? 1 : -1;
  return k;
}

/* Pose-coverage gate: decide whether a still hold `avg` (m/s^2) ADVANCES coverage
 * given the directions already banked. A FACE must have one axis clearly dominate
 * and land on a signed axis no prior face used; an EDGE must share gravity between
 * axes and sit far enough from every banked direction. This is what stops the same
 * orientation being recorded twice and keeps the 9-DOF fit well-conditioned.
 * `banked[0..n_banked)` are the accepted directions. Matching is geometric (not by
 * the prompted code), so it is independent of how the board axes are signed.
 * @implements SNS-CAL-104 */
static int pose_advances_coverage(const float avg[3], pose_kind_t kind,
                                  const float banked[][3], int n_banked) {
  float mag = m_sqrt(avg[0] * avg[0] + avg[1] * avg[1] + avg[2] * avg[2]);
  if (mag < 1.0f)
    return POSE_REJ_SHAPE; // degenerate; the stillness gate should preclude this
  float u[3] = {avg[0] / mag, avg[1] / mag, avg[2] / mag};
  float a0 = m_fabsf(u[0]), a1 = m_fabsf(u[1]), a2 = m_fabsf(u[2]);
  float amax = a0 > a1 ? a0 : a1;
  amax = amax > a2 ? amax : a2;
  float amin = a0 < a1 ? a0 : a1;
  amin = amin < a2 ? amin : a2;
  float amid = (a0 + a1 + a2) - amax - amin; // the middle |component|

  if (kind == POSE_KIND_FACE) {
    if (amax < ACCEL_POSE_FACE_DOMINANCE)
      return POSE_REJ_SHAPE; // no axis points along gravity -> not a clean face
    int sign, axis = dom_axis(u, &sign);
    for (int i = 0; i < n_banked; i++) {
      float bm =
          m_sqrt(banked[i][0] * banked[i][0] + banked[i][1] * banked[i][1] +
                 banked[i][2] * banked[i][2]);
      if (bm < 1.0f)
        continue;
      float bu[3] = {banked[i][0] / bm, banked[i][1] / bm, banked[i][2] / bm};
      float bmax = m_fabsf(bu[0]);
      if (m_fabsf(bu[1]) > bmax)
        bmax = m_fabsf(bu[1]);
      if (m_fabsf(bu[2]) > bmax)
        bmax = m_fabsf(bu[2]);
      if (bmax < ACCEL_POSE_FACE_DOMINANCE)
        continue; // a prior edge hold, not a face
      int bsign, baxis = dom_axis(bu, &bsign);
      if (baxis == axis && bsign == sign)
        return POSE_REJ_DUP; // this signed face is already banked
    }
    return POSE_OK;
  }

  // EDGE/corner: gravity must be shared, and the hold must be distinct.
  if (amax >= ACCEL_POSE_FACE_DOMINANCE)
    return POSE_REJ_SHAPE; // too face-like to add off-diagonal information
  if (amid < ACCEL_POSE_EDGE_MIN_SECOND)
    return POSE_REJ_SHAPE; // only one axis loaded -> not a shared-gravity edge
  for (int i = 0; i < n_banked; i++) {
    float bm =
        m_sqrt(banked[i][0] * banked[i][0] + banked[i][1] * banked[i][1] +
               banked[i][2] * banked[i][2]);
    if (bm < 1.0f)
      continue;
    float dot =
        (u[0] * banked[i][0] + u[1] * banked[i][1] + u[2] * banked[i][2]) / bm;
    if (dot > ACCEL_POSE_MIN_SEP_COS)
      return POSE_REJ_DUP; // within ~30 deg of a pose already taken
  }
  return POSE_OK;
}

/* Pose-tolerant static capture for the full-3x3 accel calibration. Prompts the
 * given pose code, then averages ACCEL_POSE_STILL_SAMPLES contiguous STILL raw
 * accel samples — "still" = gyro near zero AND |a| in a WIDE 1 g band (so a
 * grossly mis-scaled sensor still qualifies). The averaged hold must then pass the
 * pose-coverage gate (the right face/edge shape AND distinct from `banked[0..
 * n_banked)`); a hold that duplicates a prior orientation is refused and re-
 * prompted instead of banked. Averages acc_raw into accel_out (m/s^2). Returns 1
 * on success, -1 on cancel/starvation.
 * @implements SNS-CAL-104 */
static int wait_for_static_pose(uint8_t code, pose_kind_t kind,
                                const float banked[][3], int n_banked,
                                float accel_out[3]) {
  calib_telemetry(code, 0.0f); // prompt the operator (advisory)
  v_delay(CALIBRATION_WAIT_USER_TIME_PRE_CALIBRATION);

  const float g = 9.81f;
  const float mag_min2 = (0.45f * g) * (0.45f * g); // reject free-fall / drops
  const float mag_max2 = (2.0f * g) * (2.0f * g); // permissive: up to ~2x scale
  const float gyro_still2 = ACCEL_CAL_GYRO_STILL_DPS * ACCEL_CAL_GYRO_STILL_DPS;

  const int target = ACCEL_POSE_STILL_SAMPLES;
  const int reprompt_period = 50; // ~1 s; re-announce a dropped prompt
  const int starve_limit = 2500;  // ~5 s of empty queue -> abort cleanly

  /* Outer loop re-runs whenever a still hold fails the coverage gate (duplicate /
   * wrong shape): we discard it, re-prompt, and wait for a NEW orientation. */
  for (;;) {
    float sum[3] = {0.0f, 0.0f, 0.0f};
    int count = 0;
    int reprompt = 0;
    int starve = 0;

    while (count < target) {
      if (_calib_cancel)
        return -1;
      bmx160_all_reading_t s;
      if (!imu_queue_calibration_pop(&s)) {
        if (++starve >= starve_limit) {
          vayu_log("[CALIB] sample stream starved; aborting pose %d", code);
          return -1;
        }
        v_delay(2);
        continue;
      }
      starve = 0;
      float ax = s.converted.acc_raw[0], ay = s.converted.acc_raw[1],
            az = s.converted.acc_raw[2];
      float mag2 = ax * ax + ay * ay + az * az;
      float gx = s.converted.gyr_raw[0], gy = s.converted.gyr_raw[1],
            gz = s.converted.gyr_raw[2];
      float gyro2 = gx * gx + gy * gy + gz * gz;

      if (mag2 > mag_min2 && mag2 < mag_max2 && gyro2 < gyro_still2) {
        sum[0] += ax;
        sum[1] += ay;
        sum[2] += az;
        count++;
        if (count % (target / 20) == 0)
          calib_telemetry(CALIB_UPDATE_PROGRESS,
                          (100.0f * (float)count) / (float)target);
      } else {
        // Disturbed: require a contiguous still window, re-prompt periodically.
        count = 0;
        sum[0] = sum[1] = sum[2] = 0.0f;
        if (++reprompt >= reprompt_period) {
          reprompt = 0;
          calib_telemetry(code, 0.0f);
        }
      }
      v_delay(20);
    }

    float inv = 1.0f / (float)target;
    float avg[3] = {sum[0] * inv, sum[1] * inv, sum[2] * inv};

    /* Stillness passed; now the COVERAGE gate. Bank only an orientation that is
     * the right shape (face vs edge) AND distinct from every pose already taken —
     * otherwise re-prompt so the operator moves somewhere new. This is what stops
     * the same hold being recorded twice and keeps the ellipsoid well-posed. */
    int cov = pose_advances_coverage(avg, kind, banked, n_banked);
    if (cov == POSE_OK) {
      accel_out[0] = avg[0];
      accel_out[1] = avg[1];
      accel_out[2] = avg[2];
      return 1;
    }
    vayu_log("[CALIB] pose %d not banked (%s); hold a new orientation.", code,
             cov == POSE_REJ_DUP ? "duplicate" : "wrong shape");
    calib_telemetry(code, 0.0f); // re-show the target pose
    v_delay(CALIBRATION_WAIT_USER_TIME_PRE_CALIBRATION);
  }
}

/* Gyro bias capture (calib engine BIAS path). read_raw returns gyr_raw ONLY when
 * the board is still (same gyro+|a| gate as the accel capture) — a moving board
 * yields no accepted samples, so the engine times out and keeps the old offset
 * instead of latching a bad bias (the old routine averaged 500 samples blindly). */
/** @implements SNS-CAL-102 */
static bool gyr_read_raw_still(float v[3], void *ctx) {
  (void)ctx;
  bmx160_all_reading_t s;
  if (!imu_queue_calibration_pop(&s))
    return false;
  float ax = s.converted.acc_raw[0], ay = s.converted.acc_raw[1],
        az = s.converted.acc_raw[2];
  float mag2 = ax * ax + ay * ay + az * az;
  float gx = s.converted.gyr_raw[0], gy = s.converted.gyr_raw[1],
        gz = s.converted.gyr_raw[2];
  float gyro2 = gx * gx + gy * gy + gz * gz;
  const float g = 9.81f;
  if (mag2 < (0.45f * g) * (0.45f * g) || mag2 > (2.0f * g) * (2.0f * g))
    return false;
  if (gyro2 >= ACCEL_CAL_GYRO_STILL_DPS * ACCEL_CAL_GYRO_STILL_DPS)
    return false;
  v[0] = gx;
  v[1] = gy;
  v[2] = gz;
  return true;
}

/** @noreq Progress-callback forwarder (glue). */
static void gyr_on_progress(float pct, void *ctx) {
  (void)ctx;
  calib_telemetry(CALIB_UPDATE_PROGRESS, pct);
}

/** @implements SNS-CAL-102 */
static void gyr_commit(const float offset[3], const float mat[9], void *ctx) {
  (void)mat; // identity for a bias fit
  (void)ctx;
  for (int i = 0; i < 3; i++)
    bmx160_calib.gyr_offset[i] = offset[i];
  vayu_log("[CALIB] Gyro Bias: %.4f, %.4f, %.4f", bmx160_calib.gyr_offset[0],
           bmx160_calib.gyr_offset[1], bmx160_calib.gyr_offset[2]);
}

/** @implements SYS-CAL-001, SYS-CAL-002, SYS-CAL-003, SYS-CAL-004, SNS-CAL-001 */
void calibration_task(void *args) {
  calibration_args_t *cal_args = (calibration_args_t *)args;
  /* Set once the calibration is computed and persisted; drives the terminal
   * COMPLETE/FAILED status emitted at `done:`. Declared before the first goto so
   * every exit path sees a defined value. */
  int calib_ok = 0;
  /* args is NULL only when the command dispatcher's malloc failed; bail via the
   * shared cleanup epilogue instead of dereferencing cal_args. */
  if (cal_args == NULL) {
    vayu_log("[CALIB] no calibration args; aborting");
    goto done;
  }
  float imu_id = cal_args->imu_id;

  _calib_cancel = 0; // clear any stale cancel from a previous run
  /* Must actually enter CALIBRATING — otherwise bmx160_process_data never
   * diverts samples to the calibration queue and the capture loops would
   * spin forever. Bail cleanly if the state machine rejects the transition
   * (e.g. we're in FAILSAFE/ARMED, not STANDBY). */
  if (system_state_set(SYSTEM_STATE_CALIBRATING) != VAYU_OK) {
    vayu_log("[CALIB] cannot enter CALIBRATING from current state; aborting");
    goto done;
  }
  /* Latch sample diversion ON now that we are genuinely calibrating. Set after
   * the transition succeeds so a rejected entry never strands the diversion on.
   * From here, samples flow to the calibration queue even if the RC watchdog
   * flips the system state to FAILSAFE underneath us. */
  _calib_active = 1;
  v_delay(500);
  vayu_log("[CALIB] IMU ID: %.1f, Type: %.1f", imu_id, cal_args->type);

  if ((int)imu_id == 1) { // ACCEL (pose-tolerant full-3x3 ellipsoid)
    // ~12 prompted holds: the 6 faces + 6 edges/corners. The edge/corner holds
    // share gravity between axes, which is the only way the off-diagonal
    // (misalignment) terms of the 3x3 become observable. Poses are advisory —
    // the fit is magnitude-only, so a roughly-held pose contributes no error.
    static const uint8_t accel_poses[ACCEL_CAL_POSES] = {
        CALIB_UPDATE_UPRIGHT,    CALIB_UPDATE_UPSIDE_DOWN,
        CALIB_UPDATE_NOSE_UP,    CALIB_UPDATE_NOSE_DOWN,
        CALIB_UPDATE_RIGHT_DOWN, CALIB_UPDATE_LEFT_DOWN,
        CALIB_UPDATE_EDGE_1,     CALIB_UPDATE_EDGE_2,
        CALIB_UPDATE_EDGE_3,     CALIB_UPDATE_EDGE_4,
        CALIB_UPDATE_EDGE_5,     CALIB_UPDATE_EDGE_6};
    float pts[ACCEL_CAL_POSES][3];
    int npts = 0;

    /* SIXPOINT prompts only the 6 faces (the closed form needs nothing else);
     * ELLIPSOID prompts the faces + edges so its 3x3 misalignment terms become
     * observable. Both capture into the same pts[] and commit the same way. */
#if ACCEL_CALIB_METHOD == ACCEL_CALIB_SIXPOINT
    const int accel_n_prompt = ACCEL_CAL_FACE_POSES;
    vayu_log("[CALIB] Accel Calibration (6-side closed form, %d poses)...",
             accel_n_prompt);
#else
    const int accel_n_prompt = ACCEL_CAL_POSES;
    vayu_log("[CALIB] Accel Calibration (full 3x3 LSQ, %d poses)...",
             accel_n_prompt);
#endif
    for (int i = 0; i < accel_n_prompt; i++) {
      /* First ACCEL_CAL_FACE_POSES prompts are the 6 faces; the rest are the
       * edge/corner holds. The kind drives the coverage gate's shape test, and
       * pts[0..npts) are the directions already banked (so a repeat is refused). */
      pose_kind_t kind =
          (i < ACCEL_CAL_FACE_POSES) ? POSE_KIND_FACE : POSE_KIND_EDGE;
      if (wait_for_static_pose(accel_poses[i], kind, (const float (*)[3])pts,
                               npts, pts[npts]) < 0) {
        vayu_log("[CALIB] Accel calibration cancelled.");
        goto done;
      }
      npts++;
      vayu_log("[CALIB] Pose %d recorded.", i);
      v_delay(
          CALIBRATION_WAIT_USER_TIME_PRE_CALIBRATION); // user moves the board
    }

    calib_target_t acc_target = {
        .name = "accel",
        .radius = 9.80665f, // gravity (m/s^2) — absolute target
#if ACCEL_CALIB_METHOD == ACCEL_CALIB_SIXPOINT
        .fit = CALIB_FIT_SIXPOINT,
        .min_samples = ACCEL_CAL_FACE_POSES, // needs all 6 sides
#else
        .fit = CALIB_FIT_ELLIPSOID,
        .min_samples = ACCEL_CAL_MIN_POSES,
        .normalize_radius = true, // rescale so |a_cal| == g exactly
#endif
        .commit = acc_commit,
        .ctx = NULL,
    };
    if (calib_engine_fit_points(&acc_target, (const float (*)[3])pts, npts) !=
        0) {
      vayu_log("[CALIB] Accel fit failed; keeping old calibration.");
      goto done;
    }

  } else if ((int)imu_id == 2) { // GYRO (stillness-gated bias; orientation-
                                 // independent, so a single still hold)
    vayu_log("[CALIB] Gyro Calibration (stillness-gated)...");
    calib_telemetry(CALIB_UPDATE_UPRIGHT, 0.0f); // "hold still, level" prompt
    v_delay(CALIBRATION_WAIT_USER_TIME_PRE_CALIBRATION);

    calib_target_t gyr_target = {
        .name = "gyro",
        .fit = CALIB_FIT_BIAS,
        .min_samples = GYRO_CAL_STILL_SAMPLES,
        .max_ticks = GYRO_CAL_MAX_TICKS,
        .poll_ms = 20,
        .bias_var_max = GYRO_CAL_VAR_MAX,
        .read_raw = gyr_read_raw_still,
        .cancelled = calib_cancelled,
        .on_progress = gyr_on_progress,
        .commit = gyr_commit,
        .ctx = NULL,
    };
    if (calib_engine_run(&gyr_target) != 0) {
      vayu_log("[CALIB] Gyro calibration not committed; keeping old offset.");
      goto done;
    }

  } else if ((int)imu_id == 3) { // MAGNETOMETER (ellipsoid: hard + soft iron)
    vayu_log("[CALIB] Starting Magnetometer Calibration (Free-Rotation)...");
    calib_telemetry(CALIB_UPDATE_FREE_ROT, 0.0f);
    v_delay(1000); // give the user time to see the prompt

    // Online least-squares ellipsoid fit over the spin, run by the shared
    // calib engine: samples scaled by MAG_FIT_NORM, per-axis raw-span coverage,
    // early-finish at COV_DONE, commit-only-on-success. The RAW-span coverage is
    // offset-invariant (an uncalibrated hard-iron offset can pin m/|m| nearly
    // constant, so we never normalise the direction for coverage).
    calib_target_t mag_target = {
        .name = "mag",
        .fit = CALIB_FIT_ELLIPSOID,
        .radius = 50.0f,                    // nominal Earth field (uT)
        .min_samples = MAG_FIT_MIN_SAMPLES, // enough points to attempt the fit
        .cov_done = 80.0f,                  // per-axis % to finish early
        .max_ticks = 20000 / 20,            // 20 s at 20 ms/tick (fallback cap)
        .poll_ms = 20,
        .read_raw = mag_read_raw,
        .cancelled = calib_cancelled,
        .on_coverage = mag_on_coverage,
        .commit = mag_commit,
        .ctx = NULL,
    };
    if (calib_engine_run(&mag_target) != 0) {
      vayu_log(
          "[CALIB] Mag calibration not committed; keeping old calibration.");
      goto done;
    }

  } else if ((int)imu_id == 4) { // BOARD LEVEL / TRIM (mounting-tilt offset)
    /* Capture the gravity-derived roll/pitch while the FRAME is held level, and
     * store it as board_trim. The estimator subtracts this from its euler output
     * (attitude_task), so a cushion-tilted FC whose level != the prop plane still
     * reports and holds true level. Uses CALIBRATED accel (post accel-cal). */
    vayu_log(
        "[CALIB] Board-level (trim) calibration — hold the FRAME level...");
    calib_telemetry(CALIB_UPDATE_BOARD_LEVEL, 0.0f);
    v_delay(CALIBRATION_WAIT_USER_TIME_PRE_CALIBRATION);

    const int target = ACCEL_POSE_STILL_SAMPLES; /* contiguous still samples */
    const int max_ticks = 1500;                  /* generous cap (~15 s) */
    const float gg = 9.81f;
    const float gyro_still2 =
        ACCEL_CAL_GYRO_STILL_DPS * ACCEL_CAL_GYRO_STILL_DPS;
    const float amin2 = (0.6f * gg) * (0.6f * gg);
    const float amax2 = (1.4f * gg) * (1.4f * gg);
    float sum[3] = {0.0f, 0.0f, 0.0f};
    int count = 0, ticks = 0, reprompt = 0;
    while (count < target && ticks < max_ticks) {
      if (_calib_cancel) {
        vayu_log("[CALIB] Board-level cancelled.");
        goto done;
      }
      ticks++;
      bmx160_all_reading_t s;
      if (!imu_queue_calibration_pop(&s)) {
        v_delay(5);
        continue;
      }
      float ax = s.converted.acc[0], ay = s.converted.acc[1],
            az = s.converted.acc[2];
      float a2 = ax * ax + ay * ay + az * az;
      float gx = s.converted.gyr_raw[0], gy = s.converted.gyr_raw[1],
            gz = s.converted.gyr_raw[2];
      float g2 = gx * gx + gy * gy + gz * gz;
      if (a2 > amin2 && a2 < amax2 && g2 < gyro_still2) {
        sum[0] += ax;
        sum[1] += ay;
        sum[2] += az;
        count++;
        if (count % (target / 20 > 0 ? target / 20 : 1) == 0)
          calib_telemetry(CALIB_UPDATE_PROGRESS,
                          (100.0f * (float)count) / (float)target);
      } else {
        /* require a CONTIGUOUS still window; re-prompt periodically. */
        count = 0;
        sum[0] = sum[1] = sum[2] = 0.0f;
        if (++reprompt >= 50) {
          reprompt = 0;
          calib_telemetry(CALIB_UPDATE_BOARD_LEVEL, 0.0f);
        }
      }
      v_delay(10);
    }
    if (count < target) {
      vayu_log("[CALIB] Board-level: never settled; keeping old trim.");
      goto done;
    }
    float inv = 1.0f / (float)count;
    float ax = sum[0] * inv, ay = sum[1] * inv, az = sum[2] * inv;
    /* Gravity-derived tilt in the SAME ZYX convention the EKF uses. The driver
     * reports the GRAVITY vector, so a level board reads ~-g on Z (bmx160.c
     * negates X and Z at conversion) and the EKF models world-DOWN in body,
     * gb = R^T(0,0,-1) (see ekf.c: "using world-up here flips the estimate
     * 180deg"). World-UP in body is therefore -a/|a|, and for ZYX
     *   roll  = atan2(-ay, -az)
     *   pitch = atan2( ax, hypot(ay, az))
     * Deriving this from +a instead put a LEVEL board at roll ~180deg (az < 0),
     * which the sanity gate below then rejected — pitch looked fine throughout
     * because its hypot() term is sign-blind. */
    float roll = to_degrees(m_atan2(-ay, -az));
    float pitch = to_degrees(m_atan2(ax, m_sqrt(ay * ay + az * az)));
    /* Sanity: a real mount tilt is small; reject an absurd capture (frame wasn't
     * actually level) so we never latch a huge trim. */
    if (FABS_F(roll) > 30.0f || FABS_F(pitch) > 30.0f) {
      vayu_log("[CALIB] Board-level: %.1f/%.1f deg too large (not level?); "
               "kept old.",
               roll, pitch);
      goto done;
    }
    bmx160_calib.board_trim[0] = roll;
    bmx160_calib.board_trim[1] = pitch;
    vayu_log("[CALIB] Board trim: roll %.2f, pitch %.2f deg", roll, pitch);

  } else {
    vayu_log("[CALIB] Unknown IMU ID %d; nothing to do.", (int)imu_id);
    goto done;
  }

  vayu_log("[BMX] Calibration Done. Persisting.");
  v_delay(10);
  calib_telemetry(CALIB_UPDATE_PROGRESS, 100.0f);

  // Persist with a versioned header so a future format change (or an old
  // headerless file) is detected on load rather than misread. Hand a snapshot
  // to the centralised FS owner instead of writing SD inline: the actual
  // vfs_open/write/sync/close runs on the FS task, off this calibration task.
  // NOTE: calib_ok now means "successfully enqueued", not "written to SD" — the
  // terminal CALIB_UPDATE_COMPLETE telemetry fires on enqueue success.
  i2c_error_count = 0;
  {
    calib_file_header_t hdr = {CALIB_FILE_MAGIC, CALIB_FILE_VERSION,
                               (uint16_t)sizeof(bmx160_calibration_t)};
    calib_ok = fs_owner_enqueue_calib_save(&hdr, sizeof(hdr), &bmx160_calib,
                                           sizeof(bmx160_calibration_t));
    if (!calib_ok) {
      vayu_log("[CALIB] Failed to enqueue calibration save.");
    }
  }

done:
  /* Stop diverting samples to the calibration queue: the routine is tearing
   * down, so the estimator should get the stream back immediately. */
  _calib_active = 0;
  /* Terminal status for the GCS wizard. An explicit COMPLETE/FAILED ends the
   * wizard on a real event rather than inferring it from a STANDBY heartbeat —
   * which is correct because ending in FAILSAFE is a valid outcome, not a
   * failure. A user cancel is not a failure either (the GCS already tore the
   * wizard down locally), so suppress FAILED in that case. */
  if (calib_ok)
    calib_telemetry(CALIB_UPDATE_COMPLETE, 0.0f);
  else if (!_calib_cancel)
    calib_telemetry(CALIB_UPDATE_FAILED, 0.0f);

  /* Single exit path for every outcome — success, cancel, fit failure, save
   * failure, NULL args. Always restores STANDBY (so IMU samples resume flowing
   * to the estimator), clears the cancel flag, and frees the heap args. Fixes
   * the prior leak/wedge where a save failure returned early. The comm task
   * owns the task handle and clears it on cancel. */
  VAYU_DISCARD(system_state_set(SYSTEM_STATE_STANDBY));
  _calib_cancel = 0;
  if (cal_args)
    v_free(cal_args);
  task_exit();
}
