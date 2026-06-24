#include "sensor/bme280.h"
#include "sensor/i2c_manager.h"
#include "maths/maths_interface.h"
#include "storage/fs_owner.h"
#include "vaios.h"
#include <stdint.h>

/* Boot-time config writes can momentarily lose the bus; a couple of retries
 * with a 1 ms gap rides over that. (Only used during init — at runtime the
 * BME280 never drives the bus; the IMU DMA loop reads it.) */
#define BME280_I2C_RETRIES 4

/* Oversampling. Pressure gets the most (altitude); temp/humidity x1 is plenty. */
#define BME280_OSRS_T BME280_OSRS_X1
#define BME280_OSRS_P BME280_OSRS_X8
#define BME280_OSRS_H BME280_OSRS_X1

/* Compensation-task poll period. New raw bytes arrive ~15 Hz from the IMU loop;
 * polling at 20 ms keeps published values fresh without busy-waiting. */
#define BME280_TASK_PERIOD_MS 20

static bme280_calib_t _calib;
static volatile uint8_t _initialized = 0;
static uint8_t _have_sample = 0;
static int32_t _t_fine; /* carries temp into the pressure/humidity comp */
static float _sea_level_pa = 101325.0f; /* ISA standard sea-level pressure */

/* Raw 8-byte data block, fed by the IMU DMA callback (ISR) and consumed by the
 * compensation task. _raw_fresh is the single-word handshake (producer sets,
 * consumer clears). */
static volatile uint8_t _raw[BME280_DATA_LEN];
static volatile uint8_t _raw_fresh = 0;

/* Latest published sample. Single producer (bme280_read_task); getters copy
 * field-by-field. Each field is a naturally-aligned 32-bit word, so reads are
 * tear-free on Cortex-M4 (same lock-free convention as the BMX160 driver). */
static bme280_reading_t _last;

/* ---- low-level register access (boot only; blocking, bounded retry) ---- */

static hal_status_t bme280_read_regs(uint8_t reg, uint8_t *buf, uint16_t len) {
  for (int i = 0; i < BME280_I2C_RETRIES; i++) {
    if (i2c_manager_write_read(BME280_I2C_ADDR, &reg, 1, buf, len) == HAL_OK) {
      return HAL_OK;
    }
    v_delay(1);
  }
  return HAL_ERR_TIMEOUT;
}

static hal_status_t bme280_write_reg(uint8_t reg, uint8_t val) {
  uint8_t tx[2] = {reg, val};
  for (int i = 0; i < BME280_I2C_RETRIES; i++) {
    if (i2c_manager_write(BME280_I2C_ADDR, tx, 2) == HAL_OK) {
      return HAL_OK;
    }
    v_delay(1);
  }
  return HAL_ERR_TIMEOUT;
}

uint8_t bme280_get_chip_id(void) {
  uint8_t id = 0xFF;
  if (bme280_read_regs(BME280_REG_CHIP_ID, &id, 1) != HAL_OK) {
    return 0xFF;
  }
  return id;
}

uint8_t bme280_is_present(void) { return _initialized; }

/* ---- calibration ---- */

static hal_status_t bme280_read_calibration(void) {
  uint8_t tp[BME280_CALIB_TP_LEN];
  uint8_t h[BME280_CALIB_H_LEN];
  if (bme280_read_regs(BME280_REG_CALIB_TP, tp, BME280_CALIB_TP_LEN) != HAL_OK) {
    return HAL_ERR_TIMEOUT;
  }
  if (bme280_read_regs(BME280_REG_CALIB_H, h, BME280_CALIB_H_LEN) != HAL_OK) {
    return HAL_ERR_TIMEOUT;
  }

  /* Temperature/pressure block (0x88..0xA1) is little-endian (LSB first). */
  _calib.dig_t1 = (uint16_t)(tp[0] | (tp[1] << 8));
  _calib.dig_t2 = (int16_t)(tp[2] | (tp[3] << 8));
  _calib.dig_t3 = (int16_t)(tp[4] | (tp[5] << 8));
  _calib.dig_p1 = (uint16_t)(tp[6] | (tp[7] << 8));
  _calib.dig_p2 = (int16_t)(tp[8] | (tp[9] << 8));
  _calib.dig_p3 = (int16_t)(tp[10] | (tp[11] << 8));
  _calib.dig_p4 = (int16_t)(tp[12] | (tp[13] << 8));
  _calib.dig_p5 = (int16_t)(tp[14] | (tp[15] << 8));
  _calib.dig_p6 = (int16_t)(tp[16] | (tp[17] << 8));
  _calib.dig_p7 = (int16_t)(tp[18] | (tp[19] << 8));
  _calib.dig_p8 = (int16_t)(tp[20] | (tp[21] << 8));
  _calib.dig_p9 = (int16_t)(tp[22] | (tp[23] << 8));
  /* tp[24] is 0xA0 (reserved); dig_H1 is at 0xA1 = tp[25]. */
  _calib.dig_h1 = tp[25];

  /* Humidity block (0xE1..0xE7) — H4/H5 are nibble-packed (datasheet §4.2.2). */
  _calib.dig_h2 = (int16_t)(h[0] | (h[1] << 8));
  _calib.dig_h3 = h[2];
  _calib.dig_h4 = (int16_t)((h[3] << 4) | (h[4] & 0x0F));
  _calib.dig_h5 = (int16_t)((h[5] << 4) | (h[4] >> 4));
  _calib.dig_h6 = (int8_t)h[6];

  if (_calib.dig_t1 == 0 || _calib.dig_p1 == 0) {
    vayu_log("BME280: bad calibration (T1=%u P1=%u)", _calib.dig_t1,
             _calib.dig_p1);
    return HAL_ERR_NOT_INITIALIZED;
  }
  return HAL_OK;
}

/* ---- compensation (BME280 datasheet §4.2.3, fixed-point reference code) ---- */

/* Returns temperature in deg C, and updates _t_fine for pressure/humidity. */
static float bme280_compensate_temp(int32_t adc_t) {
  int32_t var1 = ((((adc_t >> 3) - ((int32_t)_calib.dig_t1 << 1))) *
                  ((int32_t)_calib.dig_t2)) >>
                 11;
  int32_t var2 = (((((adc_t >> 4) - ((int32_t)_calib.dig_t1)) *
                    ((adc_t >> 4) - ((int32_t)_calib.dig_t1))) >>
                   12) *
                  ((int32_t)_calib.dig_t3)) >>
                 14;
  _t_fine = var1 + var2;
  int32_t t = (_t_fine * 5 + 128) >> 8; /* deg C * 100 */
  return (float)t * 0.01f;
}

/* Returns pressure in Pa (64-bit path; result is Q24.8, i.e. p/256 = Pa). */
static float bme280_compensate_pressure(int32_t adc_p) {
  int64_t var1, var2, p;
  var1 = ((int64_t)_t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)_calib.dig_p6;
  var2 = var2 + ((var1 * (int64_t)_calib.dig_p5) << 17);
  var2 = var2 + (((int64_t)_calib.dig_p4) << 35);
  var1 = ((var1 * var1 * (int64_t)_calib.dig_p3) >> 8) +
         ((var1 * (int64_t)_calib.dig_p2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)_calib.dig_p1) >> 33;
  if (var1 == 0) {
    return 0.0f; /* avoid divide-by-zero */
  }
  p = 1048576 - adc_p;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)_calib.dig_p9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)_calib.dig_p8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)_calib.dig_p7) << 4);
  return (float)p / 256.0f;
}

/* Returns relative humidity in %RH (32-bit path; result is Q22.10, /1024). */
static float bme280_compensate_humidity(int32_t adc_h) {
  int32_t v = (_t_fine - ((int32_t)76800));
  v = (((((adc_h << 14) - (((int32_t)_calib.dig_h4) << 20) -
          (((int32_t)_calib.dig_h5) * v)) +
         ((int32_t)16384)) >>
        15) *
       (((((((v * ((int32_t)_calib.dig_h6)) >> 10) *
            (((v * ((int32_t)_calib.dig_h3)) >> 11) + ((int32_t)32768))) >>
           10) +
          ((int32_t)2097152)) *
             ((int32_t)_calib.dig_h2) +
         8192) >>
        14));
  v = (v - (((((v >> 15) * (v >> 15)) >> 7) * ((int32_t)_calib.dig_h1)) >> 4));
  v = (v < 0 ? 0 : v);
  v = (v > 419430400 ? 419430400 : v);
  return (float)((uint32_t)(v >> 12)) / 1024.0f;
}

/* ---- runtime data path (fed by the IMU DMA loop) ---- */

void bme280_ingest_raw(const uint8_t *data) {
  /* ISR context: copy the 8 bytes and flag fresh. Cheap on purpose. */
  for (int i = 0; i < BME280_DATA_LEN; i++) {
    _raw[i] = data[i];
  }
  _raw_fresh = 1;
}

void bme280_publish(float pressure_pa, float temperature_c, float humidity_rh) {
  /* Derive altitude from physical pressure and publish. The real-hardware path
   * calls this after compensating raw ADC; SITL calls it directly with vsim_d's
   * modelled pressure, so the SAME FC altitude derivation + telemetry run in
   * both (the host never supplies altitude). Barometric formula (ISA):
   * h = 44330 * (1 - (p/p0)^(1/5.255)). */
  float altitude_m =
      44330.0f * (1.0f - m_pow(pressure_pa / _sea_level_pa, 0.190294957f));

  _last.pressure_pa = pressure_pa;
  _last.temperature_c = temperature_c;
  _last.humidity_rh = humidity_rh;
  _last.altitude_m = altitude_m;
  _last.timestamp = hal_cycle_counter_get();
  _have_sample = 1;
}

static void bme280_compensate_and_publish(const uint8_t *d) {
  int32_t adc_p = (int32_t)(((uint32_t)d[0] << 12) | ((uint32_t)d[1] << 4) |
                            ((uint32_t)d[2] >> 4));
  int32_t adc_t = (int32_t)(((uint32_t)d[3] << 12) | ((uint32_t)d[4] << 4) |
                            ((uint32_t)d[5] >> 4));
  int32_t adc_h = (int32_t)(((uint32_t)d[6] << 8) | (uint32_t)d[7]);

  float temp_c = bme280_compensate_temp(adc_t); /* sets _t_fine first */
  float press_pa = bme280_compensate_pressure(adc_p);
  float hum_rh = bme280_compensate_humidity(adc_h);

  bme280_publish(press_pa, temp_c, hum_rh);
}

/* ---- public API ---- */

void bme280_set_sea_level_pa(float pa) {
  if (pa > 1.0f) {
    _sea_level_pa = pa;
  }
}

hal_status_t bme280_init(void) {
  _initialized = 0;
  _have_sample = 0;
  _raw_fresh = 0;

  uint8_t id = bme280_get_chip_id();
  if (id != BME280_CHIP_ID) {
    vayu_log("BME280: chip id 0x%02X != 0x60 (absent/mis-wired)", id);
    return HAL_ERR_NOT_INITIALIZED;
  }

  bme280_write_reg(BME280_REG_RESET, BME280_RESET_CMD);
  v_delay(10); /* startup time after reset (datasheet: ~2 ms) + margin */

  hal_status_t ret = bme280_read_calibration();
  if (ret != HAL_OK) {
    return ret;
  }

  /* Configure NORMAL (free-running) mode so the IMU loop can just read data
   * registers — no per-sample trigger needed. ctrl_hum must be written BEFORE
   * ctrl_meas to take effect (datasheet §5.4.3). */
  if (bme280_write_reg(BME280_REG_CTRL_HUM, BME280_OSRS_H) != HAL_OK) {
    return HAL_ERR_TIMEOUT;
  }
  if (bme280_write_reg(BME280_REG_CONFIG,
                       (uint8_t)((BME280_TSB_62_5MS << 5) |
                                 (BME280_FILTER_OFF << 2))) != HAL_OK) {
    return HAL_ERR_TIMEOUT;
  }
  uint8_t ctrl_meas =
      (uint8_t)((BME280_OSRS_T << 5) | (BME280_OSRS_P << 2) | BME280_MODE_NORMAL);
  if (bme280_write_reg(BME280_REG_CTRL_MEAS, ctrl_meas) != HAL_OK) {
    return HAL_ERR_TIMEOUT;
  }

  _initialized = 1; /* IMU loop may now schedule baro DMA reads */
  vayu_log("BME280: init ok, normal mode (osrs_p=%d)", BME280_OSRS_P);
  return HAL_OK;
}

void bme280_read_task(void *args) {
  (void)args;
  while (1) {
    if (_raw_fresh) {
      uint8_t snap[BME280_DATA_LEN];
      /* _raw is written only by the (higher-priority) IMU ISR. Clear the flag
       * first then copy — a re-fed sample just sets the flag again for the next
       * pass; worst case we recompensate one stale byte set, never tear. */
      _raw_fresh = 0;
      for (int i = 0; i < BME280_DATA_LEN; i++) {
        snap[i] = _raw[i];
      }
      bme280_compensate_and_publish(snap);
    }
    v_delay(BME280_TASK_PERIOD_MS);
  }
}

hal_status_t bme280_read_temperature(float *celsius) {
  if (celsius == NULL)
    return HAL_ERR_INVALID_ARG;
  if (!_have_sample)
    return HAL_ERR_NOT_INITIALIZED;
  *celsius = _last.temperature_c;
  return HAL_OK;
}

hal_status_t bme280_read_pressure(float *pascals) {
  if (pascals == NULL)
    return HAL_ERR_INVALID_ARG;
  if (!_have_sample)
    return HAL_ERR_NOT_INITIALIZED;
  *pascals = _last.pressure_pa;
  return HAL_OK;
}

hal_status_t bme280_read_humidity(float *percent_rh) {
  if (percent_rh == NULL)
    return HAL_ERR_INVALID_ARG;
  if (!_have_sample)
    return HAL_ERR_NOT_INITIALIZED;
  *percent_rh = _last.humidity_rh;
  return HAL_OK;
}

hal_status_t bme280_read_altitude(float *meters) {
  if (meters == NULL)
    return HAL_ERR_INVALID_ARG;
  if (!_have_sample)
    return HAL_ERR_NOT_INITIALIZED;
  *meters = _last.altitude_m;
  return HAL_OK;
}

hal_status_t bme280_read_all(bme280_reading_t *out) {
  if (out == NULL)
    return HAL_ERR_INVALID_ARG;
  if (!_have_sample)
    return HAL_ERR_NOT_INITIALIZED;
  *out = _last;
  return HAL_OK;
}
