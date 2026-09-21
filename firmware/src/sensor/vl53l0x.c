/*
 * Copyright (C) 2026 NAVRobotec Pvt Ltd
 * Author: Ragnar Vallhala
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "sensor/vl53l0x.h"
#include "sensor/i2c_manager.h"
#include "storage/fs_owner.h"
#include "vaios.h"
#include <stdint.h>

/* Boot-time config writes can momentarily lose the bus; a couple of retries
 * with a 1 ms gap rides over that. (Only used during init — at runtime the
 * VL53L0X never drives the bus; the IMU DMA loop reads it.) */
#define VL53L0X_I2C_RETRIES 4

/* Decode-task poll period. New raw bytes arrive ~30 Hz from the IMU loop;
 * polling at 20 ms keeps the published value fresh without busy-waiting. */
#define VL53L0X_TASK_PERIOD_MS 20

/* Bring-up: log status + raw mm once a second so a bench run shows whether the
 * sensor is actually ranging. Delete once a real consumer exists. */
#define VL53L0X_LOG_EVERY (1000 / VL53L0X_TASK_PERIOD_MS)

static volatile uint8_t _initialized = 0;
static uint8_t _have_sample = 0;

/* Raw 12-byte result block, fed by the IMU DMA callback (ISR) and consumed by
 * the decode task. _raw_fresh is the single-word handshake (producer sets,
 * consumer clears) — same convention as bme280.c. */
static volatile uint8_t _raw[VL53L0X_DATA_LEN];
static volatile uint8_t _raw_fresh = 0;

/* Latest published sample. Single producer (vl53l0x_read_task); getters copy
 * field-by-field, tear-free on Cortex-M4. */
static vl53l0x_reading_t _last;

/* ---- low-level register access (boot only; blocking, bounded retry) ---- */

/** @noreq Low-level I2C register read helper (boot only). */
static hal_status_t vl53l0x_read_regs(uint8_t reg, uint8_t *buf, uint16_t len) {
  for (int i = 0; i < VL53L0X_I2C_RETRIES; i++) {
    if (i2c_manager_write_read(VL53L0X_I2C_ADDR, &reg, 1, buf, len) == HAL_OK) {
      return HAL_OK;
    }
    v_delay(1);
  }
  return HAL_ERR_TIMEOUT;
}

/** @noreq Low-level I2C register write helper (boot only). */
static hal_status_t vl53l0x_write_reg(uint8_t reg, uint8_t val) {
  uint8_t tx[2] = {reg, val};
  for (int i = 0; i < VL53L0X_I2C_RETRIES; i++) {
    if (i2c_manager_write(VL53L0X_I2C_ADDR, tx, 2) == HAL_OK) {
      return HAL_OK;
    }
    v_delay(1);
  }
  return HAL_ERR_TIMEOUT;
}

/** @noreq Single-register model-ID read. */
uint8_t vl53l0x_get_model_id(void) {
  uint8_t id = 0xFF;
  if (vl53l0x_read_regs(VL53L0X_REG_MODEL_ID, &id, 1) != HAL_OK) {
    return 0xFF;
  }
  return id;
}

/** @noreq Presence flag read by the IMU loop before scheduling a ToF slot. */
uint8_t vl53l0x_is_present(void) { return _initialized; }

/* ---- runtime data path (fed by the IMU DMA loop) ---- */

/** @noreq ISR copy of the raw result block; glue into the shared IMU DMA loop. */
void vl53l0x_ingest_raw(const uint8_t *data) {
  /* ISR context: copy the 12 bytes and flag fresh. Cheap on purpose. */
  for (int i = 0; i < VL53L0X_DATA_LEN; i++) {
    _raw[i] = data[i];
  }
  _raw_fresh = 1;
}

/** @noreq Decode one result block and publish it if it passes the sanity window. */
static void vl53l0x_decode_and_publish(const uint8_t *d) {
  /* Device range status lives in bits [6:3] of the first status byte; the range
   * itself is a big-endian millimetre count at offset 10. */
  uint8_t status = (uint8_t)((d[0] >> 3) & 0x0F);
  uint16_t mm = (uint16_t)(((uint16_t)d[10] << 8) | (uint16_t)d[11]);

  /* status/range_mm track EVERY decode (the bring-up log wants to see the
   * out-of-range ones too); range_m and timestamp advance only on an in-window
   * sample, so a consumer can age-check `timestamp` to know the RANGE is fresh
   * rather than merely that the block was re-read. A rangefinder that quietly
   * serves its last good value while pointing at nothing is how you fly into
   * the ground. */
  _last.status = status;
  _last.range_mm = mm;

  /* Gate on the value, not the status code: an untuned part reports a mix of
   * device statuses and the published range must never be a sentinel. Tighten
   * to a status whitelist once a bench run shows what this unit actually
   * reports (the 1 Hz bring-up log below prints it). */
  if (mm >= VL53L0X_RANGE_MIN_MM && mm <= VL53L0X_RANGE_MAX_MM) {
    _last.range_m = (float)mm * 0.001f;
    _last.timestamp = hal_cycle_counter_get();
    _have_sample = 1;
  }
}

/* ---- public API ---- */

/** @noreq Boot-time bring-up of the ToF rangefinder on the shared I2C1 bus. */
hal_status_t vl53l0x_init(void) {
  _initialized = 0;
  _have_sample = 0;
  _raw_fresh = 0;

  uint8_t id = vl53l0x_get_model_id();
  if (id != VL53L0X_MODEL_ID) {
    vayu_log("VL53L0X: model id 0x%02X != 0xEE (absent/mis-wired)", id);
    return HAL_ERR_NOT_INITIALIZED;
  }

  /* Disable the GPIO interrupt and clear any latched one. The IMU loop can only
   * READ, so nothing will ever clear an interrupt again after this point — see
   * the ponytail note in vl53l0x.h. */
  if (vl53l0x_write_reg(VL53L0X_REG_INT_CONFIG_GPIO, 0x00) != HAL_OK) {
    return HAL_ERR_TIMEOUT;
  }
  if (vl53l0x_write_reg(VL53L0X_REG_INT_CLEAR, 0x01) != HAL_OK) {
    return HAL_ERR_TIMEOUT;
  }

  /* Back-to-back continuous: the device free-runs and the IMU loop just reads
   * result registers — no per-sample trigger needed, exactly like the BME280 in
   * NORMAL mode. */
  if (vl53l0x_write_reg(VL53L0X_REG_SYSRANGE_START,
                        VL53L0X_SYSRANGE_CONTINUOUS) != HAL_OK) {
    return HAL_ERR_TIMEOUT;
  }

  _initialized = 1; /* IMU loop may now schedule ToF DMA reads */
  vayu_log("VL53L0X: init ok, continuous ranging (untuned defaults)");
  return HAL_OK;
}

/** @noreq Off-ISR decode of the raw result block fed by the IMU loop. */
void vl53l0x_read_task(void *args) {
  (void)args;
  uint32_t ticks = 0;
  while (1) {
    if (_raw_fresh) {
      uint8_t snap[VL53L0X_DATA_LEN];
      /* _raw is written only by the (higher-priority) IMU ISR. Clear the flag
       * first then copy — a re-fed sample just sets it again for the next pass,
       * so worst case we redecode one stale block, never tear. */
      _raw_fresh = 0;
      for (int i = 0; i < VL53L0X_DATA_LEN; i++) {
        snap[i] = _raw[i];
      }
      vl53l0x_decode_and_publish(snap);
    }
    if (_initialized && ++ticks % VL53L0X_LOG_EVERY == 0) {
      vayu_log("VL53L0X: %d mm (status %d)", (int)_last.range_mm,
               (int)_last.status);
    }
    v_delay(VL53L0X_TASK_PERIOD_MS);
  }
}

/** @noreq Trivial published-value accessor. */
hal_status_t vl53l0x_read_range(float *meters) {
  if (meters == NULL)
    return HAL_ERR_INVALID_ARG;
  if (!_have_sample)
    return HAL_ERR_NOT_INITIALIZED;
  *meters = _last.range_m;
  return HAL_OK;
}

/** @noreq Trivial published-value accessor. */
hal_status_t vl53l0x_read_all(vl53l0x_reading_t *out) {
  if (out == NULL)
    return HAL_ERR_INVALID_ARG;
  if (!_have_sample)
    return HAL_ERR_NOT_INITIALIZED;
  *out = _last;
  return HAL_OK;
}
