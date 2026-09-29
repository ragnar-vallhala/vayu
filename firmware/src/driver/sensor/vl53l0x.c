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
#include "sensor/ride_along.h"
#include "sensor/sensor.h"
#include "driver/vl53l0x.h"
#include "hub/hub.h"
#include "driver/i2c_manager.h"
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
  uint8_t status = (uint8_t)((d[VL53L0X_OFF_RANGE_STATUS] >> 3) & 0x0F);
  uint16_t mm = (uint16_t)(((uint16_t)d[VL53L0X_OFF_RANGE_MM] << 8) |
                           (uint16_t)d[VL53L0X_OFF_RANGE_MM + 1]);

  /* status/range_mm track EVERY decode (the bring-up log wants to see the
   * out-of-range ones too); range_m and timestamp advance only on an in-window
   * sample, so a consumer can age-check `timestamp` to know the RANGE is fresh
   * rather than merely that the block was re-read. A rangefinder that quietly
   * serves its last good value while pointing at nothing is how you fly into
   * the ground. */
  _last.status = status;
  _last.range_mm = mm;

  /* Gate on the status AND the value. The value alone is not enough: a bench
   * run caught a 52 mm reading carrying status 8 (min range fail) -- inside
   * the sanity window, so a value-only gate published it as a good range. A
   * device-flagged fault that happens to land at a plausible distance is
   * exactly the reading that puts a phantom floor under an aircraft. */
  if (status == VL53L0X_DEV_STATUS_VALID && mm >= VL53L0X_RANGE_MIN_MM &&
      mm <= VL53L0X_RANGE_MAX_MM) {
    _last.range_m = (float)mm * 0.001f;
    _last.timestamp = hal_cycle_counter_get();
    _have_sample = 1;

    range_sample_t rs = {.range_m = _last.range_m,
                         .t_cyc = _last.timestamp,
                         .valid = 1,
                         .instance = 0};
    range_publish(&rs);
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
  /* Configuration detail only -- the adapter announces the outcome. */
  vayu_log("VL53L0X: continuous ranging (untuned defaults)");
  return HAL_OK;
}

/** @noreq Off-ISR decode of the raw result block fed by the IMU loop. */
void vl53l0x_read_task(void *args) {
  (void)args;
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

/* ---- Sensor-adapter registration ---------------------------------------
 * Stack 1024 words, not 640: measured peak 468 on hardware (range read plus
 * the quaternion tilt projection), and 640 would leave only 172 B free --
 * inside the 256 B TASK_STACK_OVERFLOW_THRESHOLD guard band. */
static vayu_status_t _vl53l0x_probe(void) {
  return vl53l0x_init() == HAL_OK ? VAYU_OK : VAYU_ERR_FAULT;
}

VAYU_SENSOR_DRIVER(vl53l0x_sensor) = {
    .name = "vl53l0x",
    .kind = SENSOR_RANGE,
    .instance = 0,
    .probe = _vl53l0x_probe,
    .task = vl53l0x_read_task,
    .task_name = "tof_read",
    .stack_words = 1024,
    .priority = 0,
};

/* Rides the IMU's single-owner I2C loop; see sensor/ride_along.h. */
VAYU_SENSOR_RIDE(vl53l0x_ride) = {
    .addr = VL53L0X_I2C_ADDR,
    .reg = VL53L0X_REG_BURST_START,
    .len = VL53L0X_DATA_LEN,
    .every_n = VL53L0X_RIDE_EVERY_N,
    .present = vl53l0x_is_present,
    .ingest = vl53l0x_ingest_raw,
};
