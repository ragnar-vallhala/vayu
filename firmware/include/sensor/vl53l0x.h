#ifndef VAYU_VL53L0X_H
#define VAYU_VL53L0X_H

#include "navhal.h"
#include <stdint.h>

/*
 * ST VL53L0X time-of-flight rangefinder — downward AGL height (~30 mm .. 2 m).
 *
 * Wiring: THIRD device on I2C1 (PB8=SCL, PB9=SDA), alongside the BMX160 IMU
 * @0x68 and the BME280 baro @0x76. The bus is already brought up by
 * init_i2c_manager(); this driver only talks on it. Do NOT fit a second set of
 * bus pull-ups — the existing devices already have them.
 *
 * Bus sharing: identical contract to bme280.h. The VL53L0X does NOT drive the
 * bus at runtime (that would contend with the IMU's high-rate DMA loop and trip
 * its recovery path). It is configured ONCE at boot into back-to-back
 * CONTINUOUS ranging via the BLOCKING i2c_manager API — safe because the IMU
 * read task has not started yet — and thereafter the BMX160's single-owner DMA
 * state machine reads its 12-byte result block as one more decimated slot
 * (alongside MAG/TEMP/BARO) and hands the bytes back via vl53l0x_ingest_raw().
 * One bus owner => zero contention. Decode runs off-ISR in vl53l0x_read_task.
 *
 * ponytail: MINIMAL init — chip-ID probe, GPIO interrupt disabled, start
 * continuous. Skips ST's SPAD management / tuning-settings blob / reference
 * calibration (VL53L0X_DataInit + StaticInit, ~200 register writes), running on
 * the chip's NVM defaults instead. Ceiling: shorter usable range and worse
 * accuracy under ambient IR than a fully tuned part. Upgrade path: port the
 * Pololu-distilled init sequence into a vl53l0x_apply_tuning() and call it
 * before vl53l0x_start_continuous().
 *
 * ponytail: NO per-sample interrupt clear. The textbook continuous-mode flow
 * writes SYSTEM_INTERRUPT_CLEAR (0x0B) after every sample, but the async
 * i2c_manager path is READ-ONLY (i2c_manager_read_async; its DMA is P2M), so
 * the IMU loop physically cannot write. Instead the GPIO interrupt is disabled
 * at boot and the result registers are polled. Ceiling: UNVERIFIED ON HARDWARE
 * — if range freezes after the first sample, the device is stalling on the
 * latched interrupt, and the fix is an async WRITE op in i2c_manager
 * (i2c_async_t already declares I2C_OP_WRITE; only I2C_OP_READ is implemented).
 */

/* 7-bit address. Factory default; re-addressable at runtime via reg 0x8A if a
 * second unit is ever fitted. No clash with the IMU (0x68) or baro (0x76). */
#define VL53L0X_I2C_ADDR 0x29
#define VL53L0X_MODEL_ID 0xEE

/* Register map (ST API register names). */
#define VL53L0X_REG_SYSRANGE_START 0x00
#define VL53L0X_REG_INT_CONFIG_GPIO 0x0A  /* 0 = interrupt disabled */
#define VL53L0X_REG_INT_CLEAR 0x0B
#define VL53L0X_REG_RESULT_INT_STATUS 0x13
#define VL53L0X_REG_RESULT_RANGE 0x14     /* 12-byte block 0x14..0x1F */
#define VL53L0X_REG_MODEL_ID 0xC0

/* SYSRANGE_START bits: 0x01 = single shot, 0x02 = back-to-back continuous. */
#define VL53L0X_SYSRANGE_CONTINUOUS 0x02

/* Bytes pulled in one burst from VL53L0X_REG_RESULT_RANGE: byte 0 carries the
 * device range status in bits [6:3]; bytes 10..11 are the range in mm (BE). */
#define VL53L0X_DATA_LEN 12

/* Sanity window on the decoded range. The device reports 8190/8191 mm as its
 * "no target / out of range" sentinel, and sub-30 mm readings are unreliable. */
#define VL53L0X_RANGE_MIN_MM 30
#define VL53L0X_RANGE_MAX_MM 2400

/* Latest published sample (single producer: vl53l0x_read_task). */
typedef struct {
  float range_m;      /* metres to the target, sanity-window checked */
  uint8_t status;     /* raw device range status, reg 0x14 bits [6:3] */
  uint16_t range_mm;  /* undecorated device value, for bring-up logging */
  uint32_t timestamp; /* DWT cycle stamp at acquisition */
} vl53l0x_reading_t;

/* Probe the model ID and start continuous ranging. Returns
 * HAL_ERR_NOT_INITIALIZED if the device is absent / mis-wired — a missing ToF
 * is NOT fatal, the FC just flies without it. */
hal_status_t vl53l0x_init(void);

/* Read the model ID (0xEE on a healthy VL53L0X; 0xFF on a bus error). */
uint8_t vl53l0x_get_model_id(void);

/* True once init succeeded. The IMU read loop checks this before scheduling a
 * ToF DMA slot, so an absent sensor is never read — without the guard its NACK
 * would trip the IMU hard-recovery path. ISR-safe (single volatile byte). */
uint8_t vl53l0x_is_present(void);

/* Hand 12 raw result bytes (regs 0x14..0x1F) to the driver. Called from the IMU
 * DMA-completion callback (ISR context) — stays cheap: copies the bytes and
 * sets a "fresh" flag; decode happens later in the read task. */
void vl53l0x_ingest_raw(const uint8_t *data);

/* Decode task — waits for fresh raw bytes (fed by the IMU loop), decodes and
 * publishes. Register with task_create_named. */
void vl53l0x_read_task(void *args);

/* Getters copy the latest published sample. Return HAL_OK, or
 * HAL_ERR_NOT_INITIALIZED before the first in-window sample. Note there is no
 * staleness check here: a consumer that acts on range (altitude hold, terrain
 * follow) MUST age-check `timestamp` itself before trusting it. */
hal_status_t vl53l0x_read_range(float *meters);
hal_status_t vl53l0x_read_all(vl53l0x_reading_t *out);

#endif // VAYU_VL53L0X_H
