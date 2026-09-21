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
#ifndef VAYU_BME280_H
#define VAYU_BME280_H

#include "navhal.h"
#include <stdint.h>

/*
 * Bosch BME280 humidity / pressure / temperature sensor.
 *
 * Wiring: shares I2C1 (PB8=SCL, PB9=SDA) with the BMX160 IMU — the bus is
 * already brought up by init_i2c_manager(); this driver only talks on it. The
 * BME280 is a slow sensor, so it uses the BLOCKING i2c_manager API (write /
 * write_read) rather than the IMU's high-rate DMA path; every transfer is
 * serialised by the i2c_manager bus mutex + busy flag, so the two coexist.
 *
 * Bus sharing: the BME280 does NOT independently drive the I2C bus at runtime
 * (that would contend with the IMU's high-rate DMA loop and trip its recovery
 * path). Instead it is configured ONCE at boot into NORMAL (free-running) mode
 * via the blocking i2c_manager API — safe because the IMU read task has not yet
 * started — and thereafter the IMU's own single-owner DMA state machine reads
 * the 8 data bytes as one more decimated slot (alongside MAG/TEMP) and hands
 * them back via bme280_ingest_raw(). One bus owner => zero contention, no IMU
 * cadence loss. Compensation runs off-ISR in bme280_read_task.
 *
 * Compensation follows the fixed-point reference code in the BME280 datasheet
 * (§4.2.3 / Appendix A): temp via the 32-bit path, pressure via the 64-bit path
 * (Q24.8 Pa), humidity via the 32-bit path (Q22.10 %RH).
 */

/* 7-bit address: SDO=GND -> 0x76 (this board), SDO=VDD -> 0x77. */
#define BME280_I2C_ADDR 0x76
#define BME280_CHIP_ID 0x60

/* Register map (datasheet §5.3). */
#define BME280_REG_CHIP_ID 0xD0
#define BME280_REG_RESET 0xE0
#define BME280_REG_CTRL_HUM 0xF2
#define BME280_REG_STATUS 0xF3
#define BME280_REG_CTRL_MEAS 0xF4
#define BME280_REG_CONFIG 0xF5
#define BME280_REG_DATA 0xF7 /* press(3) + temp(3) + hum(2) = 8 bytes */
#define BME280_REG_CALIB_TP                                                    \
  0x88 /* dig_T1..dig_P9 + dig_H1, 26 bytes (0x88..0xA1) */
#define BME280_REG_CALIB_H 0xE1 /* dig_H2..dig_H6, 7 bytes (0xE1..0xE7) */
#define BME280_CALIB_TP_LEN 26
#define BME280_CALIB_H_LEN 7

#define BME280_RESET_CMD 0xB6
#define BME280_STATUS_MEASURING 0x08 /* bit 3 of STATUS: conversion running */

/* Oversampling codes (osrs_*): 0=skip, 1=x1, 2=x2, 3=x4, 4=x8, 5=x16. */
#define BME280_OSRS_X1 0x01
#define BME280_OSRS_X4 0x03
#define BME280_OSRS_X8 0x04

/* ctrl_meas mode field. */
#define BME280_MODE_SLEEP 0x00
#define BME280_MODE_FORCED 0x01
#define BME280_MODE_NORMAL 0x03

/* config (0xF5): t_sb standby [7:5], filter [4:2]. Normal-mode standby sets the
 * internal sample period; 62.5 ms ~= 16 Hz. Filter off (no IIR). */
#define BME280_TSB_62_5MS 0x01 /* t_sb code (datasheet §5.4.6) */
#define BME280_FILTER_OFF 0x00

/* Number of data bytes read in one burst from BME280_REG_DATA (press+temp+hum). */
#define BME280_DATA_LEN 8

/* Factory calibration (datasheet §4.2.2). */
typedef struct {
  uint16_t dig_t1;
  int16_t dig_t2, dig_t3;
  uint16_t dig_p1;
  int16_t dig_p2, dig_p3, dig_p4, dig_p5, dig_p6, dig_p7, dig_p8, dig_p9;
  uint8_t dig_h1;
  int16_t dig_h2;
  uint8_t dig_h3;
  int16_t dig_h4, dig_h5;
  int8_t dig_h6;
} bme280_calib_t;

/* Latest published sample (single producer: bme280_read_task). */
typedef struct {
  float temperature_c; /* deg C */
  float pressure_pa;   /* Pa */
  float humidity_rh;   /* %RH */
  float altitude_m;    /* m above the configured sea-level reference */
  uint32_t
      timestamp; /* DWT cycle stamp at acquisition (see vayu_dt_from_cycles) */
} bme280_reading_t;

/* Probe the chip ID, soft-reset, read the calibration, and configure forced
 * mode. Returns HAL_ERR_NOT_INITIALIZED if the device is absent / mis-wired. */
hal_status_t bme280_init(void);

/* Read the WHO_AM_I (0x60 on a healthy BME280; 0xFF on a bus error). */
uint8_t bme280_get_chip_id(void);

/* True once init succeeded (chip present + calibrated + in normal mode). The
 * IMU read loop checks this before scheduling a baro DMA slot, so an absent
 * sensor is never read. ISR-safe (single volatile byte). */
uint8_t bme280_is_present(void);

/* Hand 8 raw data bytes (regs 0xF7..0xFE) to the driver. Called from the IMU
 * DMA-completion callback (ISR context) — must stay cheap: it only copies the
 * bytes and sets a "fresh" flag; compensation happens later in the read task. */
void bme280_ingest_raw(const uint8_t *data);

/* Compensation task — waits for fresh raw bytes (fed by the IMU loop), runs the
 * fixed-point compensation, and publishes. Register with task_create_named. */
void bme280_read_task(void *args);

/* Derive altitude from a physical pressure (Pa) + temperature (degC) + humidity
 * (%RH) reading and publish it as the latest sample. The on-hardware path calls
 * this after compensating raw ADC; SITL calls it directly with vsim_d's modelled
 * pressure so the same FC altitude/telemetry path runs without real I2C. */
void bme280_publish(float pressure_pa, float temperature_c, float humidity_rh);

/* Getters copy the latest published sample. Return HAL_OK, or
 * HAL_ERR_NOT_INITIALIZED before the first successful sample. */
hal_status_t bme280_read_temperature(float *celsius);
hal_status_t bme280_read_pressure(float *pascals);
hal_status_t bme280_read_humidity(float *percent_rh);
hal_status_t bme280_read_altitude(float *meters);
hal_status_t bme280_read_all(bme280_reading_t *out);

/* Sea-level reference pressure (Pa) used for the altitude estimate.
 * Defaults to the ISA standard 101325 Pa. */
void bme280_set_sea_level_pa(float pa);

#endif // VAYU_BME280_H
