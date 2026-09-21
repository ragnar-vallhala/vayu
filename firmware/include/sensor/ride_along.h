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
/**
 * @file sensor/ride_along.h
 * @brief Devices that share the IMU's single-owner I2C DMA loop.
 *
 * The bus has one owner and only that owner transacts (see i2c_bus_sharing in
 * the resource-ownership map). Other devices therefore take a turn in the
 * owner's rotation rather than driving the bus themselves. Until this tree
 * grew a bus arbiter, the owner was the IMU driver, and each ride-along's
 * address, register, burst length and cadence were written into `bmx160.c` --
 * so swapping the barometer meant editing the *accelerometer* driver.
 *
 * Here each device states its own bus facts and the owner reads the list. The
 * owner names no chip; a new ride-along is one descriptor plus one table
 * entry, and neither touches the acquisition loop.
 *
 * This deliberately mirrors a cyclic bus job: `every_n` is a period in slots,
 * `ingest` is a completion callback, and addr/reg/len are what a start()
 * routine would hand the HAL. When the arbiter lands, each driver registers
 * its own job and both this table and the owner's dispatch disappear -- the
 * descriptors are already the right shape to hand over.
 */
#ifndef VAYU_SENSOR_RIDE_ALONG_H
#define VAYU_SENSOR_RIDE_ALONG_H

#include <stdint.h>

/** One device's turn on the owner's bus. */
typedef struct {
  uint8_t addr;    /**< I2C device address                                  */
  uint8_t reg;     /**< first register of the burst                         */
  uint16_t len;    /**< bytes to read in one burst                          */
  uint16_t every_n;/**< slots between reads; the device's own cadence        */
  /** Skip the slot when absent -- a read to a missing device NACKs, and that
   *  trips the owner's bus-recovery path. */
  uint8_t (*present)(void);
  /** Burst complete, in ISR context: copy and flag, decode in a task. */
  void (*ingest)(const uint8_t *data);
} sensor_ride_t;

/**
 * The ride-along table, in contention order: when two devices come due on the
 * same slot the earlier entry wins and the later one waits a slot. Choosing
 * periods that are coprime makes that collision rare.
 *
 * @param count out: number of entries.
 */
const sensor_ride_t *sensor_rides(uint8_t *count);

#endif /* VAYU_SENSOR_RIDE_ALONG_H */
