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
/* Bus limits and timeouts. They lived in variables.h, which every file in
 * the tree included; they are this driver's numbers and nobody else's. */
#ifndef VAYU_I2C_MANAGER_H
#define VAYU_I2C_MANAGER_H

#include "navhal.h"
#include "driver/i2c_manager.h"

typedef enum {
  I2C_OP_WRITE,
  I2C_OP_READ,
  I2C_OP_WRITE_READ,
} i2c_op_type_t;

typedef enum {
  I2C_TRANS_BLANK, // not initialized can be used
  I2C_TRANS_IDLE,  // req in queue
  I2C_TRANS_BUSY,  // req being processed
  I2C_TRANS_DONE,  // req completed
  I2C_TRANS_ERROR, // req failed
} i2c_trans_state_t;

typedef struct {
  uint8_t id;
  uint8_t addr;
  uint8_t reg_addr;
  uint16_t tx_len;
  uint16_t rx_len;
  void (*callback)(void *); // for rx the void* has one byte of data len and
                            // then follow that many bytes of data
  i2c_op_type_t op_type;
  i2c_trans_state_t state;
} i2c_async_t;

/**
 * Bit-bang up to nine SCL pulses to free a slave holding SDA low.
 *
 * Rarely needed on its own: init_i2c_manager() already does this on every
 * attempt, so a caller that unsticks and then re-inits is pulsing the bus
 * twice. Use i2c_manager_recover() for that.
 */
void i2c_manager_unstick(void);

/**
 * Bring the bus up with @p cfg, and KEEP a copy of it.
 *
 * Also the recovery entry point -- it deinits and SWRSTs the peripheral, so
 * calling it again on a wedged bus actually resets it. The saved copy is what
 * makes i2c_manager_recover() possible, and the reason no other translation
 * unit needs to hold the configuration to restart the bus.
 */
hal_status_t init_i2c_manager(const hal_i2c_config_t *cfg);

/**
 * Restart the bus with the configuration it was given at boot.
 *
 * The recovery call for anything that is not the boot path. It exists because
 * the alternative -- every would-be recoverer reaching for the application's
 * `i2c_config` global -- put three different objects of that name in the tree
 * (main.c's, this file's static copy, and the onboard test firmware's) and
 * made a driver depend on a symbol defined in main.c.
 */
hal_status_t i2c_manager_recover(void);
hal_status_t i2c_manager_write(uint8_t addr, uint8_t *data, uint16_t len);
hal_status_t i2c_manager_read(uint8_t addr, uint8_t *data, uint16_t len);
hal_status_t i2c_manager_write_read(uint8_t addr, uint8_t *tx_data,
                                    uint16_t tx_len, uint8_t *rx_data,
                                    uint16_t rx_len);
hal_status_t i2c_manager_read_async(uint8_t addr, uint8_t reg_addr,
                                    uint16_t len, void (*callback)(void *));

#define MAX_I2C_DEVICES 10
#define I2C_MAX_TX_LEN 32
#define I2C_MAX_RX_LEN 64
#define I2C_MANAGER_SEMAPHORE_TIMEOUT 3 // ms
#define I2C_MANAGER_DMA_TIMEOUT 3       // ms

#endif // VAYU_I2C_MANAGER_H