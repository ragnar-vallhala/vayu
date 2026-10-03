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
#include "vayu_board.h"
#include "driver/i2c_manager.h"
#include "ipc.h"
#include "navhal.h"
#include "port.h"
#include "utils.h"
#include "storage/fs_owner.h"
#include "sys/clock.h" /* vayu_clock_hz, vayu_clock_cycles */
#include "vaios.h"
#include "driver/i2c_manager.h"
#include <stdint.h>

/* The configuration this bus was brought up with. Held here so recovery
 * needs nothing from the caller -- see i2c_manager_recover(). */
static hal_i2c_config_t _boot_config;
static SemaphoreHandle_t _i2c_sema; // Mutex for synchronous calls
static atomic_t _bus_busy;          // Atomic flag for bus availability
static i2c_async_t _current_trans;
static uint8_t _rx_data[I2C_MAX_RX_LEN]; // current transaction's rx data
static void i2c_manager_callback(void);

// Atomic bus acquisition (returns 1 if successful, 0 if busy)
// @noreq Internal bus-ownership primitive for the I2C manager.
static inline int i2c_manager_acquire_bus(void) {
  int acquired = 0;
  ENTER_CRITICAL();
  if (_bus_busy.counter == 0) {
    _bus_busy.counter = 1;
    acquired = 1;
  }
  EXIT_CRITICAL();
  return acquired;
}

// Atomic bus release
// @noreq Internal bus-ownership primitive for the I2C manager.
static inline void i2c_manager_release_bus(void) { atomic_set(&_bus_busy, 0); }

/* Bus-clear pulse timing.
 *
 * These used to be `for (volatile int j = 0; j < 200; j++)` spin loops, which
 * time out at whatever the core happens to run at -- about 14 us on this
 * 84 MHz part, so roughly 36 kHz. The same loop on the 400 MHz H7 this tree is
 * heading for (see firmware/board/README.md) would clock the bus five times
 * faster and out of spec, and bus-clear is exactly the path where a slave is
 * already confused.
 *
 * So the pulse is stated in microseconds and derived from the measured CPU
 * rate. 10 us half-period is 50 kHz: inside standard mode, and near where this
 * board has been running. */
#define I2C_UNSTICK_HALF_US 10u

/** @noreq bit-bang pulse timing; wrap-safe on the 32-bit cycle counter. */
static void _bitbang_delay_us(uint32_t us) {
  uint32_t per_us = vayu_clock_hz() / 1000000u;
  if (per_us == 0u) {
    per_us = 1u; /* a sub-MHz core is not a real configuration; do not hang */
  }
  const uint32_t ticks = per_us * us;
  const uint32_t t0 = vayu_clock_cycles();
  while ((vayu_clock_cycles() - t0) < ticks) {
  }
}

/** @implements SNS-I2C-102 */
void i2c_manager_unstick(void) {
  hal_gpio_set_mode(BOARD_I2C_SCL, HAL_GPIO_MODE_OUTPUT, HAL_GPIO_PULL_UP);
  hal_gpio_set_mode(BOARD_I2C_SDA, HAL_GPIO_MODE_OUTPUT, HAL_GPIO_PULL_UP);
  hal_gpio_set_output_type(BOARD_I2C_SCL, HAL_GPIO_OTYPE_OPEN_DRAIN);
  hal_gpio_set_output_type(BOARD_I2C_SDA, HAL_GPIO_OTYPE_OPEN_DRAIN);

  hal_gpio_write(BOARD_I2C_SDA, HAL_GPIO_HIGH);
  _bitbang_delay_us(I2C_UNSTICK_HALF_US);

  /* Nine pulses: enough for a slave stuck mid-byte to finish clocking it out
   * and release SDA, whichever bit it stopped on. */
  for (int i = 0; i < 9; ++i) {
    hal_gpio_write(BOARD_I2C_SCL, HAL_GPIO_LOW);
    _bitbang_delay_us(I2C_UNSTICK_HALF_US);
    hal_gpio_write(BOARD_I2C_SCL, HAL_GPIO_HIGH);
    _bitbang_delay_us(I2C_UNSTICK_HALF_US);
  }

  /* Then a STOP -- SDA low while SCL is high, then SDA released -- so the bus
   * is left idle rather than mid-transaction. */
  hal_gpio_write(BOARD_I2C_SDA, HAL_GPIO_LOW);
  _bitbang_delay_us(I2C_UNSTICK_HALF_US);
  hal_gpio_write(BOARD_I2C_SCL, HAL_GPIO_HIGH);
  _bitbang_delay_us(I2C_UNSTICK_HALF_US);
  hal_gpio_write(BOARD_I2C_SDA, HAL_GPIO_HIGH);
  _bitbang_delay_us(I2C_UNSTICK_HALF_US);
}

/** @implements SNS-I2C-001 */
hal_status_t init_i2c_manager(const hal_i2c_config_t *cfg) {
  if (cfg == NULL) {
    return HAL_ERR_INVALID_ARG;
  }
  _boot_config = *cfg;

  // NavHAL's I2C driver enables DMA1 stream 0/5 at HAL_IRQ_PRIORITY_DEFAULT
  // (a BASEPRI-maskable level), so the v_semaphore_give_from_isr in the DMA
  // completion ISR cannot preempt a kernel critical section. No app-side
  // priority pin is needed.

  // Create the bus mutex ONCE. init_i2c_manager is also a recovery entry point
  // (bus-stuck / DMA-start-fail / IMU-stall paths re-call it), so allocating a
  // new mutex here every time would leak the previous one — at the ~20 Hz IMU
  // recovery cadence that drains the heap in seconds. Reuse the existing mutex.
  if (_i2c_sema == NULL) {
    _i2c_sema = v_mutex_create();
  }

  ENTER_CRITICAL();
  _current_trans.state = I2C_TRANS_BLANK;
  v_memset(&_current_trans, 0, sizeof(i2c_async_t));
  atomic_set(&_bus_busy, 0);
  EXIT_CRITICAL();

  uint8_t init_count = 0;
i2c_init:
  init_count++;
  i2c_manager_unstick();

  /* Force a real peripheral SWRST every time. init_i2c_manager is the recovery
   * entry point (bus-stuck / DMA-fail / IMU-stall), but hal_i2c_init() early-
   * returns HAL_ERR_NOT_INITIALIZED on an already-init bus and SKIPS the SWRST —
   * so a stuck BUSY (HAL_ERR_IO) could never clear and the bus wedged for good.
   * Deinit clears the init bit so the hal_i2c_init() below actually resets +
   * reconfigures the peripheral. */
  hal_i2c_deinit(BOARD_I2C_BUS);

  // Configure GPIO for I2C1 (PB8=SCL, PB9=SDA)
  hal_gpio_set_alternate_function(BOARD_I2C_SCL, GPIO_FUNC_I2C);
  hal_gpio_set_alternate_function(BOARD_I2C_SDA, GPIO_FUNC_I2C);
  hal_gpio_set_output_type(BOARD_I2C_SCL, HAL_GPIO_OTYPE_OPEN_DRAIN);
  hal_gpio_set_output_type(BOARD_I2C_SDA, HAL_GPIO_OTYPE_OPEN_DRAIN);
  hal_gpio_set_output_speed(BOARD_I2C_SCL, HAL_GPIO_SPEED_VERY_HIGH);
  hal_gpio_set_output_speed(BOARD_I2C_SDA, HAL_GPIO_SPEED_VERY_HIGH);
  hal_status_t ts = hal_i2c_init(BOARD_I2C_BUS, &_boot_config);

  if (ts != HAL_OK && ts != HAL_ERR_NOT_INITIALIZED) {
    return ts;
  }
  if (ts == HAL_ERR_NOT_INITIALIZED && init_count < 3) {
    goto i2c_init;
  }
  return ts;
}

/** @implements SNS-I2C-102 */
hal_status_t i2c_manager_recover(void) {
  /* No unstick here: init_i2c_manager() pulses the bus on every attempt. */
  return init_i2c_manager(&_boot_config);
}

static uint32_t _consecutive_errors = 0;

/** @implements SNS-I2C-001, SNS-I2C-101 */
hal_status_t i2c_manager_write(uint8_t addr, uint8_t *data, uint16_t len) {
  if (v_mutex_lock(_i2c_sema, MS_TO_TICKS(5)) != VA_PASS) {
    return HAL_ERR_TIMEOUT;
  }
  if (!i2c_manager_acquire_bus()) {
    v_mutex_unlock(_i2c_sema);
    return HAL_ERR_TIMEOUT;
  }
  hal_status_t ts = hal_i2c_write(BOARD_I2C_BUS, addr, data, len);
  i2c_manager_release_bus();
  v_mutex_unlock(_i2c_sema);
  return ts;
}

/** @implements SNS-I2C-001, SNS-I2C-101 */
hal_status_t i2c_manager_read(uint8_t addr, uint8_t *data, uint16_t len) {
  if (v_mutex_lock(_i2c_sema, MS_TO_TICKS(5)) != VA_PASS) {
    return HAL_ERR_TIMEOUT;
  }
  if (!i2c_manager_acquire_bus()) {
    v_mutex_unlock(_i2c_sema);
    return HAL_ERR_TIMEOUT;
  }
  hal_status_t ts = hal_i2c_read(BOARD_I2C_BUS, addr, data, len);
  i2c_manager_release_bus();
  v_mutex_unlock(_i2c_sema);
  return ts;
}

/** @implements SNS-I2C-001, SNS-I2C-101 */
hal_status_t i2c_manager_write_read(uint8_t addr, uint8_t *tx_data,
                                    uint16_t tx_len, uint8_t *rx_data,
                                    uint16_t rx_len) {
  if (v_mutex_lock(_i2c_sema, MS_TO_TICKS(5)) != VA_PASS) {
    return HAL_ERR_TIMEOUT;
  }
  if (!i2c_manager_acquire_bus()) {
    v_mutex_unlock(_i2c_sema);
    return HAL_ERR_TIMEOUT;
  }
  hal_status_t ts =
      hal_i2c_write_read(BOARD_I2C_BUS, addr, tx_data, tx_len, rx_data, rx_len);
  i2c_manager_release_bus();
  v_mutex_unlock(_i2c_sema);
  return ts;
}

/** @implements SNS-I2C-001 */
hal_status_t i2c_manager_read_async(uint8_t addr, uint8_t reg_addr,
                                    uint16_t len, void (*callback)(void *)) {
  if (!i2c_manager_acquire_bus()) {
    _consecutive_errors++;
    if (_consecutive_errors > 100) {
      (void)i2c_manager_recover();
      _consecutive_errors = 0;
    }
    return HAL_ERR_TIMEOUT;
  }

  _consecutive_errors = 0;
  ENTER_CRITICAL();
  _current_trans.addr = addr;
  _current_trans.reg_addr = reg_addr;
  _current_trans.rx_len = len;
  _current_trans.callback = callback;
  _current_trans.op_type = I2C_OP_READ;
  _current_trans.state = I2C_TRANS_BUSY;
  EXIT_CRITICAL();

  /* NavHAL 0.3.x owns the DMA wiring for the bus: which controller, stream and
   * channel I2C1_RX lands on is a reference-manual property of the silicon, and
   * the hardcoded DMA1/stream 0/channel 1 and peripheral address that used to be
   * built here are exactly what a driver should not be asserting. Direction,
   * widths, increments and priority come with it. */
  hal_status_t ret = hal_i2c_read_regs_dma(HAL_I2C_1, addr, reg_addr, _rx_data,
                                           (uint16_t)len, i2c_manager_callback);

  if (ret != HAL_OK) {
    i2c_manager_release_bus();
    vayu_log("I2C DMA START FAIL: %d", ret);
    (void)i2c_manager_recover();
    return ret;
  }

  return HAL_OK;
}

// Called from DMA IRQ handler (ISR context)
// @implements SNS-I2C-001
void i2c_manager_callback(void) {
  // Idempotent against duplicate/spurious DMA completion IRQs. Only the first
  // call for a given transaction (state still BUSY) processes it; we clear the
  // state to BLANK before invoking the user callback so any re-entry returns
  // here. Without this guard a re-fired IRQ calls the bmx160 callback again,
  // which unconditionally gives the IMU read semaphore and advances _next_op —
  // a give_from_isr storm that races v_semaphore_take and orphans the reader
  // task off every scheduler list (IMU stream freezes; rest of system runs).
  if (_current_trans.state != I2C_TRANS_BUSY) {
    return;
  }
  void (*cb)(void *) = _current_trans.callback;
  _current_trans.state = I2C_TRANS_BLANK;
  i2c_manager_release_bus(); // Release bus BEFORE user callback to allow
                             // self-triggering

  if (cb) {
    cb(_rx_data);
  }
}
