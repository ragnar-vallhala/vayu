#include "sensor/i2c_manager.h"
#include "ipc.h"
#include "navhal.h"
#include "port.h"
#include "utils.h"
#include "storage/fs_owner.h"
#include "vaios.h"
#include "variables.h"
#include <stdint.h>

static hal_i2c_config_t i2c_config;
static SemaphoreHandle_t _i2c_sema; // Mutex for synchronous calls
static atomic_t _bus_busy;          // Atomic flag for bus availability
static i2c_async_t _current_trans;
static uint8_t initialized = 0;
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

/** @implements SNS-I2C-102 */
void i2c_manager_unstick(void) {
  hal_gpio_set_mode(I2C_PIN_1, HAL_GPIO_MODE_OUTPUT, HAL_GPIO_PULL_UP);
  hal_gpio_set_mode(I2C_PIN_2, HAL_GPIO_MODE_OUTPUT, HAL_GPIO_PULL_UP);
  hal_gpio_set_output_type(I2C_PIN_1, HAL_GPIO_OTYPE_OPEN_DRAIN);
  hal_gpio_set_output_type(I2C_PIN_2, HAL_GPIO_OTYPE_OPEN_DRAIN);

  hal_gpio_write(I2C_PIN_2, HAL_GPIO_HIGH);
  for (volatile int i = 0; i < 100; i++)
    ;

  for (int i = 0; i < 9; ++i) {
    hal_gpio_write(I2C_PIN_1, HAL_GPIO_LOW);
    for (volatile int j = 0; j < 200; j++)
      ;
    hal_gpio_write(I2C_PIN_1, HAL_GPIO_HIGH);
    for (volatile int j = 0; j < 200; j++)
      ;
  }

  hal_gpio_write(I2C_PIN_2, HAL_GPIO_LOW);
  for (volatile int j = 0; j < 200; j++)
    ;
  hal_gpio_write(I2C_PIN_1, HAL_GPIO_HIGH);
  for (volatile int j = 0; j < 200; j++)
    ;
  hal_gpio_write(I2C_PIN_2, HAL_GPIO_HIGH);
  for (volatile int j = 0; j < 200; j++)
    ;
}

/** @implements SNS-I2C-001 */
hal_status_t init_i2c_manager(hal_i2c_config_t *cfg) {
  i2c_config = *cfg;

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
  hal_i2c_deinit(I2C_BUS);

  // Configure GPIO for I2C1 (PB8=SCL, PB9=SDA)
  hal_gpio_set_alternate_function(I2C_PIN_1, GPIO_FUNC_I2C);
  hal_gpio_set_alternate_function(I2C_PIN_2, GPIO_FUNC_I2C);
  hal_gpio_set_output_type(I2C_PIN_1, HAL_GPIO_OTYPE_OPEN_DRAIN);
  hal_gpio_set_output_type(I2C_PIN_2, HAL_GPIO_OTYPE_OPEN_DRAIN);
  hal_gpio_set_output_speed(I2C_PIN_1, HAL_GPIO_SPEED_VERY_HIGH);
  hal_gpio_set_output_speed(I2C_PIN_2, HAL_GPIO_SPEED_VERY_HIGH);
  hal_status_t ts = hal_i2c_init(I2C_BUS, &i2c_config);

  if (ts != HAL_OK && ts != HAL_ERR_NOT_INITIALIZED) {
    return ts;
  }
  if (ts == HAL_ERR_NOT_INITIALIZED && init_count < 3) {
    goto i2c_init;
  } else if (ts == HAL_OK) {
    initialized = 1;
  }
  return ts;
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
  hal_status_t ts = hal_i2c_write(I2C_BUS, addr, data, len);
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
  hal_status_t ts = hal_i2c_read(I2C_BUS, addr, data, len);
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
      hal_i2c_write_read(I2C_BUS, addr, tx_data, tx_len, rx_data, rx_len);
  i2c_manager_release_bus();
  v_mutex_unlock(_i2c_sema);
  return ts;
}

/** @implements SNS-I2C-001 */
hal_status_t i2c_manager_read_async(uint8_t addr, uint8_t reg_addr,
                                        uint16_t len,
                                        void (*callback)(void *)) {
  if (!i2c_manager_acquire_bus()) {
    _consecutive_errors++;
    if (_consecutive_errors > 100) {
      i2c_manager_unstick();
      init_i2c_manager(&i2c_config);
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

  hal_dma_config_t i2c_dma_cfg = {.controller = HAL_DMA_CONTROLLER_1,
                                  .stream = 0,
                                  .channel = 1,
                                  .direction = HAL_DMA_DIR_P2M,
                                  .src_addr = I2C_DR_REG_ADDR,
                                  .dst_addr = (uint32_t)_rx_data,
                                  .data_count = len,
                                  .src_inc = 0,
                                  .dst_inc = 1,
                                  .data_width = HAL_DMA_DATA_WIDTH_8,
                                  .priority = HAL_DMA_PRIORITY_VERY_HIGH,
                                  .circular = 0};

  hal_status_t ret = hal_i2c_read_regs_dma(
      HAL_I2C_1, addr, reg_addr, &i2c_dma_cfg, i2c_manager_callback);

  if (ret != HAL_OK) {
    i2c_manager_release_bus();
    vayu_log("I2C DMA START FAIL: %d", ret);
    i2c_manager_unstick();
    init_i2c_manager(&i2c_config);
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
