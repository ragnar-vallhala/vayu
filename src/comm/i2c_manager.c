#include "comm/i2c_manager.h"
#include "core/cortex-m4/i2c.h"
#include "ipc.h"
#include "port.h"
#include "utils.h"
#include "utils/utils.h"
#include "vaios.h"
#include "variables.h"
#include <stdint.h>

static hal_i2c_config_t i2c_config;
static SemaphoreHandle_t _i2c_sema;
static SemaphoreHandle_t _queue_sema;
static SemaphoreHandle_t _dma_done_sema;
static i2c_queue_t _i2c_queue;
static i2c_async_t _current_trans;
static uint8_t initialized = 0;
static uint8_t _rx_data[I2C_MAX_RX_LEN]; // current transanction's rx data
static void i2c_manager_callback(void);
static void i2c_manager_signal_error(void);
void unstick_i2c_bus(void) {
  hal_gpio_setmode(I2C_PIN_1, GPIO_OUTPUT, GPIO_PULLUP);
  hal_gpio_setmode(I2C_PIN_2, GPIO_OUTPUT, GPIO_PULLUP);
  hal_gpio_set_output_type(I2C_PIN_1, GPIO_OPEN_DRAIN);
  hal_gpio_set_output_type(I2C_PIN_2, GPIO_OPEN_DRAIN);

  hal_gpio_digitalwrite(I2C_PIN_2, GPIO_HIGH);
  for (volatile int i = 0; i < 100; i++)
    ;

  for (int i = 0; i < 9; ++i) {
    hal_gpio_digitalwrite(I2C_PIN_1, GPIO_LOW);
    for (volatile int j = 0; j < 200; j++)
      ;
    hal_gpio_digitalwrite(I2C_PIN_1, GPIO_HIGH);
    for (volatile int j = 0; j < 200; j++)
      ;
  }

  hal_gpio_digitalwrite(I2C_PIN_2, GPIO_LOW);
  for (volatile int j = 0; j < 200; j++)
    ;
  hal_gpio_digitalwrite(I2C_PIN_1, GPIO_HIGH);
  for (volatile int j = 0; j < 200; j++)
    ;
  hal_gpio_digitalwrite(I2C_PIN_2, GPIO_HIGH);
  for (volatile int j = 0; j < 200; j++)
    ;
}

hal_i2c_status_t init_i2c_manager(hal_i2c_config_t *cfg) {
  i2c_config = *cfg;
  _i2c_sema = v_semaphore_create_binary();
  v_semaphore_give(_i2c_sema);
  _dma_done_sema = v_semaphore_create_binary();
  v_semaphore_take(_dma_done_sema, 0);
  i2c_queue_init(&_i2c_queue);
  ENTER_CRITICAL();
  _current_trans.state = I2C_TRANS_BLANK;
  v_memset(&_current_trans, 0, sizeof(i2c_async_t));
  EXIT_CRITICAL();
  uint8_t init_count = 0;
i2c_init:
  init_count++;
  unstick_i2c_bus();

  // Configure GPIO for I2C1 (PB8=SCL, PB9=SDA)
  hal_gpio_set_alternate_function(I2C_PIN_1, GPIO_FUNC_I2C);
  hal_gpio_set_alternate_function(I2C_PIN_2, GPIO_FUNC_I2C);
  hal_gpio_set_output_type(I2C_PIN_1, GPIO_OPEN_DRAIN);
  hal_gpio_set_output_type(I2C_PIN_2, GPIO_OPEN_DRAIN);
  hal_gpio_set_output_speed(I2C_PIN_1, GPIO_VERY_HIGH_SPEED);
  hal_gpio_set_output_speed(I2C_PIN_2, GPIO_VERY_HIGH_SPEED);
  hal_i2c_status_t ts = hal_i2c_init(I2C_BUS, &i2c_config);

  if (ts != HAL_I2C_OK && ts != HAL_I2C_ERR_REINIT) {
    return ts;
  }
  if (ts == HAL_I2C_ERR_REINIT && init_count < 3) {
    goto i2c_init;
  } else if (ts == HAL_I2C_OK) {
    initialized = 1;
  }
  return ts;
}

hal_i2c_status_t i2c_manager_write(uint8_t addr, uint8_t *data, uint16_t len) {
  if (v_semaphore_take(_i2c_sema, MS_TO_TICKS(100)) != VA_PASS) {
    return HAL_I2C_ERR_TIMEOUT;
  }
  hal_i2c_status_t ts = hal_i2c_write(I2C_BUS, addr, data, len);
  v_semaphore_give(_i2c_sema);
  return ts;
}

hal_i2c_status_t i2c_manager_read(uint8_t addr, uint8_t *data, uint16_t len) {
  if (v_semaphore_take(_i2c_sema, MS_TO_TICKS(100)) != VA_PASS) {
    return HAL_I2C_ERR_TIMEOUT;
  }
  hal_i2c_status_t ts = hal_i2c_read(I2C_BUS, addr, data, len);
  v_semaphore_give(_i2c_sema);
  return ts;
}

hal_i2c_status_t i2c_manager_write_read(uint8_t addr, uint8_t *tx_data,
                                        uint16_t tx_len, uint8_t *rx_data,
                                        uint16_t rx_len) {
  if (v_semaphore_take(_i2c_sema, MS_TO_TICKS(100)) != VA_PASS) {
    return HAL_I2C_ERR_TIMEOUT;
  }
  hal_i2c_status_t ts =
      hal_i2c_write_read(I2C_BUS, addr, tx_data, tx_len, rx_data, rx_len);
  v_semaphore_give(_i2c_sema);
  return ts;
}

hal_i2c_status_t i2c_manager_read_async(uint8_t addr, uint8_t reg_addr,
                                        uint16_t len,
                                        void (*callback)(void *)) {

  i2c_async_t item = {
      .state = I2C_TRANS_IDLE,
      .callback = callback,
      .addr = addr,
      .reg_addr = reg_addr,
      .rx_len = len,
      .op_type = I2C_OP_READ,
  };
  if (i2c_queue_push(&_i2c_queue, &item) == 1) {
    return HAL_I2C_OK;
  }

  return HAL_I2C_ERR_TIMEOUT;
}
void i2c_manager_task(void *args) {
  i2c_async_t item;
  while (1) {
    // Block here until there's work — prevents busy spin
    if (i2c_queue_pop(&_i2c_queue, &item) != 1) {
      v_delay(1); // yield rather than spin; or use a counting semaphore on the
                  // queue
      continue;
    }

    if (item.state == I2C_TRANS_IDLE && item.op_type == I2C_OP_READ) {

      if (v_semaphore_take(_i2c_sema, MS_TO_TICKS(10)) != VA_PASS) {
        // Bus locked — invoke error path
        void (*cb)(void *) =
            item.callback; // use item, not _current_trans (not set yet)
        if (cb)
          cb(NULL);
        continue;
      }

      ENTER_CRITICAL();
      _current_trans.addr = item.addr;
      _current_trans.reg_addr = item.reg_addr;
      _current_trans.rx_len = item.rx_len;
      _current_trans.callback = item.callback;
      _current_trans.op_type = item.op_type;
      _current_trans.state = I2C_TRANS_BUSY;
      EXIT_CRITICAL();

      dma_config_t i2c_dma_cfg = {.controller = DMA_CONTROLLER_1,
                                  .stream = 0,
                                  .channel = 1,
                                  .direction = DMA_DIR_P2M,
                                  .src_addr = I2C_DR_REG_ADDR,
                                  .dst_addr = (uint32_t)_rx_data,
                                  .data_count = _current_trans.rx_len,
                                  .src_inc = 0,
                                  .dst_inc = 1,
                                  .data_width = DMA_DATA_WIDTH_8,
                                  .priority = DMA_PRIORITY_VERY_HIGH,
                                  .circular = 0};

      hal_i2c_status_t ret = hal_i2c_read_regs_dma(
          I2C1, _current_trans.addr, _current_trans.reg_addr, &i2c_dma_cfg,
          i2c_manager_callback);

      if (ret != HAL_I2C_OK) {
        i2c_manager_signal_error();
        continue;
      }

      // *** Block here until DMA IRQ fires and callback completes ***
      // This prevents re-entry, prevents semaphore double-give,
      // and ensures _rx_data is stable before next transaction
      if (v_semaphore_take(_dma_done_sema, MS_TO_TICKS(20)) != VA_PASS) {
        // DMA hung — force error and release bus
        i2c_manager_signal_error();
      }
      // _i2c_sema is given inside i2c_manager_callback → remove that give
      // Actually: give it HERE instead, after we know the transaction is done
      v_semaphore_give(_i2c_sema);
    }
  }
}

// Called from DMA IRQ handler (ISR context)
void i2c_manager_callback(void) {
  void (*cb)(void *) = _current_trans.callback;
  void *data = (_current_trans.state == I2C_TRANS_BUSY) ? _rx_data : NULL;

  ENTER_CRITICAL();
  _current_trans.state = I2C_TRANS_BLANK;
  v_memset(&_current_trans, 0, sizeof(i2c_async_t));
  _current_trans.state = I2C_TRANS_BLANK;
  EXIT_CRITICAL();

  if (cb)
    cb(data);

  int hp = 0;
  v_semaphore_give_from_isr(_dma_done_sema, &hp);
  if (hp)
    task_yield();
}

// Called from task context only (error paths in i2c_manager_task)
static void i2c_manager_signal_error(void) {
  void (*cb)(void *) = _current_trans.callback;

  ENTER_CRITICAL();
  _current_trans.state = I2C_TRANS_BLANK;
  v_memset(&_current_trans, 0, sizeof(i2c_async_t));
  _current_trans.state = I2C_TRANS_BLANK;
  EXIT_CRITICAL();

  if (cb)
    cb(NULL);

  v_semaphore_give(_dma_done_sema); // task-safe version, NOT from_isr
}
// Native Queue

void i2c_queue_init(i2c_queue_t *q) {
  if (q == NULL)
    return;
  if (_queue_sema == NULL) {
    _queue_sema = v_semaphore_create_binary();
  }
  v_semaphore_give(_queue_sema);
  q->head = 0;
  q->tail = 0;
  q->count = 0;
}

int i2c_queue_push(i2c_queue_t *q, const i2c_async_t *item) {
  if (q->count >= MAX_I2C_DEVICES) {
    return 0; // FULL
  }
  if (v_semaphore_take(_queue_sema, MS_TO_TICKS(100)) != VA_PASS) {
    return 0;
  }
  q->queue[q->tail] = *item;

  q->tail = (q->tail + 1) % MAX_I2C_DEVICES;
  q->count++;

  v_semaphore_give(_queue_sema);
  return 1; // SUCCESS
}

int i2c_queue_pop(i2c_queue_t *q, i2c_async_t *item) {
  if (q->count == 0) {
    return 0; // EMPTY
  }
  if (v_semaphore_take(_queue_sema, MS_TO_TICKS(100)) != VA_PASS) {
    return 0;
  }
  *item = q->queue[q->head];

  q->head = (q->head + 1) % MAX_I2C_DEVICES;
  q->count--;

  v_semaphore_give(_queue_sema);
  return 1; // SUCCESS
}

int i2c_queue_peek(i2c_queue_t *q, i2c_async_t *item) {
  if (q->count == 0) {
    return 0;
  }
  if (v_semaphore_take(_queue_sema, MS_TO_TICKS(100)) != VA_PASS) {
    return 0;
  }
  *item = q->queue[q->head];
  v_semaphore_give(_queue_sema);
  return 1;
}

int i2c_queue_is_empty(i2c_queue_t *q) {
  if (q == NULL)
    return 1;
  return q->count == 0;
}

int i2c_queue_is_full(i2c_queue_t *q) {
  if (q == NULL)
    return 0;
  return q->count == MAX_I2C_DEVICES;
}