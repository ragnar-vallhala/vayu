#include "comm/i2c_manager.h"
#include "core/cortex-m4/i2c.h"
#include "ipc.h"
#include "port.h"
#include "utils.h"
#include "variables.h"
#include <stdint.h>

static hal_i2c_config_t i2c_config;
static SemaphoreHandle_t _i2c_sema;
static SemaphoreHandle_t _queue_sema;
static i2c_queue_t _i2c_queue;
static i2c_async_t _current_trans;
static uint8_t initialized = 0;

static void i2c_manager_callback(void);

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

  _queue_sema = v_semaphore_create_binary();
  v_semaphore_give(_queue_sema);

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
  for (int i = 0; i < MAX_I2C_DEVICES; i++) {
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
  }
  return HAL_I2C_ERR_TIMEOUT;
}
void i2c_manager_task(void *args) {
  i2c_async_t item;
  while (1) {
    if (i2c_queue_pop(&_i2c_queue, &item) == 1) {
      if (item.state == I2C_TRANS_IDLE) {
        if (item.op_type == I2C_OP_READ) {
          if (v_semaphore_take(_i2c_sema, MS_TO_TICKS(5)) == VA_PASS) {
            ENTER_CRITICAL();
            _current_trans = item;
            _current_trans.state = I2C_TRANS_BUSY;
            EXIT_CRITICAL();
            dma_config_t i2c_dma_cfg = {
                .controller = DMA_CONTROLLER_1,
                .stream = 0,
                .channel = 1,
                .direction = DMA_DIR_P2M,
                .src_addr =
                    (uint32_t)(0x40005400 + 0x10), // I2C1_BASE + DR Offset
                .dst_addr = (uint32_t)_current_trans.rx_data,
                .data_count = _current_trans.rx_len,
                .src_inc = 0,
                .dst_inc = 1,
                .data_width = DMA_DATA_WIDTH_8,
                .priority = DMA_PRIORITY_VERY_HIGH,
                .circular = 0};
            hal_i2c_read_regs_dma(I2C1, _current_trans.addr,
                                  _current_trans.reg_addr, &i2c_dma_cfg,
                                  i2c_manager_callback);
          } else {
            ENTER_CRITICAL();
            _current_trans.state = I2C_TRANS_ERROR;
            EXIT_CRITICAL();
            i2c_manager_callback();
          }
        }
      }
    }
  }
}

void i2c_manager_callback(void) {
  if (_current_trans.state == I2C_TRANS_BUSY) {
    ENTER_CRITICAL();
    _current_trans.state = I2C_TRANS_DONE;
    EXIT_CRITICAL();
    _current_trans.callback(_current_trans.rx_data);
    v_semaphore_give(_i2c_sema);
    v_memset(&_current_trans, 0, sizeof(i2c_async_t));
    _current_trans.state = I2C_TRANS_BLANK;
  } else if (_current_trans.state == I2C_TRANS_ERROR) {
    ENTER_CRITICAL();
    _current_trans.state = I2C_TRANS_BLANK;
    EXIT_CRITICAL();
    _current_trans.callback(NULL);
    v_semaphore_give(_i2c_sema);
  }
}

// Native Queue

void i2c_queue_init(i2c_queue_t *q) {
  q->head = 0;
  q->tail = 0;
  q->count = 0;
}

int i2c_queue_push(i2c_queue_t *q, const i2c_async_t *item) {
  if (q->count >= MAX_I2C_DEVICES) {
    return 0; // FULL
  }

  q->queue[q->tail] = *item;

  q->tail = (q->tail + 1) % MAX_I2C_DEVICES;
  q->count++;

  return 1; // SUCCESS
}

int i2c_queue_pop(i2c_queue_t *q, i2c_async_t *item) {
  if (q->count == 0) {
    return 0; // EMPTY
  }

  *item = q->queue[q->head];

  q->head = (q->head + 1) % MAX_I2C_DEVICES;
  q->count--;

  return 1; // SUCCESS
}

int i2c_queue_peek(i2c_queue_t *q, i2c_async_t *item) {
  if (q->count == 0) {
    return 0;
  }

  *item = q->queue[q->head];
  return 1;
}

int i2c_queue_is_empty(i2c_queue_t *q) { return q->count == 0; }

int i2c_queue_is_full(i2c_queue_t *q) { return q->count == MAX_I2C_DEVICES; }