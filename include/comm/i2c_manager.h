#ifndef VAYU_I2C_MANAGER_H
#define VAYU_I2C_MANAGER_H

#include "navhal.h"
#include "variables.h"

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
  uint8_t tx_data[I2C_MAX_TX_LEN];
  uint16_t tx_len;
  uint8_t rx_data[I2C_MAX_RX_LEN];
  uint16_t rx_len;
  void (*callback)(void *); // for rx the void* has one byte of data len and
                            // then follow that many bytes of data
  i2c_op_type_t op_type;
  i2c_trans_state_t state;
} i2c_async_t;

void unstick_i2c_bus(void);
hal_i2c_status_t init_i2c_manager(hal_i2c_config_t *cfg);
hal_i2c_status_t i2c_manager_write(uint8_t addr, uint8_t *data, uint16_t len);
hal_i2c_status_t i2c_manager_read(uint8_t addr, uint8_t *data, uint16_t len);
hal_i2c_status_t i2c_manager_write_read(uint8_t addr, uint8_t *tx_data,
                                        uint16_t tx_len, uint8_t *rx_data,
                                        uint16_t rx_len);
hal_i2c_status_t i2c_manager_read_async(uint8_t addr, uint8_t reg_addr,
                                        uint16_t len, void (*callback)(void *));
void i2c_manager_task(void *args);

// Native Queue
typedef struct {
  i2c_async_t queue[MAX_I2C_DEVICES];
  int head;  // points to next item to pop
  int tail;  // points to next free slot to push
  int count; // number of elements (IMPORTANT)
} i2c_queue_t;


void i2c_queue_init(i2c_queue_t *q);
int i2c_queue_push(i2c_queue_t *q, const i2c_async_t *item);
int i2c_queue_pop(i2c_queue_t *q, i2c_async_t *item);
int i2c_queue_is_empty(i2c_queue_t *q);
int i2c_queue_is_full(i2c_queue_t *q);
#endif // VAYU_I2C_MANAGER_H