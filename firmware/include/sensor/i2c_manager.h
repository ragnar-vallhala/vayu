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
  uint16_t tx_len;
  uint16_t rx_len;
  void (*callback)(void *); // for rx the void* has one byte of data len and
                            // then follow that many bytes of data
  i2c_op_type_t op_type;
  i2c_trans_state_t state;
} i2c_async_t;

void i2c_manager_unstick(void);
hal_status_t init_i2c_manager(hal_i2c_config_t *cfg);
hal_status_t i2c_manager_write(uint8_t addr, uint8_t *data, uint16_t len);
hal_status_t i2c_manager_read(uint8_t addr, uint8_t *data, uint16_t len);
hal_status_t i2c_manager_write_read(uint8_t addr, uint8_t *tx_data,
                                    uint16_t tx_len, uint8_t *rx_data,
                                    uint16_t rx_len);
hal_status_t i2c_manager_read_async(uint8_t addr, uint8_t reg_addr,
                                    uint16_t len, void (*callback)(void *));
#endif // VAYU_I2C_MANAGER_H