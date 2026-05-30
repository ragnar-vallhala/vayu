#ifndef VAYU_CHANNEL_H
#define VAYU_CHANNEL_H

#include "navhal.h"
#include "utils/types.h"
#include <stdint.h>

typedef struct {
  uint32_t baud_rate;
  hal_uart_t uart;
  uint16_t timeout;
} serial_args_t;

typedef enum {
  CHANNEL_TYPE_SERIAL,
  CHANNEL_TYPE_SPI,
  CHANNEL_TYPE_I2C,
  CHANNEL_TYPE_USB,
  CHANNEL_TYPE_CAN,
} channel_type_t;

typedef struct channel_s {
  channel_type_t type;
  void *handle;
  uint8_t index;
  struct channel_s *next;
} channel_t;

err_t get_handler(channel_type_t channel_type, channel_t *handler, void *args,
                  void (*onRecieve)(void));
err_t del_handler(channel_t *handler);
err_t write_channel(channel_t channel, byte *data, uint16_t length);
err_t flush_channel(channel_t channel);

/**
 * @brief Number of channel writes dropped due to a full TX buffer.
 *        Surfaced through telemetry (SYSTEM_ORIGIN_HEALTH).
 * @implements COMM-CH-002
 */
uint32_t channel_tx_overflow_count(void);

void flush_task(void *args);
#endif // VAYU_CHANNEL_H