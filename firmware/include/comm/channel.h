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
#ifndef VAYU_CHANNEL_H
#define VAYU_CHANNEL_H

#include "navhal.h"
#include "sys/types.h"
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

/**
 * @brief Like write_channel, but may use the xfer-reserved tail of the TX ring.
 *        Normal writes (telemetry, acks) are capped below the reservation so a
 *        saturating telemetry stream can never fill the whole ring; bulk xfer
 *        (XFER_DATA download chunks) calls this to claim the reserved headroom
 *        and so always makes forward progress instead of stalling behind
 *        telemetry. Same return contract as write_channel (NONE / ERROR-on-full).
 */
err_t write_channel_xfer(channel_t channel, byte *data, uint16_t length);
err_t flush_channel(channel_t channel);

/**
 * @brief Number of channel writes dropped due to a full TX buffer.
 *        Surfaced through telemetry (SYSTEM_ORIGIN_HEALTH).
 * @implements COMM-CH-002
 */
uint32_t channel_tx_overflow_count(void);

void flush_task(void *args);
#endif // VAYU_CHANNEL_H