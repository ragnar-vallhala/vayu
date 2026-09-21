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
#include "comm/channel.h"
#include "navhal.h"
#include "port.h" // ENTER_CRITICAL / EXIT_CRITICAL — serialise the ping-pong buffer
#include "sys/types.h"
#include "vaios.h"
#include "variables.h"
#include <stdint.h>

/* Per-buffer TX capacity. Must hold the largest single-shot write burst between
 * flushes: the 1 Hz perf report dumps global + up to 24 task + 16 fifo frames
 * back-to-back (~1336 B worst case). 2048 B fits a full burst with margin; the
 * DMA TX (USART6) drains it without CPU cost. */
#define CHANNEL_TX_BUF_SIZE 2048u

typedef struct {
  uint32_t baud_rate;
  hal_uart_t uart;
  uint16_t timeout;
  byte buffers[2][CHANNEL_TX_BUF_SIZE]; // Ping-Pong buffers
  uint16_t buf_lens[2];                 // Length of data in each buffer
  uint8_t active_idx;            // Buffer currently being filled (0 or 1)
  volatile uint8_t busy;         // 1 if a DMA transfer is in progress
  uint8_t is_interrupt_attached; // 1 if a attached
} serial_channel_handle_t;

// Serial handlers
static serial_channel_handle_t _serial_handlers[MAX_SERIAL_HANDLERS] = {0};

/* COMM-CH-002: count of writes dropped because the active TX buffer
 * had no room. Monotonic; surfaced through telemetry (SYSTEM_ORIGIN_HEALTH).
 * Single 32-bit scalar — atomic load/store on Cortex-M4 (R8.6).
 * @implements COMM-CH-002 */
static volatile uint32_t _tx_overflow_count = 0;

uint32_t channel_tx_overflow_count(void) { return _tx_overflow_count; }

/** @noreq TX-DMA completion ISR: releases the channel busy flag */
static void _dma_complete_callback(void) {
  // Handles only USART2/DMA1_S6; a generic impl would need to know which
  // handler triggered this.
  for (int i = 0; i < MAX_SERIAL_HANDLERS; i++) {
    if (_serial_handlers[i].uart == HAL_UART_2) {
      _serial_handlers[i].busy = 0;
      break;
    }
  }
}

/* Telemetry UART (USART6) TX-DMA completion (DMA2_Stream7). NavHAL's stream IRQ
 * handler clears the DMA flags and dispatches here; we only release the channel
 * so the next flush can ping-pong swap and send. Without DMA, flush_task would
 * block byte-by-byte pushing the whole telemetry stream (~15% CPU).
 *
 * Stream7 (not Stream6): USART6_TX shares DMA2 Stream6 with the SDIO write DMA,
 * which re-grabs the Stream6 completion IRQ on every SD block write and would
 * permanently strand `busy=1` here (flush then always returns ERROR -> telemetry
 * dies after the first SD write, e.g. saving calibration). USART6_TX's alternate
 * mapping is Stream7/Ch5, free of SDIO — see _get_uart_dma_params in NavHAL. */
/** @noreq telemetry-UART TX-DMA completion ISR: releases the channel */
static void _dma_complete_callback_u6(void) {
  for (int i = 0; i < MAX_SERIAL_HANDLERS; i++) {
    if (_serial_handlers[i].uart == HAL_UART_6) {
      _serial_handlers[i].busy = 0;
      break;
    }
  }
}

/** @noreq UART channel allocation + peripheral/DMA init glue */
static err_t get_handler_serial(channel_t *handler, void *args,
                                void (*callback)(void)) {
  if (args == NULL || handler == NULL) {
    return USAGE;
  }

  serial_args_t *s_args = (serial_args_t *)args;

  // 1. Check if a handler for this UART already exists
  int slot = -1;
  for (int i = 0; i < MAX_SERIAL_HANDLERS; i++) {
    if (_serial_handlers[i].uart == s_args->uart) {
      slot = i;
      // Already initialized, just return it
      handler->type = CHANNEL_TYPE_SERIAL;
      handler->handle = &_serial_handlers[slot];
      handler->index = (uint8_t)slot;
      return NONE;
    }
  }

  // 2. Find an available slot if not found
  for (int i = 0; i < MAX_SERIAL_HANDLERS; i++) {
    if (_serial_handlers[i].uart == 0) {
      slot = i;
      break;
    }
  }

  if (slot == -1) {
    return ERROR;
  }

  // Initialize the UART peripheral
  hal_uart_config_t _uart_cfg = {.baudrate = s_args->baud_rate};
  hal_uart_init(s_args->uart, &_uart_cfg);

  if (callback) {
    if (_serial_handlers[slot].is_interrupt_attached) {
      // Slot already has an interrupt attached; reattaching overwrites it.
    }
    hal_irq_t usart_irq = s_args->uart == HAL_UART_1   ? USART1_IRQn
                          : s_args->uart == HAL_UART_6 ? USART6_IRQn
                                                       : USART2_IRQn;
    hal_interrupt_attach_callback(usart_irq, callback);
    hal_uart_enable_interrupt(s_args->uart, 1, 0);
    _serial_handlers[slot].is_interrupt_attached = 1;
  }
  // Store the configuration
  _serial_handlers[slot].baud_rate =
      (uint16_t)s_args->baud_rate; // Cast to match struct
  _serial_handlers[slot].uart = s_args->uart;
  _serial_handlers[slot].timeout = s_args->timeout;

  // Set up the high-level handler
  handler->type = CHANNEL_TYPE_SERIAL;
  handler->handle = &_serial_handlers[slot];
  handler->index = (uint8_t)slot;

  // Initialize buffer state
  _serial_handlers[slot].active_idx = 0;
  _serial_handlers[slot].buf_lens[0] = 0;
  _serial_handlers[slot].buf_lens[1] = 0;
  _serial_handlers[slot].busy = 0;

  // Attach DMA callback if using USART2
  if (s_args->uart == HAL_UART_2) {
    // PROTECT: don't overwrite if kernel logging or another task already set
    // it! We should ideally have a multi-callback system, but for now, just
    // don't break existing ones.
    hal_interrupt_attach_callback(DMA1_Stream6_IRQn, _dma_complete_callback);
    hal_interrupt_enable(DMA1_Stream6_IRQn);
  }
  // Telemetry UART (USART6) TX uses DMA2_Stream7 so flush_channel offloads the
  // stream to DMA instead of busy-pushing it byte-by-byte; the completion IRQ
  // releases the channel (_dma_complete_callback_u6). Stream7 (not Stream6) to
  // avoid the SDIO TX-DMA conflict that wedges telemetry after an SD write.
  if (s_args->uart == HAL_UART_6) {
    hal_interrupt_attach_callback(DMA2_Stream7_IRQn, _dma_complete_callback_u6);
    hal_interrupt_enable(DMA2_Stream7_IRQn);
  }

  return NONE;
}

// Tail of the TX ring reserved for bulk xfer (XFER_DATA download chunks).
// Normal writes (telemetry/acks) are capped at CHANNEL_TX_BUF_SIZE - this, so a
// saturating telemetry stream cannot fill the whole ring and starve a download;
// write_channel_xfer may use the full ring. ~3 max frames (3*~264 B) of
// guaranteed headroom -> the download always makes forward progress, sharing the
// link with telemetry rather than pausing it.
#define CHANNEL_TX_XFER_RESERVE 768u

// Shared core: `cap` is the highest fill level this writer may reach. Normal
// writers pass the reserved cap; xfer passes the full buffer.
/** @implements COMM-CH-002 */
static err_t _write_channel(channel_t channel, byte *data, uint16_t length,
                            uint16_t cap) {
  // This function is not thread safe
  if (channel.handle == NULL || data == NULL || length == 0) {
    return USAGE;
  }

  if (channel.type == CHANNEL_TYPE_SERIAL) {
    serial_channel_handle_t *s_handle =
        (serial_channel_handle_t *)channel.handle;

    // The active buffer index + length + the copy must be atomic against
    // flush_channel's ping-pong swap, or flush can transmit a buffer this
    // writer is still filling (split/overwritten packets -> CRC errors at the
    // GCS). The copy is at most one packet (~264 B ≈ a few µs), so a short
    // critical section is acceptable.
    ENTER_CRITICAL();
    uint8_t idx = s_handle->active_idx;

    // Check if buffer has space (up to this writer's cap); if not, drop data
    if (s_handle->buf_lens[idx] + length > cap) {
      EXIT_CRITICAL();
      _tx_overflow_count++; // COMM-CH-002
      return ERROR;         // Buffer full, dropping data
    }

    // Copy data to active buffer
    for (uint16_t i = 0; i < length; i++) {
      s_handle->buffers[idx][s_handle->buf_lens[idx] + i] = data[i];
    }
    s_handle->buf_lens[idx] += length;
    EXIT_CRITICAL();

    return NONE;
  }

  return USAGE;
}

/** @noreq thin wrapper over _write_channel (normal-writer cap) */
err_t write_channel(channel_t channel, byte *data, uint16_t length) {
  return _write_channel(channel, data, length,
                        CHANNEL_TX_BUF_SIZE - CHANNEL_TX_XFER_RESERVE);
}

/** @noreq thin wrapper over _write_channel (full cap for bulk xfer) */
err_t write_channel_xfer(channel_t channel, byte *data, uint16_t length) {
  return _write_channel(channel, data, length, CHANNEL_TX_BUF_SIZE);
}

/** @implements COMM-CH-001, COMM-FLUSH-001 */
err_t flush_channel(channel_t channel) {
  if (channel.handle == NULL) {
    return USAGE;
  }

  if (channel.type == CHANNEL_TYPE_SERIAL) {
    serial_channel_handle_t *s_handle =
        (serial_channel_handle_t *)channel.handle;

    // Wait if a DMA transfer is in progress
    if (s_handle->busy) {
      return ERROR;
    }

    // Snapshot + ping-pong swap must be atomic against write_channel so it
    // can't keep appending to the buffer we are about to transmit. After the
    // swap, flush_idx is private to this flush (write_channel uses the new
    // active_idx), so the actual transmit below runs outside the critical
    // section.
    ENTER_CRITICAL();
    uint8_t flush_idx = s_handle->active_idx;
    uint16_t flush_len = s_handle->buf_lens[flush_idx];

    if (flush_len == 0) {
      EXIT_CRITICAL();
      return NONE;
    }

    // Swap buffers
    s_handle->active_idx = 1 - s_handle->active_idx;
    s_handle->buf_lens[s_handle->active_idx] = 0; // Clear the new active buffer

    // Mark busy
    s_handle->busy = 1;
    EXIT_CRITICAL();

    // Trigger transmission. The telemetry UART (USART6) goes out via DMA so the
    // flush task doesn't busy-push the whole stream a byte at a time (~15% CPU
    // otherwise); `busy` is cleared by the DMA2_Stream6 completion IRQ. Other
    // UARTs keep the blocking fallback.
    if (s_handle->uart == HAL_UART_6) {
      hal_uart_write_dma(HAL_UART_6, s_handle->buffers[flush_idx], flush_len);
    } else if (s_handle->uart == HAL_UART_2) {
#ifdef _UART_BACKEND_DMA
      hal_uart_write_dma(HAL_UART_2, s_handle->buffers[flush_idx], flush_len);
#else
      for (uint16_t i = 0; i < flush_len; i++) {
        hal_uart_write_char(HAL_UART_2, (char)s_handle->buffers[flush_idx][i]);
      }
      s_handle->busy = 0;
#endif
    } else {
      // Other UARTs (blocking fallback)
      for (uint16_t i = 0; i < flush_len; i++) {
        hal_uart_write_char(s_handle->uart,
                            (char)s_handle->buffers[flush_idx][i]);
      }
      s_handle->busy = 0;
    }

    return NONE;
  }

  return USAGE;
}

// Active handlers linked list head
static channel_t *active_handlers = NULL;

/** @noreq unsupported-channel-type stub */
static err_t get_handler_default(channel_t *handler, void *args) {
  (void)handler;
  (void)args;
  return USAGE;
}

/** @noreq channel-handler registry/dispatch glue */
err_t get_handler(channel_type_t channel_type, channel_t *handler, void *args,
                  void (*onRecieve)(void)) {
  err_t status = USAGE;
  switch (channel_type) {
  case CHANNEL_TYPE_SERIAL:
    status = get_handler_serial(handler, args, onRecieve);
    break;
  case CHANNEL_TYPE_SPI:
  case CHANNEL_TYPE_I2C:
  case CHANNEL_TYPE_USB:
  case CHANNEL_TYPE_CAN:
    status = get_handler_default(handler, args);
    break;
  default:
    return USAGE;
  }

  if (status == NONE && handler != NULL) {
    // Add to linked list
    uint32_t state = hal_interrupt_disable_global();
    handler->next = active_handlers;
    active_handlers = handler;
    hal_interrupt_enable_global(state);
  }
  return status;
}

/** @noreq channel teardown glue (unlink + detach IRQ + free slot) */
err_t del_handler(channel_t *handler) {
  if (handler == NULL) {
    return USAGE;
  }

  // Remove from linked list
  channel_t **curr = &active_handlers;
  while (*curr != NULL) {
    if (*curr == handler) {
      *curr = handler->next;
      break;
    }
    curr = &((*curr)->next);
  }

  // Free the underlying hardware slot
  if (handler->type == CHANNEL_TYPE_SERIAL) {
    serial_channel_handle_t *s_handle =
        (serial_channel_handle_t *)handler->handle;
    if (s_handle != NULL) {
      if (s_handle->is_interrupt_attached) {
        hal_irq_t usart_irq = s_handle->uart == HAL_UART_1   ? USART1_IRQn
                              : s_handle->uart == HAL_UART_6 ? USART6_IRQn
                                                             : USART2_IRQn;
        if (hal_interrupt_disable(usart_irq) == 1)
          return USAGE;
        hal_interrupt_detach_callback(usart_irq);
      }
      s_handle->uart = 0; // Mark slot as free (after detaching its IRQ)
    }
  }

  handler->handle = NULL;
  handler->next = NULL;

  return NONE;
}

/** @implements COMM-FLUSH-001 */
void flush_task(void *args) {
  (void)args;
  while (1) {
    channel_t *curr = active_handlers;
    while (curr != NULL) {
      flush_channel(*curr);
      curr = curr->next;
    }
    v_delay(1);
  }
}
