#include "comm/channel.h"
#include "navhal.h"
#include "utils/types.h"
#include "vaios.h"
#include "variables.h"
#include <stdint.h>

typedef struct {
  uint32_t baud_rate;
  hal_uart_t uart;
  uint16_t timeout;
  byte buffers[2][512];          // Ping-Pong buffers
  uint16_t buf_lens[2];          // Length of data in each buffer
  uint8_t active_idx;            // Buffer currently being filled (0 or 1)
  volatile uint8_t busy;         // 1 if a DMA transfer is in progress
  uint8_t is_interrupt_attached; // 1 if a attached
} serial_channel_handle_t;

// Serial handlers
static serial_channel_handle_t _serial_handlers[MAX_SERIAL_HANDLERS] = {};

static void _dma_complete_callback(void) {
  // For now, specifically handle USART2/DMA1_S6
  // In a more generic impl, we'd need to know which handler triggered this
  for (int i = 0; i < MAX_SERIAL_HANDLERS; i++) {
    if (_serial_handlers[i].uart == HAL_UART_2) {
      _serial_handlers[i].busy = 0;
      break;
    }
  }
}

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
      // Slot already has an interrupt attached.
      // We should either detach it first or return an error if it's different.
      // For now, let's just update the list but be VERY careful.
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

  return NONE;
}

err_t write_channel(channel_t channel, byte *data, uint16_t length) {
  // This function is not thread safe
  if (channel.handle == NULL || data == NULL || length == 0) {
    return USAGE;
  }

  if (channel.type == CHANNEL_TYPE_SERIAL) {
    serial_channel_handle_t *s_handle =
        (serial_channel_handle_t *)channel.handle;
    uint8_t idx = s_handle->active_idx;

    // uint32_t state = hal_disable_global_interrupts();
    // Check if buffer has space; if not, drop data
    if (s_handle->buf_lens[idx] + length > 512) {
      // hal_enable_global_interrupts(state);
      return ERROR; // Buffer full, dropping data
    }

    // Copy data to active buffer
    for (uint16_t i = 0; i < length; i++) {
      s_handle->buffers[idx][s_handle->buf_lens[idx] + i] = data[i];
    }
    s_handle->buf_lens[idx] += length;
    // hal_enable_global_interrupts(state);

    return NONE;
  }

  return USAGE;
}

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

    uint8_t flush_idx = s_handle->active_idx;
    uint16_t flush_len = s_handle->buf_lens[flush_idx];

    if (flush_len == 0) {
      return NONE;
    }

    // Swap buffers
    s_handle->active_idx = 1 - s_handle->active_idx;
    s_handle->buf_lens[s_handle->active_idx] = 0; // Clear the new active buffer

    // Mark busy
    s_handle->busy = 1;

    // Trigger transmission
    if (s_handle->uart == HAL_UART_2) {
#if defined(_UART_BACKEND_DMA) && !defined(VAYU_SIM)
      hal_uart_write_dma(HAL_UART_2, s_handle->buffers[flush_idx], flush_len);
#else
      // Polling path: used when DMA is disabled, OR under VAYU_SIM because
      // Renode's STM32_UART doesn't issue TX DMA requests (the CR3 DMAT bit
      // is "unhandled"), so the DMA-driven write would never complete.
      for (uint16_t i = 0; i < flush_len; i++) {
        hal_uart_write_char(HAL_UART_2, (char)s_handle->buffers[flush_idx][i]);
      }
      s_handle->busy = 0;
#endif
    } else {
      // Other UARTs (currently blocking)
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

static err_t get_handler_default(channel_t *handler, void *args) {
  return USAGE;
}

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
      s_handle->uart = 0; // Mark slot as free
    }
    if (s_handle->is_interrupt_attached) {
      hal_irq_t usart_irq = s_handle->uart == HAL_UART_1   ? USART1_IRQn
                            : s_handle->uart == HAL_UART_6 ? USART6_IRQn
                                                      : USART2_IRQn;
      if (hal_interrupt_disable(usart_irq) == 1)
        return USAGE;
      hal_interrupt_detach_callback(usart_irq);
    }
  }

  handler->handle = NULL;
  handler->next = NULL;

  return NONE;
}

void flush_task(void *args) {
  while (1) {
    channel_t *curr = active_handlers;
    while (curr != NULL) {
      flush_channel(*curr);
      curr = curr->next;
    }
    v_delay(1);
  }
}