#include "comm/ibus.h"
#include "core/cortex-m4/dma_reg.h"
#include "core/cortex-m4/uart.h"
#include "sys/state.h"
#include "utils.h"
#include "utils/utils.h"
#include "vaios.h"

#define IBUS_DMA_BUF_SIZE 128
static uint8_t ibus_dma_buf[IBUS_DMA_BUF_SIZE];
static ibus_data_t ibus_raw_data;

// Shared RC channels for other tasks to use
uint16_t rc_channels[IBUS_MAX_CHANNELS];

void rc_ibus_task(void *args) {
  (void)args;

  // Initialize iBus library
  ibus_init(&ibus_raw_data);

  // Initialize UART6 DMA RX (115200 baud is standard for iBus)
  uart6_init(115200);
  uart6_init_dma_rx(ibus_dma_buf, IBUS_DMA_BUF_SIZE);

  static uint16_t read_ptr = 0;
  static uint16_t last_ndtr = IBUS_DMA_BUF_SIZE;
  static uint32_t last_log_time = 0;

  while (1) {
    // Current remaining items in circular buffer from DMA NDTR register
    // USART1 is on DMA2, Stream 2
    uint16_t current_ndtr = DMA2->STREAM[2].NDTR;

    // Diagnostic log every 1s (User: Dont remove this this makes the rc not
    // work)
    if (v_get_ticks() - last_log_time > 1000) {
      if (current_ndtr != last_ndtr) {
        last_ndtr = current_ndtr;
      }
      last_log_time = v_get_ticks();
    }

    // write_ptr is where the DMA will write NEXT.
    uint16_t write_ptr = IBUS_DMA_BUF_SIZE - current_ndtr;
    while (read_ptr != write_ptr) {
      uint8_t b = ibus_dma_buf[read_ptr];
      if (ibus_parse_byte(b, &ibus_raw_data)) {
        v_memcpy(rc_channels, ibus_raw_data.channels, sizeof(rc_channels));
      }
      read_ptr = (read_ptr + 1) % IBUS_DMA_BUF_SIZE;
      sys_state_t current_state = system_state_get();
      if (rc_channels[4] > 1500) {
        // Switch is UP (Armed position)
        if (current_state == SYSTEM_STATE_STANDBY) {
          system_state_set(SYSTEM_STATE_ARMED);
        }
      } else {
        // Switch is DOWN (Disarmed position)
        if (current_state == SYSTEM_STATE_ARMED ||
            current_state == SYSTEM_STATE_FAILSAFE) {
          system_state_set(SYSTEM_STATE_STANDBY);
        }
      }
    }

    v_delay(2); // 500Hz
  }
}
