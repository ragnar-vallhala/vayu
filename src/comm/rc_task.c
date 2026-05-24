#include "comm/ibus.h"
#include "comm/rc_buffer.h"
#include "navhal.h"
#include "sys/state.h"
#include "utils.h"
#include "vaios.h"
#include "variables.h"
#include <stdint.h>

#define IBUS_DMA_BUF_SIZE 128
static uint8_t ibus_dma_buf[IBUS_DMA_BUF_SIZE];
static ibus_data_t ibus_raw_data;

#ifdef VAYU_SIM
/* SITL override: when sim_rc_enabled is non-zero, rc_ibus_task skips
 * iBus parsing and uses sim_rc_channels[] as the live RC input. The
 * host SITL driver writes these directly. */
volatile uint8_t  sim_rc_enabled = 0;
volatile uint16_t sim_rc_channels[14] = {1500, 1500, 1000, 1500, 1000,
                                         1000, 1500, 1500, 1500, 1500,
                                         1500, 1500, 1500, 1500};
#endif

void rc_ibus_task(void *args) {
  (void)args;

  // Initialize iBus library
  ibus_init(&ibus_raw_data);

  // Initialize UART6 DMA RX (115200 baud is standard for iBus)
  hal_uart_config_t ibus_uart_cfg = {.baudrate = 115200};
  hal_uart_init(HAL_UART_6, &ibus_uart_cfg);
  hal_uart_init_dma_rx(HAL_UART_6, ibus_dma_buf, IBUS_DMA_BUF_SIZE);

  static uint16_t read_ptr = 0;
  static uint16_t last_ndtr = IBUS_DMA_BUF_SIZE;
  static uint32_t last_log_time = 0;
  static uint8_t new_data = 0;
  while (1) {
#ifdef VAYU_SIM
    if (sim_rc_enabled) {
      for (int i = 0; i < 14; i++) {
        ibus_raw_data.channels[i] = sim_rc_channels[i];
      }
      ibus_raw_data.is_failsafe = false;
      /* Run the same state-machine logic the parser-driven path runs. */
      sys_state_t current_state = system_state_get();
      for (int i = 0; i < 4; i++) {
        if (i != 2 && (ibus_raw_data.channels[i] > 1500 - RADIO_AVOID_BAND &&
                       ibus_raw_data.channels[i] < 1500 + RADIO_AVOID_BAND)) {
          ibus_raw_data.channels[i] = 1500;
        }
      }
      if (ibus_raw_data.channels[4] > 1500) {
        if (current_state == SYSTEM_STATE_STANDBY &&
            ibus_raw_data.channels[2] < 1100) {
          system_state_set(SYSTEM_STATE_ARMED);
        } else if (current_state == SYSTEM_STATE_STANDBY &&
                   ibus_raw_data.channels[2] > 1100) {
          system_state_set(SYSTEM_STATE_FAILSAFE);
        }
      } else {
        if (current_state == SYSTEM_STATE_ARMED ||
            current_state == SYSTEM_STATE_FAILSAFE) {
          system_state_set(SYSTEM_STATE_STANDBY);
        }
      }
      rc_queue_control_push(&ibus_raw_data);
      rc_queue_telemetry_push(&ibus_raw_data);
      v_delay(20);   /* 50 Hz, matches real iBus rate */
      continue;
    }
#endif
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
#ifdef VAYU_SIM
    /* If sim RC was just enabled, bail out of the iBus inner loop so we
     * pick it up on the next outer iteration. */
    if (sim_rc_enabled) {
      v_delay(20);
      continue;
    }
#endif
    while (read_ptr != write_ptr) {
#ifdef VAYU_SIM
      if (sim_rc_enabled) break;
#endif
      uint8_t b = ibus_dma_buf[read_ptr];
      if (ibus_parse_byte(b, &ibus_raw_data)) {
        new_data = 1;
      }
      read_ptr = (read_ptr + 1) % IBUS_DMA_BUF_SIZE;
      sys_state_t current_state = system_state_get();
      for (int i = 0; i < 4; i++) {
        if (i != 2 && (ibus_raw_data.channels[i] > 1500 - RADIO_AVOID_BAND &&
                       ibus_raw_data.channels[i] < 1500 + RADIO_AVOID_BAND)) {
          ibus_raw_data.channels[i] = 1500;
        }
      }
      if (ibus_raw_data.channels[4] > 1500) {
        // Switch is UP (Armed position)
        if (current_state == SYSTEM_STATE_STANDBY && ibus_raw_data.channels[2] < 1100) {
          system_state_set(SYSTEM_STATE_ARMED);
        } else if (current_state == SYSTEM_STATE_STANDBY &&
                   ibus_raw_data.channels[2] > 1100) {
          system_state_set(SYSTEM_STATE_FAILSAFE);
        }
      } else {
        // Switch is DOWN (Disarmed position)
        if (current_state == SYSTEM_STATE_ARMED ||
            current_state == SYSTEM_STATE_FAILSAFE) {
          system_state_set(SYSTEM_STATE_STANDBY);
        }
      }
      if (new_data) {
        rc_queue_control_push(&ibus_raw_data);
        rc_queue_telemetry_push(&ibus_raw_data);
        new_data = 0;
      }
    }

    v_delay(2); // 500Hz
  }
}
