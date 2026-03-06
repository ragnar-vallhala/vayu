#include "vayu_tasks.h"
#include "comm/channel.h"
#include "comm/comm_types.h"
#include "comm/serializer.h"
#include "core/cortex-m4/uart.h"
#include "utils.h"
#include "utils/types.h"
#include "vaios.h"
#include "variables.h"
#include <stdint.h>

void physical_heartbeat(void *args) {

  // Configure Physical Heartbeat
  uint32_t time_period_half =
      _HEARTBEAT_DEFAULT_TIMEPERIOD / 2; // 1000ms by default
  if (args) {
    time_period_half = (*(uint32_t *)args) / 2;
  }
  // clamp to 250ms for a max of 2Hz blink
  time_period_half = time_period_half >= 250 ? time_period_half : 250;
  // Configure pins
  hal_gpio_setmode(_HEARTBEAT_LED_PIN, GPIO_OUTPUT, GPIO_PUPD_NONE);

  // Configure UART heartbeat
  channel_t my_uart_channel;
  serial_args_t uart_cfg = {
      .baud_rate = 115200, .uart = UART2, .timeout = 1000};

  while (get_handler(CHANNEL_TYPE_SERIAL, &my_uart_channel, &uart_cfg,
                     uart2_packet_recv_callback) != NONE) {
    v_delay(500);
  }
  while (1) {
    hal_gpio_digitalwrite(_HEARTBEAT_LED_PIN, GPIO_HIGH);
    v_delay(time_period_half);
    hal_gpio_digitalwrite(_HEARTBEAT_LED_PIN, GPIO_LOW);
    v_delay(time_period_half);
    send_packet(&my_uart_channel, PACKET_TYPE_HEARTBEAT, NULL, 0);
  }
}