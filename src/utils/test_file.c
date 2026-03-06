#include "utils/test_file.h"
#include "comm/channel.h"
#include "comm/comm_types.h"
#include "comm/serializer.h"
#include "utils.h"
#include <stdint.h>

void test_task(void *args) {
  channel_t my_uart_channel;
  serial_args_t uart_cfg = {
      .baud_rate = 115200, .uart = UART2, .timeout = 1000};
  err_t status = get_handler(CHANNEL_TYPE_SERIAL, &my_uart_channel, &uart_cfg);

  if (status != NONE) {
    // Failed to allocate a handler or initialize UART
    return;
  }
  uint8_t data[4] = {0x1, 0x2, 0x3, 0x4};
  send_packet(&my_uart_channel, PACKET_TYPE_IMU_DATA_FULL, data, 4);

  while (1)
    ;
}