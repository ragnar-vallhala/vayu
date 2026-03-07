#include "comm/comm_types.h"
#include "comm/serializer.h"
#include "core/cortex-m4/uart.h"
#include "sensor/imu_buffer.h"
#include "utils/utils.h"
#include "vaios.h"
#include "variables.h"
#include <stdint.h>

void imu_telemetry_task(void *args) {
  (void)args;
  static bmx160_all_reading_t samples[IMU_BUFFER_SIZE];
  int16_t payload[10]; // Acc[3], Gyr[3], Mag[3], Temp
  channel_t uart_channel;

  // ... existing initialization ...
  serial_args_t uart_args = {
      .baud_rate = 115200, .uart = UART2, .timeout = 100};

  if (get_handler(CHANNEL_TYPE_SERIAL, &uart_channel, &uart_args,
                  uart2_packet_recv_callback) != NONE) {
    return;
  }

  while (1) {
    int count = imu_buffer_pop_all(samples, IMU_BUFFER_SIZE);

    if (count > 0) {
      int32_t sum_acc[3] = {0, 0, 0};
      int32_t sum_gyr[3] = {0, 0, 0};
      int32_t sum_mag[3] = {0, 0, 0};
      int32_t sum_temp = 0;

      for (int i = 0; i < count; i++) {
        for (int j = 0; j < 3; j++) {
          sum_acc[j] += samples[i].raw.acc[j];
          sum_gyr[j] += samples[i].raw.gyr[j];
          sum_mag[j] += samples[i].raw.mag[j];
        }
        sum_temp += samples[i].raw.temp;
      }

      // Calculate averages
      payload[0] = (int16_t)(sum_acc[0] / count);
      payload[1] = (int16_t)(sum_acc[1] / count);
      payload[2] = (int16_t)(sum_acc[2] / count);
      payload[3] = (int16_t)(sum_gyr[0] / count);
      payload[4] = (int16_t)(sum_gyr[1] / count);
      payload[5] = (int16_t)(sum_gyr[2] / count);
      payload[6] = (int16_t)(sum_mag[0] / count);
      payload[7] = (int16_t)(sum_mag[1] / count);
      payload[8] = (int16_t)(sum_mag[2] / count);
      payload[9] = (int16_t)(sum_temp / count);

      // Send packet (20 bytes payload)
      send_packet(&uart_channel, PACKET_TYPE_IMU_DATA_FULL, (uint8_t *)payload,
                  20);
    }

    // Run at 100Hz
    v_delay(10);
  }
}
