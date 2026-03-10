#include "comm/comm_types.h"
#include "comm/ibus.h"
#include "comm/serializer.h"
#include "core/cortex-m4/uart.h"
#include "maths/sensor_fusion.h"
#include "sensor/bmx160.h"
#include "sensor/imu_buffer.h"
#include "utils.h"
#include "utils/math_utils.h"
#include "utils/utils.h"
#include "vaios.h"
#include "variables.h"
#include <stdint.h>
#include <string.h>

void imu_telemetry_task(void *args) {
  (void)args;
  static bmx160_all_reading_t samples[IMU_BUFFER_SIZE];
  float current_floats[10];  // Acc[3], Gyr[3], Mag[3], Temp
  float previous_floats[10]; // For delta calculation
  bool first_packet = true;
  uint32_t packet_counter = 0;

  channel_t uart_channel;
  serial_args_t uart_args = {
      .baud_rate = 115200, .uart = UART2, .timeout = 100};

  if (get_handler(CHANNEL_TYPE_SERIAL, &uart_channel, &uart_args,
                  uart2_packet_recv_callback) != NONE) {
    return;
  }

  while (1) {
    int count = imu_buffer_pop_all(samples, IMU_BUFFER_SIZE);

    if (count > 0) {
      float sum_acc[3] = {0, 0, 0};
      float sum_gyr[3] = {0, 0, 0};
      float sum_mag[3] = {0, 0, 0};
      float sum_temp = 0;

      for (int i = 0; i < count; i++) {
        for (int j = 0; j < 3; j++) {
          sum_acc[j] += samples[i].converted.acc[j];
          sum_gyr[j] += samples[i].converted.gyr[j];
          sum_mag[j] += samples[i].converted.mag[j];
        }
        sum_temp += samples[i].converted.temp;
      }

      current_floats[0] = (float)sum_acc[0] / count;
      current_floats[1] = (float)sum_acc[1] / count;
      current_floats[2] = (float)sum_acc[2] / count;
      current_floats[3] = (float)sum_gyr[0] / count;
      current_floats[4] = (float)sum_gyr[1] / count;
      current_floats[5] = (float)sum_gyr[2] / count;
      current_floats[6] = (float)sum_mag[0] / count;
      current_floats[7] = (float)sum_mag[1] / count;
      current_floats[8] = (float)sum_mag[2] / count;
      current_floats[9] = (float)sum_temp / count;
    }

    // 150 Hz Base Loop (approx 6.66ms)
    // - 1 Hz: Full IMU (every 150 ticks)
    // - 50 Hz: Compressed IMU (every 3 ticks)
    // - 10 Hz: Attitude (every 15 ticks)

    bool send_full = (packet_counter % 150 == 0);
    bool send_comp = (packet_counter % 3 == 0);
    bool send_att = (packet_counter % 15 == 0);
    bool send_rc = (packet_counter % 15 == 0); // 10 Hz

    if (send_full) {
      send_packet(&uart_channel, PACKET_TYPE_IMU_DATA_FULL,
                  (uint8_t *)current_floats, 40);
      memcpy(previous_floats, current_floats, sizeof(current_floats));
      first_packet = false;
    } else if (send_comp && !first_packet) {
      uint16_t delta_payload[10];
      for (int i = 0; i < 10; i++) {
        delta_payload[i] =
            float32_to_float16(current_floats[i] - previous_floats[i]);
      }
      send_packet(&uart_channel, PACKET_TYPE_IMU_DATA_COMPRESSED,
                  (uint8_t *)delta_payload, 20);
      memcpy(previous_floats, current_floats, sizeof(current_floats));
    }

    if (send_att) {
      attitude_t att;
      bmx160_get_attitude(&att);
      float att_vals[3] = {att.roll, att.pitch, att.yaw};
      send_packet(&uart_channel, PACKET_TYPE_ATTITUDE, (uint8_t *)att_vals, 12);
    }

    if (send_rc) {
      send_packet(&uart_channel, PACKET_TYPE_RC_CHANNELS,
                  (uint8_t *)rc_channels, 28);
    }

    packet_counter++;
    v_delay(6); // ~150 Hz
  }
}
