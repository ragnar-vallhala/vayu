#include "comm/comm_types.h"
#include "comm/ibus.h"
#include "comm/serializer.h"
#include "vaios_app_config.h"
#include "core/cortex-m4/uart.h"
#include "maths/sensor_fusion.h"
#include "sensor/bmx160.h"
#include "sensor/imu_buffer.h"
#include "sys/state.h"
#include "utils.h"
#include "utils/math_utils.h"
#include "utils/utils.h"
#include "vaios.h"
#include "variables.h"
#include <stdint.h>

void imu_telemetry_task(void *args) {
  (void)args;
  static bmx160_all_reading_t samples;
  float current_floats[10];  // Acc[3], Gyr[3], Mag[3], Temp
  float previous_floats[10]; // For delta calculation
  bool first_packet = true;
  uint32_t packet_counter = 0;

  while (1) {
    int count = imu_distribution_queue_peek(&samples);

    if (count > 0) {
      current_floats[0] = (float)samples.converted.acc[0];
      current_floats[1] = (float)samples.converted.acc[1];
      current_floats[2] = (float)samples.converted.acc[2];
      current_floats[3] = (float)samples.converted.gyr[0];
      current_floats[4] = (float)samples.converted.gyr[1];
      current_floats[5] = (float)samples.converted.gyr[2];
      current_floats[6] = (float)samples.converted.mag_compensated[0];
      current_floats[7] = (float)samples.converted.mag_compensated[1];
      current_floats[8] = (float)samples.converted.mag_compensated[2];
      current_floats[9] = (float)samples.converted.temp;
    }

    // 150 Hz Base Loop (approx 6.66ms)
    // - 1 Hz: Full IMU (every 150 ticks)
    // - 50 Hz: Compressed IMU (every 3 ticks)
    // - 10 Hz: Attitude (every 15 ticks)

    bool send_full = (packet_counter % 100 == 0);  // 1 Hz
    bool send_comp = (packet_counter % 3 == 0);    // 50 Hz
    bool send_att = (packet_counter % 10 == 0);    // 10 Hz
    bool send_rc = (packet_counter % 10 == 0);     // 10 Hz
    bool send_status = (packet_counter % 50 == 0); // 2 Hz

    if (send_status) {
      uint8_t state_payload[6];
      state_payload[0] = 0x04; // SYSTEM_ORIGIN_SYS_STATE
      state_payload[1] = 0x00; // Reserved/Padding
      float current_state = (float)system_state_get();
      v_memcpy(&state_payload[2], &current_state, 4);

      send_packet(&g_telemetry_channel, PACKET_TYPE_SYSTEM_STATUS,
                  state_payload, 6);
    }

    if (send_full) {
      send_packet(&g_telemetry_channel, PACKET_TYPE_IMU_DATA_FULL,
                  (uint8_t *)current_floats, 40);
      v_memcpy(previous_floats, current_floats, sizeof(current_floats));
      first_packet = false;
    } else if (send_comp && !first_packet) {
      uint16_t delta_payload[10];
      for (int i = 0; i < 10; i++) {
        delta_payload[i] =
            float32_to_float16(current_floats[i] - previous_floats[i]);
      }
      send_packet(&g_telemetry_channel, PACKET_TYPE_IMU_DATA_COMPRESSED,
                  (uint8_t *)delta_payload, 20);
      v_memcpy(previous_floats, current_floats, sizeof(current_floats));
    }

    if (send_att) {
      attitude_t att;
      bmx160_get_attitude(&att);
      float att_vals[3] = {att.roll, att.pitch, att.yaw};
      send_packet(&g_telemetry_channel, PACKET_TYPE_ATTITUDE,
                  (uint8_t *)att_vals, 12);
    }

    if (send_rc) {
      send_packet(&g_telemetry_channel, PACKET_TYPE_RC_CHANNELS,
                  (uint8_t *)rc_channels, 28);
    }

    packet_counter++;
    v_delay(6); // ~166 Hz
  }
}
