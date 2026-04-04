#include "comm/comm_types.h"
#include "comm/ibus.h"
#include "comm/rc_buffer.h"
#include "comm/serializer.h"
#include "maths/control_buffer.h"
#include "maths/sensor_fusion.h"
#include "sensor/bmx160.h"
#include "sensor/imu_buffer.h"
#include "sys/state.h"
#include "utils.h"
#include "utils/math_utils.h"
#include "utils/utils.h"
#include "vaios.h"
#include "vaios_app_config.h"
#include "variables.h"
#include <stdint.h>

channel_t g_telemetry_channel = {0};

void imu_telemetry_task(void *args) {
  (void)args;
  static bmx160_all_reading_t samples;
  static float current_floats[10];  // Acc[3], Gyr[3], Mag[3], Temp
  static float previous_floats[10]; // For delta calculation
  static bool first_packet = true;
  static uint32_t packet_counter = 0;
  static pid_error_data_t e_data;
  static motor_pwm_data_t m_data;
  static ibus_data_t rc_data;
  static attitude_t att;

  while (1) {
    if (imu_queue_telemetry_pop(&samples)) {
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
    bool send_heartbeat = (packet_counter % 150 == 0); // 1 Hz
    bool send_full = (packet_counter % 100 == 0);      // 1 Hz
    bool send_comp = (packet_counter % 6 == 0);        // 25 Hz
    bool send_att = (packet_counter % 15 == 0);        // 10 Hz
    bool send_rc = (packet_counter % 15 == 0);         // 10 Hz
    bool send_status = (packet_counter % 50 == 0);     // 2 Hz
    bool send_motor = (packet_counter % 15 == 0);      // 10 Hz
    bool send_pid_err = (packet_counter % 15 == 0);    // 10 Hz
    bool send_log = (packet_counter % 10 == 0);        // 15 Hz
    if (send_log) {
      static char log_buf[VAYU_LOG_QUEUE_SIZE];
      uint8_t len = mpmc_pop_bulk(&vayu_log_queue, log_buf, sizeof(log_buf));
      if (len > 0) {
        send_packet(&g_telemetry_channel, PACKET_TYPE_LOG, (uint8_t *)log_buf,
                    len);
      }
    }
    if (send_heartbeat) {
      send_packet(&g_telemetry_channel, PACKET_TYPE_HEARTBEAT, NULL, 0);
    }
    if (send_status) {
      uint8_t state_payload[6];
      state_payload[0] = 0x04; // SYSTEM_ORIGIN_SYS_STATE
      state_payload[1] = 0x00; // Reserved/Padding
      float current_state = (float)system_state_get();
      v_memcpy(&state_payload[2], &current_state, 4);

      send_packet(&g_telemetry_channel, PACKET_TYPE_SYSTEM_STATUS,
                  state_payload, 6);
    }
    if (send_pid_err && pid_error_queue_pop(&e_data)) {
      uint8_t pid_payload[14];
      pid_payload[0] = 0x05; // SYSTEM_ORIGIN_PID_ERROR
      pid_payload[1] = 3;    // Number of elements (3 floats)
      v_memcpy(&pid_payload[2], e_data.errors, sizeof(e_data.errors));
      send_packet(&g_telemetry_channel, PACKET_TYPE_SYSTEM_STATUS,
                  pid_payload, 14);
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

    if (send_att && attitude_queue_telemetry_pop(&att)) {
      float att_vals[3] = {att.roll, att.pitch, att.yaw};
      send_packet(&g_telemetry_channel, PACKET_TYPE_ATTITUDE,
                  (uint8_t *)att_vals, 12);
    }

    if (send_rc && rc_queue_telemetry_pop(&rc_data)) {
      send_packet(&g_telemetry_channel, PACKET_TYPE_RC_CHANNELS,
                  (uint8_t *)rc_data.channels, sizeof(rc_data.channels));
    }

    if (send_motor && motor_queue_pop(&m_data)) {
      send_packet(&g_telemetry_channel, PACKET_TYPE_MOTOR_TELEMETRY,
                  (uint8_t *)m_data.motors, sizeof(m_data.motors));
    }
    packet_counter++;
    v_delay(6); // ~166 Hz
  }
}
