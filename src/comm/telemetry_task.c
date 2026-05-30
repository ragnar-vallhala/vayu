#include "actuator/actuator.h"
#include "comm/channel.h"
#include "comm/comm_types.h"
#include "comm/ibus.h"
#include "comm/rc_buffer.h"
#include "comm/serializer.h"
#include "logger/logger.h"
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
#include "vfs.h"
#include <stdint.h>

channel_t g_telemetry_channel = {0};

void imu_telemetry_task(void *args) {
  (void)args;
  static bmx160_all_reading_t samples;
  static float current_floats[10];  // Acc[3], Gyr[3], Mag[3], Temp
  static float previous_floats[10]; // For delta calculation
  static bool first_packet = true;
  static uint32_t packet_counter = 0;
  static ibus_data_t rc_data;
  static motor_outputs_t m_data;
  static control_telemetry_t c_data;
  static attitude_t att;
  static imu_calibration_telemetry_t imu_calibration_telemetry;

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

    // Base loop is v_delay(6) below => ~166 Hz tick.
    // COMM-TEL-002 / SYS-TEL-001: heartbeat must be >= 1 Hz. 166 ticks x
    // 6 ms = 996 ms => 1.004 Hz (the previous 150 ticks = 900 ms = 1.11 Hz
    // was mislabelled "1 Hz"; 166 is the closest period to 1 s that still
    // satisfies >= 1 Hz).
#define HEARTBEAT_PERIOD_TICKS 166
    bool send_heartbeat = (packet_counter % HEARTBEAT_PERIOD_TICKS == 0); // ~1 Hz
    bool send_full = (packet_counter % 100 == 0);      // 1 Hz
    bool send_comp = (packet_counter % 6 == 0);        // 25 Hz
    bool send_att = (packet_counter % 15 == 0);        // 10 Hz
    bool send_rc = (packet_counter % 15 == 0);         // 10 Hz
    bool send_status = (packet_counter % 50 == 0);     // 2 Hz
    bool send_motor = (packet_counter % 8 == 0);       // 18 Hz
    bool send_pid_err = (packet_counter % 8 == 0);     // 18 Hz
    bool send_log = (packet_counter % 10 == 0);        // 15 Hz
    if (send_log) {
      /* LOG-TXT-002: drain the text-log queue to the LOG channel.
       * @implements LOG-TXT-002 */
      static char log_buf[VAYU_LOG_QUEUE_SIZE];
      uint8_t len = mpmc_pop_bulk(&vayu_log_queue, log_buf, sizeof(log_buf));
      if (len > 0) {
        send_packet(&g_telemetry_channel, PACKET_TYPE_LOG, (uint8_t *)log_buf,
                    len);
      }
    }
    if (send_heartbeat) {
      /* @implements COMM-TEL-002 */
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

      /* Surface the health counters as a HEALTH status:
       *   [origin][pad][tx_overflow:4][imu_drop:4][log_wrap:4]
       * @implements COMM-CH-002, SNS-BUF-002, LOG-SD-002 */
      uint8_t health_payload[14];
      health_payload[0] = SYSTEM_ORIGIN_HEALTH;
      health_payload[1] = 0x00; // reserved/padding
      uint32_t tx_overflow = channel_tx_overflow_count();
      uint32_t imu_drop = imu_buffer_drop_count();
      uint32_t log_wrap = logger_wrap_count_total();
      v_memcpy(&health_payload[2], &tx_overflow, 4);
      v_memcpy(&health_payload[6], &imu_drop, 4);
      v_memcpy(&health_payload[10], &log_wrap, 4);
      send_packet(&g_telemetry_channel, PACKET_TYPE_SYSTEM_STATUS,
                  health_payload, 14);
    }
    if (send_pid_err && control_telemetry_queue_pop(&c_data)) {
      uint8_t payload[74];
      payload[0] = SYSTEM_ORIGIN_PID_ERROR;
      payload[1] = 18; // Number of elements (18 floats)
      v_memcpy(&payload[2], &c_data, sizeof(control_telemetry_t));
      // 2 bytes header + 18 * 4 bytes = 74 bytes
      send_packet(&g_telemetry_channel, PACKET_TYPE_SYSTEM_STATUS, payload, 74);
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

    if (send_motor && motor_telemetry_queue_pop(&m_data)) {
      send_packet(&g_telemetry_channel, PACKET_TYPE_MOTOR_TELEMETRY,
                  (uint8_t *)&m_data, sizeof(m_data));
    }
    if (imu_queue_calibration_telemetry_pop(&imu_calibration_telemetry)) {
      send_packet(&g_telemetry_channel, PACKET_TYPE_SYSTEM_STATUS,
                  imu_calibration_telemetry.buffer,
                  imu_calibration_telemetry.size);
    }
    packet_counter++;
    v_delay(6); // ~166 Hz
  }
}
