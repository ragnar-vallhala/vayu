#include "comm/navlink_tx.h"
#include "comm/channel.h"     /* write_channel, channel_t */
#include "comm/serializer.h"  /* send_packet */
#include "comm/perf_packet.h" /* PERF_TASKNAME_MAX */
#include "sys/sys_utils.h"    /* get_device_id */
#include "utils.h"            /* v_memcpy */
#include "navlink_msgs.h"     /* generated codec — included ONLY here + navlink_router.c */
#include <stdint.h>

extern channel_t g_telemetry_channel; /* defined in telemetry_task.c */

/* deg -> rad: attitude_t angles are degrees (est/sensor_fusion.c to_degrees()),
 * NavLink v2 ATTITUDE_EULER carries radians. */
#define ATT_DEG2RAD 0.017453292519943295f

/* --- periodic telemetry --------------------------------------------------- */

void navlink_tx_log(const char *buf, uint8_t len) {
  send_packet(&g_telemetry_channel, PACKET_TYPE_LOG, (uint8_t *)buf, len);
}

void navlink_tx_heartbeat(void) {
  send_packet(&g_telemetry_channel, PACKET_TYPE_HEARTBEAT, NULL, 0);
}

void navlink_tx_system_state(int sys_state) {
  uint8_t payload[6];
  payload[0] = 0x04; /* SYSTEM_ORIGIN_SYS_STATE */
  payload[1] = 0x00; /* reserved/padding */
  float current_state = (float)sys_state;
  v_memcpy(&payload[2], &current_state, 4);
  send_packet(&g_telemetry_channel, PACKET_TYPE_SYSTEM_STATUS, payload, 6);
}

void navlink_tx_flight_mode(uint8_t mode, uint8_t source) {
  uint8_t payload[4];
  payload[0] = SYSTEM_ORIGIN_FLIGHT_MODE;
  payload[1] = 0x00; /* reserved/padding */
  payload[2] = mode;
  payload[3] = source;
  send_packet(&g_telemetry_channel, PACKET_TYPE_SYSTEM_STATUS, payload, 4);
}

void navlink_tx_health(uint32_t tx_overflow, uint32_t imu_drop,
                       uint32_t log_wrap) {
  uint8_t payload[14];
  payload[0] = SYSTEM_ORIGIN_HEALTH;
  payload[1] = 0x00; /* reserved/padding */
  v_memcpy(&payload[2], &tx_overflow, 4);
  v_memcpy(&payload[6], &imu_drop, 4);
  v_memcpy(&payload[10], &log_wrap, 4);
  send_packet(&g_telemetry_channel, PACKET_TYPE_SYSTEM_STATUS, payload, 14);
}

void navlink_tx_pid_error(const control_telemetry_t *c) {
  uint8_t payload[2 + sizeof(control_telemetry_t)];
  payload[0] = SYSTEM_ORIGIN_PID_ERROR;
  payload[1] = 18; /* number of float elements */
  v_memcpy(&payload[2], c, sizeof(control_telemetry_t));
  send_packet(&g_telemetry_channel, PACKET_TYPE_SYSTEM_STATUS, payload,
              (uint8_t)sizeof(payload));
}

void navlink_tx_est_perf(const est_perf_telemetry_t *e) {
  uint8_t payload[2 + sizeof(est_perf_telemetry_t)];
  payload[0] = SYSTEM_ORIGIN_EST_PERF;
  payload[1] = 4; /* 4 floats: peak_us, mean_us, decim, rate_hz */
  v_memcpy(&payload[2], e, sizeof(est_perf_telemetry_t));
  send_packet(&g_telemetry_channel, PACKET_TYPE_SYSTEM_STATUS, payload,
              (uint8_t)sizeof(payload));
}

void navlink_tx_imu_full(const float floats10[10]) {
  send_packet(&g_telemetry_channel, PACKET_TYPE_IMU_DATA_FULL,
              (uint8_t *)floats10, 40);
}

void navlink_tx_imu_compressed(const uint16_t delta_f16[10]) {
  send_packet(&g_telemetry_channel, PACKET_TYPE_IMU_DATA_COMPRESSED,
              (uint8_t *)delta_f16, 20);
}

void navlink_tx_attitude(const attitude_t *att_deg) {
  /* The single v2-encoded message today; the v1->v2 migration replicates this
   * shape for the others (navlink/INTEGRATION.md). */
  static uint8_t s_att_tx_seq = 0;
  navlink_attitude_euler_t a = {0};
  a.roll = att_deg->roll * ATT_DEG2RAD;
  a.pitch = att_deg->pitch * ATT_DEG2RAD;
  a.yaw = att_deg->yaw * ATT_DEG2RAD;
  uint8_t frame[NAVLINK_MAX_FRAME];
  size_t n = navlink_attitude_euler_encode(frame, &a, s_att_tx_seq++,
                                           get_device_id(), 1);
  write_channel(g_telemetry_channel, frame, (uint16_t)n);
}

void navlink_tx_rc_channels(const ibus_data_t *rc) {
  send_packet(&g_telemetry_channel, PACKET_TYPE_RC_CHANNELS,
              (uint8_t *)rc->channels, sizeof(rc->channels));
}

void navlink_tx_motor(const motor_outputs_t *m) {
  send_packet(&g_telemetry_channel, PACKET_TYPE_MOTOR_TELEMETRY, (uint8_t *)m,
              sizeof(*m));
}

void navlink_tx_calibration(const uint8_t *buf, uint8_t len) {
  send_packet(&g_telemetry_channel, PACKET_TYPE_SYSTEM_STATUS, (uint8_t *)buf,
              len);
}

/* --- command responses ---------------------------------------------------- */

void navlink_tx_time_sync_response(const time_sync_payload_t *out) {
  send_packet(&g_telemetry_channel, PACKET_TYPE_TIME_SYNC, (uint8_t *)out,
              (uint8_t)sizeof(*out));
}

void navlink_tx_perf_taskname(uint8_t id, const char *name) {
  uint8_t buf[1 + PERF_TASKNAME_MAX];
  buf[0] = id;
  uint8_t n = 0;
  while (n < PERF_TASKNAME_MAX - 1 && name[n]) {
    buf[1 + n] = (uint8_t)name[n];
    n++;
  }
  buf[1 + n] = '\0';
  send_packet(&g_telemetry_channel, PACKET_TYPE_PERF_TASKNAME, buf,
              (uint8_t)(2 + n));
}
