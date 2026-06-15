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
  /* v2 FLIGHT_MODE (msgid 3); replaces v1 SYSTEM_STATUS origin FLIGHT_MODE. */
  static uint8_t seq = 0;
  navlink_flight_mode_t msg = {0};
  msg.mode = mode;
  msg.source = source;
  uint8_t frame[NAVLINK_MAX_FRAME];
  size_t n = navlink_flight_mode_encode(frame, &msg, seq++, get_device_id(), 1);
  write_channel(g_telemetry_channel, frame, (uint16_t)n);
}

void navlink_tx_health(uint32_t tx_overflow, uint32_t imu_drop,
                       uint32_t log_wrap) {
  /* v2 SYSTEM_HEALTH (msgid 2); replaces v1 SYSTEM_STATUS origin HEALTH. The
   * new cpu_load field has no firmware source yet -> 0. */
  static uint8_t seq = 0;
  navlink_system_health_t msg = {0};
  msg.tx_overflow = tx_overflow;
  msg.imu_drop = imu_drop;
  msg.log_wrap = log_wrap;
  msg.cpu_load = 0;
  uint8_t frame[NAVLINK_MAX_FRAME];
  size_t n =
      navlink_system_health_encode(frame, &msg, seq++, get_device_id(), 1);
  write_channel(g_telemetry_channel, frame, (uint16_t)n);
}

void navlink_tx_pid_error(const control_telemetry_t *c) {
  /* v2 CONTROL_TRACE (msgid 1030); replaces v1 SYSTEM_STATUS origin PID_ERROR.
   * control_telemetry_t and navlink_control_trace_t are both 18 contiguous f32
   * in identical field order, so a straight copy reproduces the wire payload. */
  static uint8_t seq = 0;
  navlink_control_trace_t msg;
  v_memcpy(&msg, c, sizeof(msg));
  uint8_t frame[NAVLINK_MAX_FRAME];
  size_t n =
      navlink_control_trace_encode(frame, &msg, seq++, get_device_id(), 1);
  write_channel(g_telemetry_channel, frame, (uint16_t)n);
}

void navlink_tx_est_perf(const est_perf_telemetry_t *e) {
  /* v2 EST_PERF (msgid 1033); replaces v1 SYSTEM_STATUS origin EST_PERF. */
  static uint8_t seq = 0;
  navlink_est_perf_t msg;
  msg.peak_us = e->peak_us;
  msg.mean_us = e->mean_us;
  msg.decimation = e->decim;
  msg.rate_hz = e->rate_hz;
  uint8_t frame[NAVLINK_MAX_FRAME];
  size_t n = navlink_est_perf_encode(frame, &msg, seq++, get_device_id(), 1);
  write_channel(g_telemetry_channel, frame, (uint16_t)n);
}

void navlink_tx_imu_full(const float floats10[10]) {
  /* v2 IMU_RAW (msgid 1024); replaces v1 PACKET_TYPE_IMU_DATA_FULL. floats10 is
   * acc[3], gyr[3], mag[3], temp. sample_time_us has no source here -> 0. */
  static uint8_t seq = 0;
  navlink_imu_raw_t msg = {0};
  msg.acc[0] = floats10[0];
  msg.acc[1] = floats10[1];
  msg.acc[2] = floats10[2];
  msg.gyr[0] = floats10[3];
  msg.gyr[1] = floats10[4];
  msg.gyr[2] = floats10[5];
  msg.mag[0] = floats10[6];
  msg.mag[1] = floats10[7];
  msg.mag[2] = floats10[8];
  msg.temp = floats10[9];
  msg.sample_time_us = 0;
  uint8_t frame[NAVLINK_MAX_FRAME];
  size_t n = navlink_imu_raw_encode(frame, &msg, seq++, get_device_id(), 1);
  write_channel(g_telemetry_channel, frame, (uint16_t)n);
}

void navlink_tx_imu_compressed(const uint16_t delta_f16[10]) {
  /* v2 IMU_COMPRESSED (msgid 1025); replaces v1 PACKET_TYPE_IMU_DATA_COMPRESSED.
   * The 10 binary16 delta bit patterns ride as u16 (delta vs the last IMU_RAW;
   * the GCS reconstructs). ref_seq is unused by the GCS today -> 0. */
  static uint8_t seq = 0;
  navlink_imu_compressed_t msg = {0};
  msg.ref_seq = 0;
  msg._pad = 0;
  for (int i = 0; i < 10; i++)
    msg.delta[i] = delta_f16[i];
  uint8_t frame[NAVLINK_MAX_FRAME];
  size_t n =
      navlink_imu_compressed_encode(frame, &msg, seq++, get_device_id(), 1);
  write_channel(g_telemetry_channel, frame, (uint16_t)n);
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
  /* v2 RC_CHANNELS (msgid 1028); replaces v1 PACKET_TYPE_RC_CHANNELS. v1 carried
   * IBUS_MAX_CHANNELS (14) u16; v2 widens to 18, so the tail stays 0. rssi has
   * no source -> 0; count reports how many channels are populated. */
  static uint8_t seq = 0;
  navlink_rc_channels_t msg = {0};
  for (int i = 0; i < IBUS_MAX_CHANNELS && i < 18; i++)
    msg.chan[i] = rc->channels[i];
  msg.rssi = 0;
  msg.count = (uint8_t)IBUS_MAX_CHANNELS;
  uint8_t frame[NAVLINK_MAX_FRAME];
  size_t n = navlink_rc_channels_encode(frame, &msg, seq++, get_device_id(), 1);
  write_channel(g_telemetry_channel, frame, (uint16_t)n);
}

void navlink_tx_motor(const motor_outputs_t *m) {
  /* v2 MOTOR_TELEMETRY (msgid 1029); replaces v1 PACKET_TYPE_MOTOR_TELEMETRY.
   * v1 carried 4 f32; v2 allows up to 8, so motors 4..7 stay 0. */
  static uint8_t seq = 0;
  navlink_motor_telemetry_t msg = {0};
  msg.cmd[0] = m->m1;
  msg.cmd[1] = m->m2;
  msg.cmd[2] = m->m3;
  msg.cmd[3] = m->m4;
  uint8_t frame[NAVLINK_MAX_FRAME];
  size_t n =
      navlink_motor_telemetry_encode(frame, &msg, seq++, get_device_id(), 1);
  write_channel(g_telemetry_channel, frame, (uint16_t)n);
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
