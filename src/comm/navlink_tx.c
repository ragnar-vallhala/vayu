#include "comm/navlink_tx.h"
#include "comm/channel.h"     /* write_channel, channel_t */
#include "sys/state.h"        /* system_state_get, sys_state_t */
#include "sys/sys_utils.h"    /* get_device_id */
#include "utils.h"            /* v_memcpy, v_get_ticks */
#include "navlink_msgs.h"     /* generated codec — included ONLY here + navlink_router.c */
#include <stdint.h>

extern channel_t g_telemetry_channel; /* defined in telemetry_task.c */

/* deg -> rad: attitude_t angles are degrees (est/sensor_fusion.c to_degrees()),
 * NavLink v2 ATTITUDE_EULER carries radians. */
#define ATT_DEG2RAD 0.017453292519943295f

/* --- periodic telemetry --------------------------------------------------- */

void navlink_tx_log(const char *buf, uint8_t len) {
  /* v2 STATUSTEXT (msgid 4); replaces v1 PACKET_TYPE_LOG. The v1 payload is a
   * bulk drain of newline-delimited log lines; emit one STATUSTEXT per line.
   * Each NavLink text field holds up to 50 chars; a longer line spills into the
   * next frame. The GCS renders one log line per STATUSTEXT. */
  static uint8_t seq = 0;
  uint8_t i = 0;
  while (i < len) {
    navlink_statustext_t msg = {0};
    msg.severity = 6; /* NAVLINK_SEVERITY_INFO */
    uint8_t j = 0;
    while (i < len && buf[i] != '\n' && j < 50) {
      msg.text[j++] = buf[i++];
    }
    if (i < len && buf[i] == '\n') {
      i++; /* consume the line delimiter */
    }
    if (j == 0) {
      continue; /* skip empty lines */
    }
    uint8_t frame[NAVLINK_MAX_FRAME];
    size_t n = navlink_statustext_encode(frame, &msg, seq++, get_device_id(), 1);
    write_channel(g_telemetry_channel, frame, (uint16_t)n);
  }
}

void navlink_tx_sysid_sample(uint16_t start, uint16_t total, uint16_t hz,
                             uint8_t axis, uint8_t count, const int16_t *sp,
                             const int16_t *gyro) {
  static uint8_t seq = 0;
  navlink_sysid_sample_t msg = {0};
  msg.start_index = start;
  msg.total = total;
  msg.capture_hz = hz;
  msg.axis = axis;
  msg.count = count;
  for (uint8_t i = 0; i < count && i < 10; i++) {
    msg.sp[i] = sp[i];
    msg.gyro[i] = gyro[i];
  }
  uint8_t frame[NAVLINK_MAX_FRAME];
  size_t n = navlink_sysid_sample_encode(frame, &msg, seq++, get_device_id(), 1);
  write_channel(g_telemetry_channel, frame, (uint16_t)n);
}

void navlink_tx_heartbeat(void) {
  /* v2 HEARTBEAT (msgid 0); replaces the v1 empty heartbeat AND folds in the
   * former SYSTEM_STATUS SYS_STATE origin. The firmware flight-state machine is
   * a one-hot bitmask (sys/state.h: 0x1..0x100); the v2 nav_state enum is its
   * sequential index, so the set-bit position (ctz) maps one to the other. */
  static uint8_t seq = 0;
  navlink_heartbeat_t msg = {0};
  msg.type = 0;          /* vehicle type — unused by the GCS today */
  msg.autopilot = 0;
  msg.base_mode = 0;
  msg.system_status = 0;
  sys_state_t st = system_state_get();
  msg.nav_state = (uint8_t)(st ? __builtin_ctz((unsigned)st) : 0);
  msg.capabilities = 0;
  msg.timestamp = (uint32_t)v_get_ticks();
  uint8_t frame[NAVLINK_MAX_FRAME];
  size_t n = navlink_heartbeat_encode(frame, &msg, seq++, get_device_id(), 1);
  write_channel(g_telemetry_channel, frame, (uint16_t)n);
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

void navlink_tx_baro(float pressure_pa, float temperature_c, float humidity_rh,
                     float altitude_m) {
  /* v2 BARO (msgid 1039); BME280 baro/humidity. No v1 equivalent. */
  static uint8_t seq = 0;
  navlink_baro_t b = {0};
  b.pressure = pressure_pa;
  b.temperature = temperature_c;
  b.humidity = humidity_rh;
  b.altitude = altitude_m;
  uint8_t frame[NAVLINK_MAX_FRAME];
  size_t n = navlink_baro_encode(frame, &b, seq++, get_device_id(), 1);
  write_channel(g_telemetry_channel, frame, (uint16_t)n);
}

void navlink_tx_vertical_state(const vertical_state_t *vs) {
  /* v2 VERTICAL_STATE (msgid 1040); fused vertical estimate + raw baro alt. */
  static uint8_t seq = 0;
  navlink_vertical_state_t m = {0};
  m.altitude = vs->altitude;
  m.climb_rate = vs->climb_rate;
  m.vertical_accel = vs->vertical_accel;
  m.baro_altitude = vs->baro_altitude;
  m.agl = vs->agl;
  m.valid = vs->valid ? 1u : 0u;
  uint8_t frame[NAVLINK_MAX_FRAME];
  size_t n = navlink_vertical_state_encode(frame, &m, seq++, get_device_id(), 1);
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
  /* v2 CALIBRATION_STATUS (msgid 12320); replaces v1 SYSTEM_STATUS origin 0x01.
   * v1 buffer layout: [0]=origin [1]=nargs [2]=step [3..]=float payload. For
   * the MAG_AXIS_COVERAGE step (8, len>=15) the payload is coverage x/y/z (3
   * f32); otherwise [3..6] is a progress float (0..100 for the PROGRESS step,
   * 0 for the orientation-instruction steps). */
  static uint8_t seq = 0;
  if (len < 3) {
    return;
  }
  navlink_calibration_status_t msg = {0};
  msg.step = buf[2];
  if (msg.step == 8 /* MAG_AXIS_COVERAGE */ && len >= 15) {
    v_memcpy(&msg.coverage[0], &buf[3], 4);
    v_memcpy(&msg.coverage[1], &buf[7], 4);
    v_memcpy(&msg.coverage[2], &buf[11], 4);
  } else if (len >= 7) {
    float p;
    v_memcpy(&p, &buf[3], 4);
    msg.progress = (uint8_t)p;
  }
  uint8_t frame[NAVLINK_MAX_FRAME];
  size_t n =
      navlink_calibration_status_encode(frame, &msg, seq++, get_device_id(), 1);
  write_channel(g_telemetry_channel, frame, (uint16_t)n);
}

/* --- PERF (one v2 message per row; seq ties a report together) ------------- */

void navlink_tx_perf_global(const perf_global_body_t *g, uint32_t seq) {
  static uint8_t s = 0;
  navlink_perf_global_t m = {0};
  m.seq = seq;
  m.flags = g->flags;
  m.total_tasks = g->total_tasks;
  m.total_fifos = g->total_fifos;
  m.uptime_ticks = g->uptime_ticks;
  m.sched_switches = g->sched_switches;
  m.cpu_cycles_lo = g->cpu_cycles_lo;
  m.idle_cycles_lo = g->idle_cycles_lo;
  m.systick_count = g->systick_count;
  m.systick_last_cyc = g->systick_last_cyc;
  m.systick_max_cyc = g->systick_max_cyc;
  m.systick_preemptions = g->systick_preemptions;
  m.ipc_takes = g->ipc_takes;
  m.ipc_takes_blocked = g->ipc_takes_blocked;
  m.ipc_gives = g->ipc_gives;
  m.ipc_timeouts = g->ipc_timeouts;
  m.heap_allocs = g->heap_allocs;
  m.heap_frees = g->heap_frees;
  m.heap_oom = g->heap_oom;
  m.heap_peak_bytes = g->heap_peak_bytes;
  m.heap_total_bytes = g->heap_total_bytes;
  uint8_t frame[NAVLINK_MAX_FRAME];
  size_t n = navlink_perf_global_encode(frame, &m, s++, get_device_id(), 1);
  write_channel(g_telemetry_channel, frame, (uint16_t)n);
}

void navlink_tx_perf_task(const perf_task_row_t *row, uint32_t seq) {
  static uint8_t s = 0;
  navlink_perf_task_t m = {0};
  m.seq = seq;
  m.task_id = row->task_id;
  m.priority = row->priority;
  m.state = row->state;
  m.stack_peak = row->stack_peak;
  m.stack_size = row->stack_size;
  m.cycles_lo = row->cycles_lo;
  m.switches_in = row->switches_in;
  m.max_burst_cyc = row->max_burst_cyc;
  uint8_t frame[NAVLINK_MAX_FRAME];
  size_t n = navlink_perf_task_encode(frame, &m, s++, get_device_id(), 1);
  write_channel(g_telemetry_channel, frame, (uint16_t)n);
}

void navlink_tx_perf_fifo(const perf_fifo_row_t *row, uint32_t seq) {
  static uint8_t s = 0;
  navlink_perf_fifo_t m = {0};
  m.seq = seq;
  m.fifo_id = row->fifo_id;
  m.peak = row->peak;
  m.capacity = row->capacity;
  m.drops = row->drops;
  uint8_t frame[NAVLINK_MAX_FRAME];
  size_t n = navlink_perf_fifo_encode(frame, &m, s++, get_device_id(), 1);
  write_channel(g_telemetry_channel, frame, (uint16_t)n);
}

/* --- command responses ---------------------------------------------------- */

void navlink_tx_time_sync_response(const time_sync_payload_t *out) {
  /* v2 TIME_SYNC (msgid 10); the payload is byte-identical to time_sync_payload_t
   * (dialect note), so copy field-by-field into the aligned struct. */
  static uint8_t seq = 0;
  navlink_time_sync_t m = {0};
  m.role = out->role;
  m.seq = out->seq;
  m.t1_gcs_tx = out->t1_gcs_tx;
  m.t2_fc_rx = out->t2_fc_rx;
  m.t3_fc_tx = out->t3_fc_tx;
  m.commanded_offset_ms = out->commanded_offset_ms;
  m.commanded_offset_hi_ms = out->commanded_offset_hi_ms;
  uint8_t frame[NAVLINK_MAX_FRAME];
  size_t n = navlink_time_sync_encode(frame, &m, seq++, get_device_id(), 1);
  write_channel(g_telemetry_channel, frame, (uint16_t)n);
}

void navlink_tx_perf_taskname(uint8_t id, const char *name) {
  /* v2 PERF_TASKNAME (msgid 1038); name is a NUL-padded char[32]. */
  static uint8_t seq = 0;
  navlink_perf_taskname_t m = {0};
  m.task_id = id;
  for (uint8_t k = 0; k < sizeof(m.name) - 1 && name[k]; k++) {
    m.name[k] = name[k];
  }
  uint8_t frame[NAVLINK_MAX_FRAME];
  size_t n = navlink_perf_taskname_encode(frame, &m, seq++, get_device_id(), 1);
  write_channel(g_telemetry_channel, frame, (uint16_t)n);
}
