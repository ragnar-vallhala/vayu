#include "vayu_tasks.h"
#include "actuator/actuator.h"
#include "comm/channel.h"
#include "comm/comm_types.h"
#include "comm/ibus.h"
#include "comm/navlink_tx.h"
#include "comm/rc_buffer.h"
#include "logger/logger.h"
#include "control/control.h"
#include "control/flight_mode.h"
#include "control/sysid.h"
#include "est/est.h"
#include "sensor/sensor.h"
#include "sys/state.h"
#include "sys/sys_utils.h"
#include "utils.h"
#include "sys/math_utils.h"
#include "vaios.h"
#include "vaios_app_config.h"
#include "variables.h"
#include "vfs.h"
#include <stdint.h>

/* Owned here; the TX seam (navlink_tx.c) and RX router both extern it. */
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
  static est_perf_telemetry_t e_data;
  static vertical_state_t vert_data;
  static imu_calibration_telemetry_t imu_calibration_telemetry;

  while (1) {
    /* Slew the disciplined clock toward the GCS-commanded offset (~166 Hz,
     * independent of how often a sync arrives). docs/telemetry/time_sync.md */
    time_sync_discipline_tick();

    if (imu_queue_telemetry_pop(&samples)) {
      current_floats[0] = (float)samples.converted.acc[0];
      current_floats[1] = (float)samples.converted.acc[1];
      current_floats[2] = (float)samples.converted.acc[2];
      current_floats[3] = (float)samples.converted.gyr[0];
      current_floats[4] = (float)samples.converted.gyr[1];
      current_floats[5] = (float)samples.converted.gyr[2];
      current_floats[6] = (float)samples.converted.mag[0];
      current_floats[7] = (float)samples.converted.mag[1];
      current_floats[8] = (float)samples.converted.mag[2];
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
    bool send_baro = (packet_counter % 15 == 0);       // 10 Hz
    bool send_vert = (packet_counter % 15 == 0);        // 10 Hz (fused vertical)

    /* While dumping the system-ID capture, hand the bridge's ~150 pkt/s budget
     * to the dump: suppress the heavy periodic telemetry so the chunks aren't
     * crowded out and dropped (this is a deliberate post-run, bench-only op). */
    if (sysid_dump_active()) {
      send_full = send_comp = send_att = send_rc = send_motor = send_pid_err =
          send_baro = send_vert = false;
    }
    /* Gather domain data + hand it to the TX seam; this task is codec-blind
     * (all framing lives in navlink_tx.c). */
    if (send_log) {
      /* LOG-TXT-002: drain the text-log queue to the LOG channel. */
      static char log_buf[VAYU_LOG_QUEUE_SIZE];
      uint8_t len = (uint8_t)mpmc_pop_bulk(&vayu_log_queue, log_buf, sizeof(log_buf));
      if (len > 0) {
        navlink_tx_log(log_buf, len);
      }
    }
    (void)send_heartbeat; /* heartbeat now rides the send_status gate below */
    if (send_status) {
      /* v2 HEARTBEAT carries nav_state (folds in the former SYS_STATE origin);
       * emitted at the status cadence so the vehicle-state pill stays responsive
       * while still satisfying COMM-TEL-002 (>= 1 Hz). @implements COMM-TEL-002 */
      navlink_tx_heartbeat();
      /* stabilise/acro + RC/GCS source so the GCS can reflect the mode. */
      navlink_tx_flight_mode((uint8_t)flight_mode_get(),
                             (uint8_t)flight_mode_get_source());
      /* Health counters (COMM-CH-002, SNS-BUF-002, LOG-SD-002). The legacy IMU
       * averaging ring was removed; imu_drop stays 0 to preserve the layout. */
      navlink_tx_health(channel_tx_overflow_count(), 0u,
                        logger_wrap_count_total());
    }
    if (send_pid_err && control_telemetry_queue_pop(&c_data)) {
      navlink_tx_pid_error(&c_data);
    }

    if (send_full) {
      navlink_tx_imu_full(current_floats);
      v_memcpy(previous_floats, current_floats, sizeof(current_floats));
      first_packet = false;
    } else if (send_comp && !first_packet) {
      uint16_t delta_payload[10];
      for (int i = 0; i < 10; i++) {
        delta_payload[i] =
            float32_to_float16(current_floats[i] - previous_floats[i]);
      }
      navlink_tx_imu_compressed(delta_payload);
      v_memcpy(previous_floats, current_floats, sizeof(current_floats));
    }

    if (send_att && attitude_queue_telemetry_pop(&att)) {
      navlink_tx_attitude(&att);
    }

    if (send_rc && rc_queue_telemetry_pop(&rc_data)) {
      navlink_tx_rc_channels(&rc_data);
    }

    if (send_motor && motor_telemetry_queue_pop(&m_data)) {
      navlink_tx_motor(&m_data);
    }
    if (imu_queue_calibration_telemetry_pop(&imu_calibration_telemetry)) {
      navlink_tx_calibration(imu_calibration_telemetry.buffer,
                             imu_calibration_telemetry.size);
    }
    /* System-ID: flush captured half-buffers to SD during a run (all the vfs I/O
     * lives here, off the 1 kHz control loop). Cheap no-op when idle. */
    sysid_flush_poll();
    /* System-ID capture dump: read the SD file back as SYSID_SAMPLE chunks, a few
     * per cycle so write_channel isn't flooded. The host re-sends CMD_SYSID_DUMP
     * to refill any chunks the lossy link dropped (idempotent re-dump). */
    if (sysid_dump_active()) {
      /* Pace to ~80 chunks/s (1 chunk per 2 cycles at 166 Hz): the ESP bridge
       * only sustains ~150 pkts/s shared with the rest of telemetry, so blasting
       * the whole file at once just gets ~80% dropped. Paced under capacity, a
       * single dump pass arrives intact (the host still re-requests for any
       * residual drops, deduping by start_index). */
      uint16_t total = (uint16_t)sysid_capture_count();
      uint16_t hz = (uint16_t)sysid_capture_hz();
      uint8_t axis = (uint8_t)sysid_capture_axis();
      for (int b = 0; b < 2 && sysid_dump_active(); b++) {
        uint16_t start = 0;
        int16_t sp[10], gyro[10];
        int n = sysid_dump_next(&start, sp, gyro, 10);
        if (n <= 0)
          break;
        navlink_tx_sysid_sample(start, total, hz, axis, (uint8_t)n, sp, gyro);
      }
    }
    /* Estimator cost probe (~1 Hz): peak/mean per-update cost + cadence. */
    if (est_perf_queue_pop(&e_data)) {
      navlink_tx_est_perf(&e_data);
    }
    /* Barometer (~10 Hz): BME280 pressure/temp/humidity + derived altitude.
     * bme280_read_all returns the last published sample; only emit once the
     * sensor has produced one (skips cleanly when absent/mis-wired). */
    if (send_baro) {
      bme280_reading_t baro;
      if (bme280_read_all(&baro) == HAL_OK) {
        navlink_tx_baro(baro.pressure_pa, baro.temperature_c, baro.humidity_rh,
                        baro.altitude_m);
      }
    }
    /* Fused vertical estimate (~10 Hz): VERT task output, with raw baro alt
     * alongside for a fused-vs-raw chart. Latest-wins ring; skips cleanly until
     * the VERT task has published. */
    if (send_vert && vertical_state_queue_pop(&vert_data)) {
      navlink_tx_vertical_state(&vert_data);
    }
    packet_counter++;
    v_delay(6); // ~166 Hz
  }
}
