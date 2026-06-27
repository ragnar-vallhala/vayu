/**
 * @file attitude_task.c
 * @brief Dedicated attitude-estimation task.
 *
 * Split out of the IMU driver (bmx160_process_data) so the driver only
 * acquires, converts, timestamps and fans out samples. This task consumes
 * timestamped IMU samples, runs the configured fusion filter using the sample's
 * acquisition interval as dt (the source-sample timestamp delta, not a DWT read
 * at run time), stamps the resulting attitude with that same timestamp, and
 * publishes it to the control and telemetry queues. The angle loop then derives
 * its own dt from attitude-timestamp deltas — see vayu_dt_from_cycles().
 *
 * Estimator decimation: the EKF covariance update is ~30x the cost of the
 * Mahony/complementary filters, so running it on every 2 kHz IMU sample
 * saturates the F4 and starves the low-priority telemetry tasks. The EKF is
 * therefore decimated to the control-loop rate (INNER_LOOP_FREQ_HZ, 1 kHz):
 * the latest sample of the window is fed to the estimator (no averaging — the
 * IMU driver already low-passes accel/gyro, so a boxcar average just added group
 * delay) and dt is accumulated so predict integrates the full elapsed interval.
 * The rate loop
 * reads the freshest attitude each cycle, so one estimate per control cycle
 * loses nothing. Cheaper filters keep running on every sample (decim = 1).
 */
#include "est/est.h"
#include "sensor/bmx160.h"
#include "sensor/imu_buffer.h"
#include "vaios.h"
#include "vaios_app_config.h"
#include "variables.h"
#include "vayu_tasks.h"
#include <stdbool.h>

/* Cap the blocking wait so the estimator safety step keeps ticking even if the
 * IMU stalls; the loop is otherwise driven by sample arrival (event-driven). */
#define ATTITUDE_MAX_PERIOD_MS 50u

/* Estimator update rate. The EKF measures ~0.7 ms/update on-target, so running
 * it at the 1 kHz inner-loop rate burns ~70% CPU. Decimate it instead to the
 * angle-loop rate (OUTER_LOOP_FREQ_HZ, 250 Hz) — that loop is the *consumer* of
 * attitude, so nothing downstream is starved, and CPU drops ~4x. Cheap filters
 * still run on every IMU sample. */
#if (SF_FILTER_USED == SF_EKF) || (SF_FILTER_USED == SF_EKF_ACCEL_BIAS)
#define ATTITUDE_EST_RATE_HZ OUTER_LOOP_FREQ_HZ
#else
#define ATTITUDE_EST_RATE_HZ IMU_SAMPLE_FREQ_HZ
#endif
#define ATTITUDE_DECIM (IMU_SAMPLE_FREQ_HZ / ATTITUDE_EST_RATE_HZ)
#if ATTITUDE_DECIM < 1
#undef ATTITUDE_DECIM
#define ATTITUDE_DECIM 1
#endif

/* Cycle probe: report the real estimator cost (peak/mean µs over a ~1 s window)
 * as SYSTEM_ORIGIN_EST_PERF telemetry (plotted on the GCS control-loop page) so
 * the per-update budget can be checked against the loop period on-target. Set
 * to 0 to compile it out. */
#ifndef ATTITUDE_CYCLE_PROBE
#define ATTITUDE_CYCLE_PROBE 1
#endif

/**
 * Attitude-estimation task: selects the single active fusion filter at runtime
 * (SF_FILTER_USED) and publishes each estimate to the telemetry and control
 * SPSC queues every step.
 *
 * @implements EST-MAH-003, EST-COV-001
 */
void attitude_task(void *args) {
  (void)args;
  static bmx160_all_reading_t sample;
  attitude_t ori = {0};
  ori.q.w = 1.0f; /* identity quaternion */
  uint32_t prev_cyc = 0;
  bool have_prev = false;

  /* Decimation state: count samples and sum dt across the window so predict
   * integrates the full elapsed interval. We feed the estimator the LATEST
   * sample (no averaging) to minimise group delay; the IMU driver already
   * low-passes accel/gyro, so the boxcar average mostly just added latency. */
  float dt_sum = 0.0f;
  int acc_n = 0;

#if ATTITUDE_CYCLE_PROBE
  uint32_t probe_peak = 0, probe_acc = 0, probe_cnt = 0;
  uint32_t cyc_per_us = hal_cycle_counter_cycles_per_us();
  if (cyc_per_us == 0)
    cyc_per_us = 1;
#endif

#if (SF_FILTER_USED == SF_EKF) || (SF_FILTER_USED == SF_EKF_ACCEL_BIAS)
  /* Build the EKF covariance/state before the first sample; the 9-state
   * variant (SF_EKF_ACCEL_BIAS) additionally estimates accelerometer bias. */
  ekf_init(SF_FILTER_USED == SF_EKF_ACCEL_BIAS);
#endif

  while (1) {
    /* Block until a fresh IMU sample is handed over (or the cap elapses). */
    if (!imu_queue_attitude_wait(MS_TO_TICKS(ATTITUDE_MAX_PERIOD_MS)))
      continue;
    if (!imu_queue_attitude_pop(&sample))
      continue;

    /* dt = time since the previous sample, from acquisition stamps. */
    uint32_t now_cyc = sample.converted.timestamp;
    float dt = have_prev ? vayu_dt_from_cycles(now_cyc, prev_cyc)
                         : (1.0f / (float)IMU_SAMPLE_FREQ_HZ);
    prev_cyc = now_cyc;
    have_prev = true;

    dt_sum += dt;
    if (++acc_n < ATTITUDE_DECIM)
      continue; /* accumulate dt until the estimator step is due */

    /* Estimator step due: feed the LATEST sample directly (lowest latency).
     * dt_sum spans the decimation window so predict integrates the full
     * elapsed interval. */
    float ax = sample.converted.acc[0];
    float ay = sample.converted.acc[1];
    float az = sample.converted.acc[2];
    float gx = sample.converted.gyr[0];
    float gy = sample.converted.gyr[1];
    float gz = sample.converted.gyr[2];
    float mx = sample.converted.mag_fusion[0];
    float my = sample.converted.mag_fusion[1];
    float mz = sample.converted.mag_fusion[2];
    float step_dt = dt_sum;
    dt_sum = 0.0f;
    acc_n = 0;

#if ATTITUDE_CYCLE_PROBE
    uint32_t c0 = hal_cycle_counter_get();
#endif

    if (SF_FILTER_USED == SF_MAHONY) {
      m_mahony_filter(ax, ay, az, gx, gy, gz, mx, my, mz, step_dt, &ori);
    } else if (SF_FILTER_USED == SF_EKF ||
               SF_FILTER_USED == SF_EKF_ACCEL_BIAS) {
      m_ekf_filter(ax, ay, az, gx, gy, gz, mx, my, mz, step_dt, &ori);
    } else {
      m_complementary_filter(ax, ay, az, gx, gy, gz, mx, my, mz, step_dt, &ori);
    }

#if ATTITUDE_CYCLE_PROBE
    uint32_t dc = hal_cycle_counter_get() - c0; /* wrap-safe 32-bit delta */
    if (dc > probe_peak)
      probe_peak = dc;
    probe_acc += dc;
    if (++probe_cnt >= (uint32_t)ATTITUDE_EST_RATE_HZ) { /* ~1 s of updates */
      est_perf_telemetry_t perf = {
          .peak_us = (float)probe_peak / (float)cyc_per_us,
          .mean_us =
              ((float)probe_acc / (float)probe_cnt) / (float)cyc_per_us,
          .decim = (float)ATTITUDE_DECIM,
          .rate_hz = (float)ATTITUDE_EST_RATE_HZ,
      };
      est_perf_queue_push(&perf); /* drained by the telemetry task -> GCS */
      probe_peak = 0;
      probe_acc = 0;
      probe_cnt = 0;
    }
#endif

    ori.degraded = estimator_is_degraded();
    ori.timestamp = now_cyc; /* propagate source-sample stamp for loop dt */

    attitude_queue_telemetry_push(&ori);
    attitude_queue_control_push(&ori);

    /* Hand the same (attitude, specific-force, dt) triple to the vertical
     * estimator task so it integrates a self-consistent sample without racing
     * the angle loop on the attitude control queue. step_dt spans the
     * decimation window; ax/ay/az are this step's body specific force. */
    vert_input_t vin = {.q = ori.q,
                        .a_body = {ax, ay, az},
                        .dt = step_dt,
                        .timestamp = now_cyc};
    vert_input_queue_push(&vin);

    /* SYS-SAFE-003: if degraded persists in a flight-relevant state, request
     * FAILSAFE, at one-IMU-period latency. */
    estimator_safety_step();
  }
}
