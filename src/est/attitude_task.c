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

void attitude_task(void *args) {
  (void)args;
  static bmx160_all_reading_t sample;
  attitude_t ori = {0};
  ori.q.w = 1.0f; /* identity quaternion */
  uint32_t prev_cyc = 0;
  bool have_prev = false;

  while (1) {
    /* Block until a fresh IMU sample is handed over (or the cap elapses). */
    if (!imu_queue_attitude_wait(MS_TO_TICKS(ATTITUDE_MAX_PERIOD_MS)))
      continue;
    if (!imu_queue_attitude_pop(&sample))
      continue;

    /* dt = time since the previous sample, from acquisition stamps. */
    uint32_t now_cyc = sample.converted.timestamp;
    float dt = have_prev ? vayu_dt_from_cycles(now_cyc, prev_cyc) : 1e-3f;
    prev_cyc = now_cyc;
    have_prev = true;

    if (SF_FILTER_USED == SF_MAHONY) {
      m_mahony_filter(sample.converted.acc[0], sample.converted.acc[1],
                      sample.converted.acc[2], sample.converted.gyr[0],
                      sample.converted.gyr[1], sample.converted.gyr[2],
                      sample.converted.mag_fusion[0],
                      sample.converted.mag_fusion[1],
                      sample.converted.mag_fusion[2], dt, &ori);
    } else {
      m_complementary_filter(
          sample.converted.acc[0], sample.converted.acc[1],
          sample.converted.acc[2], sample.converted.gyr[0],
          sample.converted.gyr[1], sample.converted.gyr[2],
          sample.converted.mag_fusion[0], sample.converted.mag_fusion[1],
          sample.converted.mag_fusion[2], dt, &ori);
    }

    ori.degraded = estimator_is_degraded();
    ori.timestamp = now_cyc; /* propagate source-sample stamp for loop dt */

    attitude_queue_telemetry_push(&ori);
    attitude_queue_control_push(&ori);

    /* SYS-SAFE-003: if degraded persists in a flight-relevant state, request
     * FAILSAFE. Same one-IMU-period latency as before the task split. */
    estimator_safety_step();
  }
}
