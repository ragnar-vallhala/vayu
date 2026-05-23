/*
 * bmx160_sim.c -- IMU injection task for VAYU_SIM builds.
 *
 * Phase 5 Light of the Renode + Gazebo SITL plan. Replaces the real
 * bmx160_initiate_read task: every millisecond, copy the current
 * sample out of the Renode `imu_inject_mock` Python peripheral
 * (mapped at IMU_INJECT_MOCK_BASE) and push it through vayu's normal
 * imu_buffer / imu_queue_control / imu_queue_telemetry APIs. The
 * control loop, sensor fusion, and telemetry then see Gazebo's IMU
 * via Phase 4's bridge (which writes to the peripheral's host FIFO
 * /tmp/vayu_imu.fifo).
 *
 * The full I2C + BMX160 + BMM150 mock is Phase 5 Heavy follow-up.
 */
#ifdef VAYU_SIM

#include "sensor/bmx160.h"
#include "sensor/imu_buffer.h"
#include "utils.h"
#include "vaios.h"
#include "vayu_tasks.h"
#include <stdint.h>
#include <string.h>

/* Must match tools/sim_renode/nucleo_f401re.repl. */
#define IMU_INJECT_MOCK_BASE 0x40002400U

void bmx160_initiate_read(void *args) {
  (void)args;
  bmx160_all_reading_t sample;
  while (1) {
    /* The Python peripheral at IMU_INJECT_MOCK_BASE returns the latest
     * 76-byte bmx160_all_converted_reading_t-shaped sample one
     * uint32 at a time. Read it word by word into our buffer. */
    volatile uint32_t *src = (volatile uint32_t *)IMU_INJECT_MOCK_BASE;
    uint32_t *dst = (uint32_t *)&sample;
    const uint32_t n_words = sizeof(bmx160_all_reading_t) / 4;
    for (uint32_t i = 0; i < n_words; i++) {
      dst[i] = src[i];
    }

    imu_buffer_push(&sample);
    imu_queue_control_push(&sample);
    imu_queue_telemetry_push(&sample);

    /* Match the real BMX160 ~1 kHz cadence. */
    v_delay(1);
  }
}

#endif /* VAYU_SIM */
