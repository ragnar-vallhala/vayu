#ifndef VAYU_IMU_BUFFER_H
#define VAYU_IMU_BUFFER_H

#include "comm/perf_packet.h"
#include "est/est.h"
#include "est/vertical_estimator.h"
#include "sensor/bmx160.h"
#include <stdbool.h>
#include <stdint.h>

/* Ring depths. Usable slots = SIZE - 1 (spsc_init spends one on the empty
 * marker, and one more re-aligning a buffer whose element size isn't a power
 * of two, which none of these are).
 *
 * Control rings are drained with `while (pop())` on every consumer wake, so
 * depth only has to absorb scheduler jitter: 4 usable slots is 4 ms at the
 * 1 kHz sample rate, and 2x the worst peak measured over a full flight
 * (2 of 9, zero drops -- docs/journal/log-analysis/20260907-231254-*). */
#define IMU_BUFFER_SIZE 5

/* Telemetry rings are mailboxes: a 1 kHz producer, a consumer that pops ONE
 * element per gate tick (20-50 ms). Depth buys no throughput here -- it only
 * makes the sample that IS read older, by SIZE-1 samples, every time. Two
 * usable slots is the minimum that keeps producer and consumer decoupled. */
#define IMU_TELEMETRY_BUFFER_SIZE 3

void imu_buffer_init(void);

/* Fill perf wire rows for this module's SPSC fifos (peak/capacity/drops).
 * Returns the number of rows written (<= max). */
int imu_buffer_perf_fifos(perf_fifo_row_t *rows, int max);

bool imu_queue_telemetry_push(const bmx160_all_reading_t *sample);
bool imu_queue_telemetry_pop(bmx160_all_reading_t *out_sample);
bool imu_queue_telemetry_peek(bmx160_all_reading_t *out_sample);

bool imu_queue_control_push(const bmx160_all_reading_t *sample);
bool imu_queue_control_pop(bmx160_all_reading_t *out_sample);
bool imu_queue_control_peek(bmx160_all_reading_t *out_sample);

/**
 * @brief Block until a fresh IMU control sample is pushed, or the
 *        timeout elapses.
 *
 * Backs the event-driven rate loop (CTRL-RATE-101): the rate task waits
 * here instead of clock-polling with v_delay(). The underlying SPSC ring
 * stays lock-free (R8.6) — this is a pure edge notification, not a lock
 * around the data. The producer (imu_queue_control_push) signals it from
 * task context after each write.
 *
 * @param ticks_to_wait  max ticks to block (use MS_TO_TICKS()).
 * @return true if signalled (a sample is likely available), false on
 *         timeout — the caller falls back to the last sample so the
 *         control / failsafe path keeps running if the IMU stalls.
 *
 * @implements CTRL-RATE-101
 */
bool imu_queue_control_wait(uint32_t ticks_to_wait);

/* IMU -> attitude task queue (estimator input). push from the IMU driver,
 * pop/wait from the attitude task. */
bool imu_queue_attitude_push(const bmx160_all_reading_t *sample);
bool imu_queue_attitude_pop(bmx160_all_reading_t *out_sample);
bool imu_queue_attitude_wait(uint32_t ticks_to_wait);

bool attitude_queue_telemetry_push(const attitude_t *attitude);
bool attitude_queue_telemetry_pop(attitude_t *out_attitude);
bool attitude_queue_telemetry_peek(attitude_t *out_attitude);

bool attitude_queue_control_push(const attitude_t *attitude);
bool attitude_queue_control_pop(attitude_t *out_attitude);
bool attitude_queue_control_peek(attitude_t *out_attitude);
/* Block until the next attitude control sample is pushed (or timeout). Lets the
 * outer/angle loop pace itself off the inner-loop sample rate. */
bool attitude_queue_control_wait(uint32_t ticks_to_wait);

/* Estimator cost-probe telemetry (attitude task -> telemetry task). One push
 * per probe window (~1 Hz) when ATTITUDE_CYCLE_PROBE is enabled; the telemetry
 * task drains it onto the SYSTEM_ORIGIN_EST_PERF wire packet. */
bool est_perf_queue_push(const est_perf_telemetry_t *perf);
bool est_perf_queue_pop(est_perf_telemetry_t *out_perf);

/* Fused vertical state (VERT task -> control loop / IN_AIR detector / telemetry).
 * OVERWRITE ring: producer is the vertical estimator task, consumers peek/pop
 * the latest fused {altitude, climb_rate, vertical_accel}. */
bool vertical_state_queue_push(const vertical_state_t *vs);
bool vertical_state_queue_pop(vertical_state_t *out_vs);
bool vertical_state_queue_peek(vertical_state_t *out_vs);

/* Synchronized estimator input for the VERT task. The attitude task publishes
 * {q, body specific force, dt} from the SAME sample it ran the EKF on, so the
 * vertical estimator integrates a self-consistent (attitude, accel, dt) triple
 * without racing the angle loop on the attitude control queue. Event-driven via
 * a wake semaphore, same pattern as imu_queue_attitude_*. */
typedef struct {
  quaternion_t q;     /* body->world attitude at this sample. */
  float a_body[3];    /* body specific force (m/s^2), as the IMU reports it. */
  float dt;           /* integration interval (s) for this step. */
  uint32_t timestamp; /* DWT cycle stamp of the source IMU sample. */
} vert_input_t;

bool vert_input_queue_push(const vert_input_t *in);
bool vert_input_queue_pop(vert_input_t *out_in);
bool vert_input_queue_wait(uint32_t ticks_to_wait);

bool imu_queue_calibration_telemetry_push(
    const imu_calibration_telemetry_t *sample);
bool imu_queue_calibration_telemetry_pop(
    imu_calibration_telemetry_t *out_sample);
bool imu_queue_calibration_telemetry_peek(
    imu_calibration_telemetry_t *out_sample);

bool imu_queue_calibration_push(const bmx160_all_reading_t *sample);
bool imu_queue_calibration_pop(bmx160_all_reading_t *out_sample);
bool imu_queue_calibration_peek(bmx160_all_reading_t *out_sample);

#endif // VAYU_IMU_BUFFER_H
