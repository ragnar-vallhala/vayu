/*
 * Copyright (C) 2026 NAVRobotec Pvt Ltd
 * Author: Ragnar Vallhala
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef VAYU_IMU_BUFFER_H
#define VAYU_IMU_BUFFER_H

#include "comm/perf_packet.h"
#include "est/est.h"
#include "est/vertical_estimator.h"
#include "hub/sample.h"
#include "structure.h" /* spsc_fifo_t */

/* Calibration progress on its way to the GCS. An opaque payload -- the hub
 * neither builds nor reads it -- so it names no device and lives here rather
 * than in the driver that happens to emit it. */
typedef struct {
  uint8_t buffer[20];
  uint8_t size;
} imu_calibration_telemetry_t;
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

/* Offer a driver-owned FIFO to the perf view. The hub cannot enumerate queues
 * that live inside a driver without naming the driver, so the driver offers
 * them instead -- at init, before perf is first sampled. */
#define HUB_PERF_EXTRA_MAX 4
void hub_perf_register(uint8_t id, const spsc_fifo_t *fifo);

/* ---------------------------------------------------------------------------
 * Latest-value topics.
 *
 * A queue is the right shape for the inertial stream, where the estimator must
 * integrate every sample and a dropped one is a hole. These sensors are the
 * other kind: they run at tens of Hz, each reading supersedes the last, and a
 * consumer wants the most recent one rather than all of them. So each topic is
 * a single slot the producer overwrites -- which is exactly the semantics the
 * driver getters had before, now without the consumer naming the driver.
 *
 * A reader tells a fresh reading from a repeat by watching t_cyc advance; a
 * `*_latest` before the first publish returns false rather than zeros, so
 * "nothing yet" and "a reading of zero" stay distinguishable.
 * ------------------------------------------------------------------------- */
void mag_publish(const mag_sample_t *sample);
bool mag_latest(mag_sample_t *out);

void baro_publish(const baro_sample_t *sample);
bool baro_latest(baro_sample_t *out);

void range_publish(const range_sample_t *sample);
bool range_latest(range_sample_t *out);

bool imu_queue_telemetry_push(const imu_sample_t *sample);
bool imu_queue_telemetry_pop(imu_sample_t *out_sample);
bool imu_queue_telemetry_peek(imu_sample_t *out_sample);

bool imu_queue_control_push(const imu_sample_t *sample);
bool imu_queue_control_pop(imu_sample_t *out_sample);
bool imu_queue_control_peek(imu_sample_t *out_sample);

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
bool imu_queue_attitude_push(const imu_sample_t *sample);
bool imu_queue_attitude_pop(imu_sample_t *out_sample);
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

/* Board mounting tilt (degrees, roll/pitch), published by whoever owns the
 * calibration store and subtracted from the estimator's euler output so a
 * tilted FC still reports the TRUE frame level. 0,0 until a board-level
 * calibration (imu_id 4) has been run.
 *
 * It lives here for the same reason the samples do: it is an SI quantity the
 * estimator needs, and reaching into the IMU driver to fetch it put
 * driver/bmx160.h -- and with it the whole register map -- inside est/
 * (fault line F1). Scalars, not a ring: last write wins, and the only writes
 * are a boot load and the end of a calibration run. */
void hub_set_board_trim(float roll_deg, float pitch_deg);
void hub_get_board_trim(float *roll_deg, float *pitch_deg);

#endif // VAYU_IMU_BUFFER_H
