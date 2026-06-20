#ifndef VAYU_TASKS_H
#define VAYU_TASKS_H

#include "comm/comm_types.h"

void boot_task(void *args);
void heartbeat_task(void *args);
void comm_processor_task(void *args);
/* Dispatch one received packet (heartbeat / command) to its handler.
 * Exposed for SITL so the GCS -> FC command path is unit-testable. */
void comm_processor_dispatch(const packet_t *pkt);
void flush_task(void *args);
void imu_telemetry_task(void *args);
void rc_ibus_task(void *args);
void motor_task(void *args);
void calibration_task(void *args);
/* Attitude estimation (fusion) — consumes timestamped IMU samples, publishes
 * timestamped attitude. Split out of the IMU driver. */
void attitude_task(void *args);
/* Vertical estimator (VERT) — sibling of the attitude task. Drains the
 * synchronized {q, accel, dt} input, fuses baro altitude, publishes the
 * fused {altitude, climb_rate, vertical_accel} for control / IN_AIR / telemetry. */
void vertical_estimator_task(void *args);
/* Periodic kernel/observability reporter (FC -> GCS, PACKET_TYPE_PERF_STATS). */
void perf_telemetry_task(void *args);

#endif // !VAYU_TASKS_H
