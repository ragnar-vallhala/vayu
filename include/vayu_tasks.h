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

#endif // !VAYU_TASKS_H
