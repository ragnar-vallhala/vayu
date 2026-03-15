#ifndef VAYU_TASKS_H
#define VAYU_TASKS_H

void boot_task(void *args);
void heartbeat_task(void *args);
void comm_processor_task(void *args);
void flush_task(void *args);
void imu_telemetry_task(void *args);
void rc_ibus_task(void *args);
void motor_task(void *args);
void calibration_task(void *args);

#endif // !VAYU_TASKS_H
