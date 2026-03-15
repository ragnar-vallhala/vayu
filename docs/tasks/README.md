# Vayu Running Tasks

This document captures all the Vaios tasks currently running in the Vayu flight controller firmware. The tasks are all initialized in `src/main.c` via the `init_tasks()` function.

---

## Tasks Index

Below is the list of active tasks within the system. Click on a task to view its detailed configuration, loop rate, and purpose.

- [Motor Task](motor_task.md)
- [RC iBus Task](rc_ibus_task.md)
- [BMX160 IMU Task](bmx160_initiate_read.md)
- [Telemetry Task](imu_telemetry_task.md)
- [Physical Heartbeat](physical_heartbeat.md)
- [Communications Processor](comm_processor_task.md)
- [Channel Flush Task](flush_task.md)
