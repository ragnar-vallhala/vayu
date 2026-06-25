#ifndef VAYU_SIM_HOST_IMU_FEEDER_H
#define VAYU_SIM_HOST_IMU_FEEDER_H

/* Spawn the IMU-feeder pthread.
 * Reads bmx160_all_converted_reading_t frames from /tmp/vayu_imu.fifo
 * (gz_imu_to_vayu.py writes them), pushes them into imu_queue_*, runs
 * the mahony filter and pushes the resulting attitude into
 * attitude_queue_*. */
void host_imu_feeder_start(void);

/* Single-shot pump for the RTOS cooperative stepper (Phase 4). open() returns
 * the IMU FIFO fd (<0 on failure); pump() blocks for one framed IMU sample and
 * injects it into the firmware queues, returning 1 on success / 0 on EOF. */
int host_imu_feeder_open(void);
int host_imu_feeder_pump(void);

#endif
