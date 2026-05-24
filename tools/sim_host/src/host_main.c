/*
 * host_main.c -- entry point for vayu's native host SITL binary.
 *
 * Initializes the firmware's queue subsystems, spawns the controller +
 * motor tasks (mapped to pthreads via host_vaios.c), then runs synthetic
 * RC / IMU / attitude feeders so the chain spins. PWM output lands in
 * /tmp/vayu_pwm.fifo via host_navhal.c, ready for vayu_pwm_to_gz.py.
 *
 * Real Gazebo IMU + sim_bridge UART RC come next (see TODO at bottom).
 */
#define _GNU_SOURCE
#include "control/angle_controller.h"
#include "control/angle_rate_controller.h"
#include "actuator/motor.h"
#include "comm/ibus.h"
#include "comm/rc_buffer.h"
#include "maths/control_buffer.h"
#include "maths/sensor_fusion.h"
#include "sensor/imu_buffer.h"
#include "sensor/bmx160.h"
#include "sys/state.h"
#include "task.h"
#include "vaios.h"

#include "host_imu_feeder.h"

#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

static volatile int running = 1;

static void on_sigint(int sig) { (void)sig; running = 0; }

/* Synthetic RC feeder: pushes a hover frame at 50 Hz.
 * channels: roll=1500, pitch=1500, throttle=1300, yaw=1500, sw_a=2000(arm) */
static void *rc_feeder(void *arg) {
    (void)arg;
    ibus_data_t rc = {0};
    rc.channels[0] = 1500;   /* roll */
    rc.channels[1] = 1500;   /* pitch */
    rc.channels[2] = 1300;   /* throttle - just above min */
    rc.channels[3] = 1500;   /* yaw */
    rc.channels[4] = 2000;   /* SwA = arm */
    for (int i = 5; i < 14; i++) rc.channels[i] = 1500;
    rc.is_failsafe = false;

    fprintf(stderr, "host_main: RC feeder up (synthetic hover frame @ 50 Hz)\n");
    while (running) {
        rc_queue_control_push(&rc);
        rc_queue_telemetry_push(&rc);
        v_delay(20);
    }
    return NULL;
}

int main(int argc, char **argv) {
    (void)argc; (void)argv;
    signal(SIGINT, on_sigint);

    fprintf(stderr, "vayu_sitl: host SITL up\n");

    /* Initialize firmware buffer subsystems. */
    imu_buffer_init();
    rc_buffer_init();
    control_telemetry_buffer_init();

    /* System state: jump straight to STANDBY so rc_task's state machine
     * can transition to ARMED on the next SwA-up frame. */
    system_state_set(SYSTEM_STATE_STANDBY);

    /* Spawn the controller + motor tasks. task_create maps to pthread. */
    task_create(angle_controller_task,      NULL, 1024 * 8, 1);
    task_create(angle_rate_controller_task, NULL, 1024 * 8, 1);
    task_create(motor_task,                 NULL, 1024 * 8, 1);

    /* Let motor_init finish, then enable motor outputs. */
    v_delay(200);
    set_motor_ready(true);

    /* The vayu rc_task state machine arms the system when SwA is up AND
     * channel-2 (throttle) is below 1100. Our synthetic feeder starts at
     * throttle=1300, so without the rc_task running we have to arm by hand. */
    system_state_set(SYSTEM_STATE_ARMED);
    fprintf(stderr, "host_main: state = ARMED, motor_ready = true\n");

    /* RC stays synthetic for now; IMU + attitude come from Gazebo. */
    pthread_t th_rc;
    pthread_create(&th_rc, NULL, rc_feeder, NULL);
    host_imu_feeder_start();

    while (running) {
        v_delay(1000);
        fprintf(stderr, "host_main: alive @ t=%u ms\n", v_get_ticks());
    }

    fprintf(stderr, "vayu_sitl: shutting down\n");
    return EXIT_SUCCESS;
}

/* TODO: replace the synthetic rc_feeder with a /dev/ttyUSB0 CSV reader
 * fed by tools/sim_bridge/ (task #19). */
