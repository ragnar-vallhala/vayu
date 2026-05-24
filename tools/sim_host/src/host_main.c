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
#include "host_rc_feeder.h"

#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

static volatile int running = 1;

static void on_sigint(int sig) { (void)sig; running = 0; }

/* Passthrough mode: pop the latest RC frame and write the normalized
 * throttle stick to all four motors equally. Bypasses the cascaded PID,
 * which is useful in two scenarios:
 *
 *   1) Validating Gazebo physics + bridges + the SITL plumbing end-to-end
 *      without the controller being in the loop.
 *   2) Avoiding rate-PID integral windup while the drone is stuck on the
 *      ground - the integrator builds against the (immovable) rate error
 *      and makes one motor saturate while another idles, costing ~50% of
 *      available thrust and preventing liftoff.
 *
 * Enable with VAYU_SITL_PASSTHROUGH=1. */
static void passthrough_task(void *arg) {
    (void)arg;
    ibus_data_t rc;
    static ibus_data_t prev_rc;
    fprintf(stderr, "host_main: passthrough mode active "
                    "(ch3 -> all 4 motors equally; PID bypassed)\n");
    while (running) {
        if (!rc_queue_control_pop(&rc)) {
            rc = prev_rc;
        } else {
            prev_rc = rc;
        }
        float throttle = 0.0f;
        if (system_state_get() == SYSTEM_STATE_ARMED) {
            /* Map RC throttle channel (ch[2]) us -> 0..1. */
            int us = rc.channels[2];
            if (us < 1000) us = 1000;
            if (us > 2000) us = 2000;
            throttle = (float)(us - 1000) / 1000.0f;
        }
        motor_outputs_t mo = { throttle, throttle, throttle, throttle };
        motor_set_outputs(mo);
        v_delay(2);   /* 500 Hz, same cadence as angle_rate_controller_task */
    }
}

int main(int argc, char **argv) {
    (void)argc; (void)argv;
    signal(SIGINT, on_sigint);

    int passthrough = 0;
    const char *pt_env = getenv("VAYU_SITL_PASSTHROUGH");
    if (pt_env && pt_env[0] && pt_env[0] != '0') passthrough = 1;

    fprintf(stderr, "vayu_sitl: host SITL up%s\n",
            passthrough ? " (passthrough mode)" : "");

    /* Initialize firmware buffer subsystems. */
    imu_buffer_init();
    rc_buffer_init();
    control_telemetry_buffer_init();

    /* System state: jump straight to STANDBY so rc_task's state machine
     * can transition to ARMED on the next SwA-up frame. */
    system_state_set(SYSTEM_STATE_STANDBY);

    /* Spawn the controller chain, or skip it under passthrough. motor_task
     * runs in both modes - it's the one that calls esc_set_throttle and
     * writes the PWM FIFO. */
    if (!passthrough) {
        task_create(angle_controller_task,      NULL, 1024 * 8, 1);
        task_create(angle_rate_controller_task, NULL, 1024 * 8, 1);
    } else {
        task_create(passthrough_task,           NULL, 1024 * 8, 1);
    }
    task_create(motor_task,                     NULL, 1024 * 8, 1);

    /* Let motor_init finish, then enable motor outputs. */
    v_delay(200);
    set_motor_ready(true);

    fprintf(stderr, "host_main: motor_ready = true, awaiting SwA-up + low "
                    "throttle to arm\n");

    /* RC from sim_bridge MCU on /dev/ttyUSB0 (falls back to synthetic
     * hover if the port isn't present). IMU + attitude from Gazebo. */
    host_rc_feeder_start();
    host_imu_feeder_start();

    while (running) {
        v_delay(1000);
        fprintf(stderr, "host_main: alive @ t=%u ms state=0x%x\n",
                v_get_ticks(), (unsigned)system_state_get());
    }

    fprintf(stderr, "vayu_sitl: shutting down\n");
    return EXIT_SUCCESS;
}

