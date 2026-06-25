/*
 * host_lifecycle.c -- vayu_sitl_start / vayu_sitl_stop / passthrough
 * control. Lives in libvayu_sitl_core.a so both the Navigator host
 * (which links the library) and the standalone vayu_sitl binary
 * (which has its own main()) share the same boot path.
 *
 * Lifecycle limitations (v1):
 *   - vayu_sitl_start() can be called at most ONCE per process. The
 *     firmware's vaios "tasks" are detached pthreads with while(1)
 *     bodies; they have no clean stop path. Calling start a second
 *     time spawns more of them, which double-pushes into the queues.
 *   - vayu_sitl_stop() is best-effort: it tears down the IMU feeder
 *     (which exits cleanly when iface->running goes to 0) and
 *     signals the cond; the other tasks remain alive but quiesce
 *     because no fresh IMU samples flow.
 *   - For a true reset, restart the host process.
 */
#define _GNU_SOURCE
#include "control/control.h"
#include "actuator/actuator.h"
#include "comm/comm.h"
#include "est/est.h"
#include "sensor/sensor.h"
#include "sys/state.h"
#include "task.h"
#include "storage/fs_owner.h"
#include "sys/sys_utils.h"
#include "variables.h"   /* HIGH_FREQ_TIMER_FREQ */
#include "vaios.h"
#include "vayu_tasks.h"

#include "host_clock.h"
#include "host_imu_feeder.h"
#include "host_baro.h"
#include "host_rc_feeder.h"
#include "vsim_iface.h"

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

/* Visible to host_main.c so its SIGINT handler can break the main
 * sleep loop. Defined here because it's where vayu_sitl_stop sets it. */
volatile int g_vayu_sitl_running = 1;

static int started = 0;
static int passthrough_mode = 0;

/* Passthrough mode: ch[2] -> all motors equally, PID bypassed. */
static void passthrough_task(void *arg) {
    (void)arg;
    ibus_data_t rc;
    static ibus_data_t prev_rc;
    fprintf(stderr, "host_lifecycle: passthrough mode active "
                    "(ch3 -> all 4 motors equally; PID bypassed)\n");
    while (g_vayu_sitl_running) {
        if (!rc_queue_control_pop(&rc)) {
            rc = prev_rc;
        } else {
            prev_rc = rc;
        }
        float throttle = 0.0f;
        if (system_state_get() == SYSTEM_STATE_ARMED) {
            int us = rc.channels[2];
            if (us < 1000) us = 1000;
            if (us > 2000) us = 2000;
            throttle = (float)(us - 1000) / 1000.0f;
        }
        motor_outputs_t mo = { throttle, throttle, throttle, throttle };
        motor_set_outputs(mo);
        v_delay(2);
    }
}

void vayu_sitl_set_passthrough(int enabled) {
    passthrough_mode = enabled ? 1 : 0;
}

/* High-frequency timestamp counter. On hardware `_time_stamp_high_freq`
 * (src/utils/utils.c, via increment_high_freq_timer) is bumped by a 10 kHz
 * timer ISR; every outgoing telemetry packet stamps from it.
 *
 * In SITL this is now driven by the IMU feeder off the VIRTUAL sim clock
 * (host_imu_feeder.c bumps it HIGH_FREQ_TIMER_FREQ/SITL_IMU_FEED_HZ ticks per
 * sample) — Phase 1 of docs/plans/sitl-lockstep-sim.md. The old wall-clock
 * hf_timer_thread that lived here is gone: telemetry timestamps and firmware
 * delays now share one sim-time base, so they stay correct at any sim speed
 * (and the wall-clock-wraparound bug it once carried can't recur). */

int vayu_sitl_start(vsim_iface_t *iface) {
    if (started) return -1;
    started = 1;

    /* The IMU feeder owns the virtual clock from here on: firmware task delays
     * block until sim time advances (lockstep), rather than self-advancing as
     * they do in driver-less unit tests. Set before any task is created so no
     * task races the feeder to advance the clock. */
    host_clock_set_driven(1);

    vsim_iface_set_global(iface);

    fprintf(stderr, "vayu_sitl: starting (%s, %s)\n",
            iface ? "in-process iface" : "legacy FIFO",
            passthrough_mode ? "passthrough" : "PID");

    imu_buffer_init();
    rc_buffer_init();
    control_telemetry_buffer_init();

    extern void vayu_log(const char *fmt, ...);
    vayu_log("vayu_sitl: telemetry chain initialized\n");

    {
        extern channel_t g_telemetry_channel;
        extern void uart2_packet_recv_callback(void);
        serial_args_t uart_args = {
            .baud_rate = UART_BAUDRATE,
            .uart = HAL_UART_2,
            .timeout = 100,
        };
        if (get_handler(CHANNEL_TYPE_SERIAL, &g_telemetry_channel, &uart_args,
                        uart2_packet_recv_callback) != NONE) {
            fprintf(stderr, "host_lifecycle: telemetry channel init FAILED\n");
        } else {
            fprintf(stderr, "host_lifecycle: telemetry channel on UART2 registered\n");
        }
    }

    /* Walk the allowed transition path (SYS-SAFE-006): UNINITIALIZED ->
     * INIT (via system_state_init) -> STANDBY. A direct jump to STANDBY is
     * rejected by the transition table. (On hardware boot_task does this.) */
    system_state_init();
    VAYU_DISCARD(system_state_set(SYSTEM_STATE_STANDBY));

    /* COMM-CMD-003: restore any persisted PID tune (0:pid.bin) before the
     * controllers init from it — mirrors src/main.c on real hardware. The host
     * VFS is now disk-backed, so a tune saved via CMD_SET_PID survives a
     * restart. Must precede the controller tasks (they read the store at init). */
    extern void pid_config_init(void);
    pid_config_init();

    if (!passthrough_mode) {
        /* The firmware's REAL attitude estimator (EKF/Mahony per SF_FILTER_USED).
         * Previously SITL omitted this and the host IMU feeder ran a stand-in
         * mahony — which meant SITL never exercised the shipped estimator. It
         * now runs here, fed raw IMU via imu_queue_attitude, exactly as on HW. */
        task_create(attitude_task,              NULL, 1024 * 8, 1);
        /* Vertical estimator (VERT): sibling of attitude_task, consumes its
         * synchronized {q, accel, dt} output + the modelled baro, publishes the
         * fused vertical state (VERTICAL_STATE telemetry). */
        task_create(vertical_estimator_task,    NULL, 1024 * 8, 1);
        task_create(angle_controller_task,      NULL, 1024 * 8, 1);
        task_create(angle_rate_controller_task, NULL, 1024 * 8, 1);
    } else {
        task_create(passthrough_task,           NULL, 1024 * 8, 1);
    }
    task_create(motor_task,                     NULL, 1024 * 8, 1);
    task_create(imu_telemetry_task,             NULL, 1024 * 8, 1);
    task_create(flush_task,                     NULL, 1024 * 4, 0);
    /* GCS->FC command path: consume RX packets (CMD_SET_PID, calibrate,
     * heartbeat) the UART2 reader buffers. Without this, commands are
     * received but never dispatched. */
    task_create(comm_processor_task,            NULL, 1024 * 4, 0);

    host_wall_delay_ms(200);  /* boot settle, runs before the clock advances */
    set_motor_ready(true);

    fprintf(stderr, "host_lifecycle: motor_ready = true, awaiting SwA-up + "
                    "low throttle to arm\n");

#ifndef VAYU_SITL_RTOS
    /* Legacy pthread SITL: free-running feeder threads. The RTOS variant
     * (Phase 4) drives sensors single-threaded from the stepper, so it does not
     * start feeder threads here. */
    host_rc_feeder_start();
    host_imu_feeder_start();
    host_baro_start();
    /* HF timestamp counter is driven by the IMU feeder off the virtual clock;
     * no separate wall-clock timer thread (see the comment above). */
#endif

    return 0;
}

void vayu_sitl_stop(void) {
    if (!started) return;
    vsim_iface_t *iface = vsim_iface_get_global();
    if (iface) {
        pthread_mutex_lock(&iface->lock);
        iface->running = 0;
        pthread_cond_broadcast(&iface->imu_cond);
        pthread_mutex_unlock(&iface->lock);
    }
    g_vayu_sitl_running = 0;
    /* Wake any firmware task blocked in a virtual v_delay so the detached
     * pthreads can observe g_vayu_sitl_running==0 and unwind instead of
     * sleeping forever on a clock that will no longer advance. */
    host_clock_stop();
}
