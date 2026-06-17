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
#include "logger/logger.h"
#include "sys/sys_utils.h"
#include "variables.h"   /* HIGH_FREQ_TIMER_FREQ */
#include "vaios.h"
#include "vayu_tasks.h"

#include "host_imu_feeder.h"
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

/* High-frequency tick thread. On hardware, `_time_stamp_high_freq`
 * (private to src/utils/utils.c, accessed via increment_high_freq_timer)
 * is bumped by a 10 kHz timer ISR. In the host build there's no ISR,
 * so without this thread the counter stays at 0 forever and every
 * outgoing telemetry packet stamps timestamp=0 - which is what made the
 * 2026-05-25 log analyser hit a "duration 0.0s" divide-by-zero.
 *
 * We wake at ~1 ms and use CLOCK_MONOTONIC to compute how many ticks
 * *should* have elapsed since vayu_sitl_start, then bump the counter
 * to catch up. Ticks come in bursts of ~10 per wake instead of one
 * every 100 us, but get_timestamp_unix() divides by HIGH_FREQ_TIMER_FREQ
 * / 1000 = 10 before returning - i.e. it converts to ms - so the burstiness
 * is invisible at the wire layer. */
static void *hf_timer_thread(void *arg) {
    (void)arg;
    struct timespec origin;
    clock_gettime(CLOCK_MONOTONIC, &origin);
    uint64_t emitted = 0;
    while (g_vayu_sitl_running) {
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        /* Signed subtract for tv_nsec - if `now.tv_nsec < origin.tv_nsec`
         * the unsigned cast (previous version) wrapped around to ~1.8e19
         * and the inner loop bumped the counter that many times,
         * producing a 213-million-ms ARMED state transition in the
         * 2026-05-25 23:54 log. Borrow from tv_sec the standard way. */
        int64_t sec  = (int64_t)now.tv_sec  - (int64_t)origin.tv_sec;
        int64_t nsec = (int64_t)now.tv_nsec - (int64_t)origin.tv_nsec;
        if (nsec < 0) { sec--; nsec += 1000000000LL; }
        if (sec < 0)  { sec = 0; nsec = 0; }   /* clock jumped backwards */
        uint64_t elapsed_us = (uint64_t)sec * 1000000ULL +
                              (uint64_t)nsec / 1000ULL;
        /* 10 kHz -> 100 us per tick. */
        uint64_t target = elapsed_us / (1000000ULL / HIGH_FREQ_TIMER_FREQ);
        while (emitted < target) {
            increment_high_freq_timer();
            emitted++;
        }
        struct timespec rest = { 0, 1000000 };  /* 1 ms */
        nanosleep(&rest, NULL);
    }
    return NULL;
}

static void host_start_hf_timer(void) {
    pthread_t th;
    pthread_create(&th, NULL, hf_timer_thread, NULL);
    pthread_detach(th);
}

int vayu_sitl_start(vsim_iface_t *iface) {
    if (started) return -1;
    started = 1;

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

    v_delay(200);
    set_motor_ready(true);

    fprintf(stderr, "host_lifecycle: motor_ready = true, awaiting SwA-up + "
                    "low throttle to arm\n");

    host_rc_feeder_start();
    host_imu_feeder_start();
    host_start_hf_timer();

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
}
