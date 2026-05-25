/*
 * host_imu_feeder.c -- read Gazebo IMU samples from /tmp/vayu_imu.fifo,
 * push them into vayu's imu_queue, run mahony to derive the attitude,
 * push that into vayu's attitude_queue.
 *
 * Wire format: tools/sim_gazebo/gz_imu_to_vayu.py writes the
 * bmx160_all_converted_reading_t struct (76 B, little-endian) on every
 * Gazebo IMU sample. We read that whole frame and use the calibrated
 * acc / gyr / mag triplets.
 *
 * The mahony filter holds its quaternion state inside the attitude_t
 * we pass in - the same pattern bmx160.c uses on real hardware. We
 * keep one static attitude here, seeded to identity.
 *
 * If the FIFO doesn't exist yet we create it; if the producer hasn't
 * connected, open(O_RDONLY) blocks until it does. That's fine: this
 * runs in its own pthread and doesn't gate the rest of the SITL.
 */
#define _GNU_SOURCE
#include "host_imu_feeder.h"
#include "vsim_iface.h"

#include "maths/sensor_fusion.h"
#include "sensor/bmx160.h"
#include "sensor/imu_buffer.h"

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "task.h"   /* v_delay */
#include "utils.h"  /* v_get_ticks */

#define IMU_FIFO_PATH "/tmp/vayu_imu.fifo"

/* Sanity: gz_imu_to_vayu.py packs exactly this many bytes per sample.
 * If bmx160_all_converted_reading_t ever grows, the bridge will need
 * to grow with it. */
#define EXPECTED_FRAME_BYTES 76

static void ensure_fifo(void) {
    struct stat st;
    if (stat(IMU_FIFO_PATH, &st) != 0) {
        if (mkfifo(IMU_FIFO_PATH, 0666) != 0 && errno != EEXIST) {
            fprintf(stderr, "host_imu_feeder: mkfifo %s failed: %s\n",
                    IMU_FIFO_PATH, strerror(errno));
        }
    }
}

static int read_full(int fd, void *buf, size_t n) {
    size_t got = 0;
    while (got < n) {
        ssize_t r = read(fd, (char *)buf + got, n - got);
        if (r > 0) { got += (size_t)r; continue; }
        if (r == 0) return 0;          /* EOF (writer closed) */
        if (errno == EINTR) continue;
        return -1;
    }
    return 1;
}

/* Block until the host posts a new IMU sample. Returns 1 with the
 * sample copied into `sample`, or 0 if the iface signaled shutdown. */
static int wait_iface_sample(vsim_iface_t *iface,
                             bmx160_all_reading_t *sample,
                             uint64_t *consumer_seq) {
    pthread_mutex_lock(&iface->lock);
    while (iface->running && iface->imu_seq == *consumer_seq) {
        pthread_cond_wait(&iface->imu_cond, &iface->lock);
    }
    int alive = iface->running;
    if (alive) {
        memcpy(&sample->converted, iface->imu_frame, EXPECTED_FRAME_BYTES);
        *consumer_seq = iface->imu_seq;
    }
    pthread_mutex_unlock(&iface->lock);
    return alive;
}

static void *imu_feeder_thread(void *arg) {
    (void)arg;

    /* Compile-time guarantee that the on-wire layout matches our struct. */
    _Static_assert(sizeof(bmx160_all_converted_reading_t) == EXPECTED_FRAME_BYTES,
                   "bmx160_all_converted_reading_t must be 76 B on this build");

    bmx160_all_reading_t sample;
    attitude_t att = { .roll = 0, .pitch = 0, .yaw = 0,
                       .q = { 1.0f, 0.0f, 0.0f, 0.0f } };

    uint32_t frames = 0;
    uint32_t last_log_t = 0;

    vsim_iface_t *iface = vsim_iface_get_global();
    int fd = -1;
    uint64_t consumer_seq = 0;

    if (iface) {
        fprintf(stderr,
                "host_imu_feeder: in-process iface mode (no FIFO)\n");
    } else {
        ensure_fifo();
        fprintf(stderr, "host_imu_feeder: waiting for producer on %s\n",
                IMU_FIFO_PATH);
        fd = open(IMU_FIFO_PATH, O_RDONLY);
        if (fd < 0) {
            fprintf(stderr, "host_imu_feeder: open failed: %s\n", strerror(errno));
            return NULL;
        }
        fprintf(stderr, "host_imu_feeder: producer connected, draining frames\n");
    }

    while (1) {
        if (iface) {
            if (!wait_iface_sample(iface, &sample, &consumer_seq)) {
                /* Shutdown signaled. */
                break;
            }
        } else {
        int rc = read_full(fd, &sample.converted, EXPECTED_FRAME_BYTES);
        if (rc == 0) {
            /* Producer disconnected -- reopen and keep going. */
            fprintf(stderr, "host_imu_feeder: producer closed, reopening\n");
            close(fd);
            fd = open(IMU_FIFO_PATH, O_RDONLY);
            if (fd < 0) {
                fprintf(stderr, "host_imu_feeder: reopen failed\n");
                return NULL;
            }
            continue;
        }
        if (rc < 0) {
            fprintf(stderr, "host_imu_feeder: read failed: %s\n", strerror(errno));
            break;
        }
        }

        /* The IMU sample feeds the angle_rate_controller directly. */
        imu_queue_control_push(&sample);
        imu_queue_telemetry_push(&sample);

        /* Mahony updates `att` in place (its quaternion is the filter state).
         * dt is taken from hal_cycle_counter_get(), which our host shim
         * derives from CLOCK_MONOTONIC. */
        m_mahony_filter(sample.converted.acc[0], sample.converted.acc[1],
                        sample.converted.acc[2],
                        sample.converted.gyr[0], sample.converted.gyr[1],
                        sample.converted.gyr[2],
                        sample.converted.mag[0], sample.converted.mag[1],
                        sample.converted.mag[2],
                        &att);

        attitude_queue_control_push(&att);
        attitude_queue_telemetry_push(&att);

        frames++;
        uint32_t now = v_get_ticks();
        if (now - last_log_t >= 1000) {
            fprintf(stderr,
                    "host_imu_feeder: %u frames, last roll=%.2f pitch=%.2f yaw=%.2f\n",
                    frames, (double)att.roll, (double)att.pitch, (double)att.yaw);
            last_log_t = now;
        }
    }

    if (fd >= 0) close(fd);
    return NULL;
}

void host_imu_feeder_start(void) {
    pthread_t th;
    pthread_create(&th, NULL, imu_feeder_thread, NULL);
    pthread_detach(th);
}
