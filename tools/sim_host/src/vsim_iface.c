/*
 * vsim_iface.c -- implementation of the shared firmware<->host
 * interface, plus the global pointer the firmware-side IO shims
 * (host_navhal.c, host_imu_feeder.c) look up at runtime.
 */
#define _GNU_SOURCE
#include "vsim_iface.h"

#include <pthread.h>
#include <string.h>

static vsim_iface_t *g_iface = NULL;

void vsim_iface_init(vsim_iface_t *iface) {
    if (!iface) return;
    memset(iface, 0, sizeof(*iface));
    pthread_mutex_init(&iface->lock, NULL);
    pthread_cond_init(&iface->imu_cond, NULL);
    iface->running = 1;
}

void vsim_iface_destroy(vsim_iface_t *iface) {
    if (!iface) return;
    pthread_cond_destroy(&iface->imu_cond);
    pthread_mutex_destroy(&iface->lock);
}

void vsim_iface_post_imu(vsim_iface_t *iface, const uint8_t *frame) {
    if (!iface || !frame) return;
    pthread_mutex_lock(&iface->lock);
    memcpy(iface->imu_frame, frame, VSIM_IMU_FRAME_BYTES);
    iface->imu_seq++;
    pthread_cond_broadcast(&iface->imu_cond);
    pthread_mutex_unlock(&iface->lock);
}

void vsim_iface_get_motor_duty(const vsim_iface_t *iface, float out_duty[4]) {
    if (!iface || !out_duty) return;
    /* The lock field is mutable through const-cast because there's no
     * other clean way to express "logically read-only but I still need
     * to take the mutex". The mutex is the source of truth. */
    vsim_iface_t *m = (vsim_iface_t *)iface;
    pthread_mutex_lock(&m->lock);
    for (int i = 0; i < 4; ++i) out_duty[i] = iface->motor_duty[i];
    pthread_mutex_unlock(&m->lock);
}

void vsim_iface_set_uart2_callback(
    vsim_iface_t *iface,
    void (*cb)(void *user, const uint8_t *data, size_t n),
    void *user) {
    if (!iface) return;
    pthread_mutex_lock(&iface->lock);
    iface->on_uart2_bytes = cb;
    iface->on_uart2_bytes_user = user;
    pthread_mutex_unlock(&iface->lock);
}

void vsim_iface_set_global(vsim_iface_t *iface) { g_iface = iface; }
vsim_iface_t *vsim_iface_get_global(void) { return g_iface; }
