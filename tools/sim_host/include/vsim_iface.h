/*
 * vsim_iface.h -- shared interface between the vayu firmware (built
 * into libvayu_sitl_core.a) and an external host that simulates the
 * drone body. Replaces the legacy /tmp/vayu_pwm.fifo + /tmp/vayu_imu.fifo
 * pipes when the firmware is linked in-process (Navigator GCS).
 *
 * Two channels, one direction each:
 *   - motor_duty[4]  : firmware writes, host reads. Pushed by
 *                      hal_pwm_set_duty_cycle() in host_navhal.c.
 *                      Sub-millisecond cadence.
 *   - imu_frame[76]  : host writes, firmware reads. The 76-byte
 *                      bmx160_all_converted_reading_t struct already
 *                      assumed by host_imu_feeder.c. Cadence ~200 Hz.
 *
 * All access is serialized by `lock`. The mutex is held while reading /
 * writing the small shared state, then released before any compute -
 * uncontended pthread_mutex_lock is sub-100 ns on Linux, so even at
 * the highest realistic rates the lock cost is in the noise.
 *
 * Lifecycle:
 *   1. Host allocates a vsim_iface_t (zero-initialize), calls
 *      vsim_iface_init().
 *   2. Host calls vayu_sitl_start(iface) - firmware spawns its vaios
 *      threads, host_navhal grabs the iface pointer for PWM writes,
 *      host_imu_feeder grabs it for IMU reads.
 *   3. Host pushes IMU samples (vsim_iface_post_imu) and pulls latest
 *      motor duties (vsim_iface_get_motor_duty) on its own thread.
 *   4. Host calls vayu_sitl_stop() to tear down. iface can be reused.
 *   5. Host calls vsim_iface_destroy() before freeing.
 *
 * Backward compat: if vayu_sitl_start(NULL) is called (standalone
 * binary mode), host_navhal falls back to /tmp/vayu_pwm.fifo and
 * host_imu_feeder falls back to /tmp/vayu_imu.fifo. The standalone
 * vayu_sitl binary still works that way for offline testing.
 */
#ifndef VAYU_VSIM_IFACE_H
#define VAYU_VSIM_IFACE_H

#include <pthread.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VSIM_IMU_FRAME_BYTES 76

typedef struct vsim_iface_s {
    pthread_mutex_t lock;
    pthread_cond_t  imu_cond;

    /* Firmware -> host. Index 0..3 = M1..M4. Updated by every
     * hal_pwm_set_duty_cycle call. Already ESC-band-stripped in the
     * 0..1 linear range, matching the bridge contract today. */
    float motor_duty[4];

    /* Host -> firmware. The latest IMU sample. The host increments
     * imu_seq each time it overwrites the frame and broadcasts
     * imu_cond; the firmware's feeder thread waits on imu_cond until
     * imu_seq != imu_consumer_seq, then copies and advances its own
     * counter. Samples that arrive faster than the consumer drains
     * are dropped (latest wins) - typical IMU rates (200 Hz) are
     * comfortably below the consumer's processing rate, so drops
     * shouldn't happen in steady state. */
    uint8_t imu_frame[VSIM_IMU_FRAME_BYTES];
    uint64_t imu_seq;            /* incremented by host */
    uint64_t imu_consumer_seq;   /* advanced by firmware */

    /* Set to 0 when shutting down; firmware threads see this and
     * exit their loops. */
    int running;

    /* UART2 byte callback (firmware -> host). Invoked from inside
     * host_navhal's hal_uart_write_dma / write_char on the firmware
     * thread that produced the bytes. host receives the same byte
     * stream that the standalone binary's pty would emit (DroneProtocol
     * packets, vayu_log() text, telemetry, etc.). Host is responsible
     * for marshalling onto its own event loop if needed - we do not
     * hold any iface lock across this callback. NULL = no consumer
     * (bytes are dropped on the host side). */
    void (*on_uart2_bytes)(void *user, const uint8_t *data, size_t n);
    void *on_uart2_bytes_user;
} vsim_iface_t;

/* Initialize the iface members. The caller still owns the struct
 * (stack or heap) - this just sets up the mutex/cond and zeros the
 * state. */
void vsim_iface_init(vsim_iface_t *iface);

/* Tear down the mutex/cond. Caller frees the struct itself. */
void vsim_iface_destroy(vsim_iface_t *iface);

/* Host posts a new IMU sample. `frame` is a 76-byte
 * bmx160_all_converted_reading_t (little-endian, NED frame, gyro in
 * deg/s, see host_imu_feeder.c for the exact layout). */
void vsim_iface_post_imu(vsim_iface_t *iface, const uint8_t *frame);

/* Host pulls the latest motor duties. Returns the four 0..1 floats. */
void vsim_iface_get_motor_duty(const vsim_iface_t *iface, float out_duty[4]);

/* Set / clear the UART2 byte callback. Safe to call before
 * vayu_sitl_start so the firmware's first telemetry tick already
 * routes to the host. */
void vsim_iface_set_uart2_callback(
    vsim_iface_t *iface,
    void (*cb)(void *user, const uint8_t *data, size_t n),
    void *user);

/* ----- firmware-side (called from inside libvayu_sitl_core.a) ----- */

/* Set or clear the global iface pointer. host_navhal.c and
 * host_imu_feeder.c look up this pointer at runtime and route IO
 * through it when non-NULL, falling back to the FIFOs when NULL. */
void vsim_iface_set_global(vsim_iface_t *iface);
vsim_iface_t *vsim_iface_get_global(void);

/* ----- vayu_sitl lifecycle (the public API of libvayu_sitl_core.a) ----- */

/* Boot the firmware. If iface is non-NULL it's adopted as the
 * in-process channel; if NULL the firmware uses the legacy FIFOs.
 * Returns 0 on success, -1 if the firmware is already running. */
int vayu_sitl_start(vsim_iface_t *iface);

/* Tell all firmware threads to stop, join them, and tear down the
 * vaios kernel state. Safe to call when not running. */
void vayu_sitl_stop(void);

/* Optional: enable passthrough mode (RC throttle -> motors equally,
 * PID bypassed). Must be called before vayu_sitl_start. */
void vayu_sitl_set_passthrough(int enabled);

#ifdef __cplusplus
}
#endif

#endif  /* VAYU_VSIM_IFACE_H */
