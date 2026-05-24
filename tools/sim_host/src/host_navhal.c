/*
 * host_navhal.c -- host implementations of the NavHAL functions the vayu
 * controller and ESC layers call.
 *
 * Coverage:
 *   PWM: hal_pwm_init / start / stop / set_duty_cycle -> writes
 *        "motor_idx duty\n" to /tmp/vayu_pwm.fifo. The FIFO is opened
 *        O_RDWR + O_NONBLOCK so writes never block whether a consumer
 *        has connected yet or not (kernel pipe buffer absorbs).
 *   GPIO: no-ops.
 *   Clock: SYSCLK = 84 MHz.
 *   Cycle counter: CLOCK_MONOTONIC nanoseconds scaled to 84 MHz.
 *
 * UART, timer, I2C, DMA, CRC, interrupts: not implemented here. The host
 * SITL doesn't run the RTOS-side iBus, DMA, or telemetry paths - those
 * are replaced by host feeder threads (see host_rc_feeder.c /
 * host_imu_feeder.c) that write directly to the vayu queues.
 */
#define _GNU_SOURCE
#include "navhal.h"

#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#define VAYU_PWM_FIFO_PATH "/tmp/vayu_pwm.fifo"
#define PWM_NUM_TIMER_CHANNELS 4

/* The four ESCs all share TIM1; we identify the motor by channel 1..4. */
typedef struct {
    uint32_t frequency_hz;
    float    last_duty;
    int      started;
} host_pwm_state_t;

static host_pwm_state_t pwm_state[PWM_NUM_TIMER_CHANNELS + 1]; /* 1..4 indexed */
static int  pwm_fifo_fd = -1;
static char pwm_fifo_open_failed = 0;

static void pwm_fifo_ensure_open(void) {
    if (pwm_fifo_fd >= 0 || pwm_fifo_open_failed) return;
    struct stat st;
    if (stat(VAYU_PWM_FIFO_PATH, &st) != 0) {
        /* Best-effort create. mkfifo lives in libc on Linux. */
        if (mkfifo(VAYU_PWM_FIFO_PATH, 0666) != 0) {
            /* If somebody else just created it, fall through. */
        }
    }
    pwm_fifo_fd = open(VAYU_PWM_FIFO_PATH, O_RDWR | O_NONBLOCK);
    if (pwm_fifo_fd < 0) {
        fprintf(stderr, "host_navhal: open %s failed\n", VAYU_PWM_FIFO_PATH);
        pwm_fifo_open_failed = 1;
    } else {
        fprintf(stderr, "host_navhal: PWM FIFO %s open (fd=%d)\n",
                VAYU_PWM_FIFO_PATH, pwm_fifo_fd);
    }
}

/* ---- PWM HAL ---------------------------------------------------------- */
hal_status_t hal_pwm_init(hal_pwm_handle_t *pwm, uint32_t frequency,
                          float duty_cycle) {
    if (!pwm || pwm->channel < 1 || pwm->channel > 4 || frequency == 0)
        return HAL_ERR_INVALID_ARG;
    pwm_fifo_ensure_open();
    pwm_state[pwm->channel].frequency_hz = frequency;
    pwm_state[pwm->channel].last_duty    = duty_cycle;
    pwm_state[pwm->channel].started      = 0;
    return HAL_OK;
}

hal_status_t hal_pwm_start(hal_pwm_handle_t *pwm) {
    if (!pwm || pwm->channel < 1 || pwm->channel > 4)
        return HAL_ERR_INVALID_ARG;
    pwm_state[pwm->channel].started = 1;
    return HAL_OK;
}

hal_status_t hal_pwm_stop(hal_pwm_handle_t *pwm) {
    if (!pwm || pwm->channel < 1 || pwm->channel > 4)
        return HAL_ERR_INVALID_ARG;
    pwm_state[pwm->channel].started = 0;
    return HAL_OK;
}

hal_status_t hal_pwm_set_duty_cycle(hal_pwm_handle_t *pwm, float duty_cycle) {
    if (!pwm || pwm->channel < 1 || pwm->channel > 4)
        return HAL_ERR_INVALID_ARG;
    if (duty_cycle < 0.0f) duty_cycle = 0.0f;
    if (duty_cycle > 1.0f) duty_cycle = 1.0f;
    pwm_state[pwm->channel].last_duty = duty_cycle;

    if (pwm_fifo_fd < 0) pwm_fifo_ensure_open();
    if (pwm_fifo_fd < 0) return HAL_OK; /* degrade: skip */

    /* Channel 1..4 -> motor index 0..3 (vayu_pwm_to_gz.py expects 0..3). */
    int motor_idx = (int)pwm->channel - 1;
    char buf[40];
    int n = snprintf(buf, sizeof(buf), "%d %.6f\n", motor_idx, duty_cycle);
    if (n > 0) {
        ssize_t w = write(pwm_fifo_fd, buf, (size_t)n);
        (void)w;
    }
    return HAL_OK;
}

hal_status_t hal_pwm_set_frequency(hal_pwm_handle_t *pwm, uint32_t frequency) {
    if (!pwm || pwm->channel < 1 || pwm->channel > 4)
        return HAL_ERR_INVALID_ARG;
    pwm_state[pwm->channel].frequency_hz = frequency;
    return HAL_OK;
}

/* ---- GPIO HAL: no-op on host ----------------------------------------- */
hal_status_t hal_gpio_init(hal_gpio_pin_t pin, const hal_gpio_config_t *cfg) {
    (void)pin; (void)cfg; return HAL_OK;
}
hal_status_t hal_gpio_set_mode(hal_gpio_pin_t pin, hal_gpio_mode_t mode,
                               hal_gpio_pull_t pull) {
    (void)pin; (void)mode; (void)pull; return HAL_OK;
}
hal_gpio_mode_t hal_gpio_get_mode(hal_gpio_pin_t pin) {
    (void)pin; return HAL_GPIO_MODE_INPUT;
}
hal_status_t hal_gpio_enable_clock(hal_gpio_pin_t pin) {
    (void)pin; return HAL_OK;
}
hal_status_t hal_gpio_set_alternate_function(hal_gpio_pin_t pin, hal_gpio_af_t af) {
    (void)pin; (void)af; return HAL_OK;
}
hal_status_t hal_gpio_set_output_type(hal_gpio_pin_t pin, hal_gpio_output_type_t t) {
    (void)pin; (void)t; return HAL_OK;
}
hal_status_t hal_gpio_set_output_speed(hal_gpio_pin_t pin, hal_gpio_output_speed_t s) {
    (void)pin; (void)s; return HAL_OK;
}

/* ---- Clock HAL: stub fixed sysclk ------------------------------------ */
uint32_t hal_clock_get_sysclk(void)   { return 84000000u; }
uint32_t hal_clock_get_ahbclk(void)   { return 84000000u; }
uint32_t hal_clock_get_apb1clk(void)  { return 42000000u; }
uint32_t hal_clock_get_apb2clk(void)  { return 84000000u; }

hal_status_t hal_clock_init(const hal_clock_config_t *cfg,
                            const hal_pll_config_t *pll) {
    (void)cfg; (void)pll; return HAL_OK;
}

/* ---- Cycle counter (DWT-style) on host -------------------------------
 *
 * The firmware's sensor_fusion get_dt() reads this on the first sample
 * with `last_dwt = 0`, then computes (now - last_dwt) / 84e6 to get dt
 * in seconds. On real hardware, "now" is small at first boot. On host,
 * CLOCK_MONOTONIC is the kernel monotonic counter, which can be huge
 * (uptime in seconds). Taking that mod 2^32 produces a near-random
 * first dt that triggers a ~100 deg attitude integration spike and
 * trips the controller's MAX_ANGLE_CUTOFF failsafe.
 *
 * Anchor the host cycle counter at process start so the firmware's
 * "first sample has last_dwt = 0" assumption matches a small "now". */
static struct timespec hal_cyc_origin;
static int hal_cyc_origin_set = 0;
static pthread_once_t hal_cyc_once = PTHREAD_ONCE_INIT;

static void hal_cyc_anchor(void) {
    clock_gettime(CLOCK_MONOTONIC, &hal_cyc_origin);
    hal_cyc_origin_set = 1;
}

hal_status_t hal_cycle_counter_init(void) {
    pthread_once(&hal_cyc_once, hal_cyc_anchor);
    return HAL_OK;
}

uint32_t hal_cycle_counter_get(void) {
    pthread_once(&hal_cyc_once, hal_cyc_anchor);
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    uint64_t sec  = (uint64_t)(t.tv_sec  - hal_cyc_origin.tv_sec);
    int64_t  nsec = (int64_t)t.tv_nsec - (int64_t)hal_cyc_origin.tv_nsec;
    if (nsec < 0) { sec--; nsec += 1000000000L; }
    /* 84 MHz virtual clock = 84 cycles/usec. */
    uint64_t cycles = sec * 84000000ULL + (uint64_t)nsec * 84ULL / 1000ULL;
    return (uint32_t)(cycles & 0xFFFFFFFFu);
}
