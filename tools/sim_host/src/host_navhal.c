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

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#define VAYU_PWM_FIFO_PATH    "/tmp/vayu_pwm.fifo"
#define VAYU_UART2_LOG_PATH   "/tmp/vayu_uart2.log"
#define VAYU_UART2_PTY_PATH   "/tmp/vayu_uart2_pty"  /* slave-path advertisement */
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

/* ---- UART HAL ---------------------------------------------------------
 *
 * UART2 carries vayu's binary telemetry (DroneProtocol packets). The SITL
 * pipes those bytes through a pty so the GCS can connect to a /dev/pts/N
 * "serial port" the same way it talks to real hardware, AND tees a copy
 * to a raw log file for post-sim analysis.
 *
 * UART6 is the iBus RC input on hardware; the firmware's rc_task uses
 * the sim_rc bypass when sim_rc_enabled is set, so its hal_uart_read_char
 * here is a no-op.
 */
static pthread_mutex_t uart2_mu = PTHREAD_MUTEX_INITIALIZER;
static int uart2_pty_master_fd = -1;
static int uart2_log_fd = -1;
static char uart2_slave_path[256];

static void uart2_ensure_open(void) {
    pthread_mutex_lock(&uart2_mu);
    if (uart2_pty_master_fd < 0) {
        int fd = posix_openpt(O_RDWR | O_NOCTTY);
        if (fd >= 0 && grantpt(fd) == 0 && unlockpt(fd) == 0) {
            const char *p = ptsname(fd);
            if (p) {
                strncpy(uart2_slave_path, p, sizeof(uart2_slave_path) - 1);
                uart2_slave_path[sizeof(uart2_slave_path) - 1] = '\0';
                uart2_pty_master_fd = fd;
                /* Advertise the slave path. The GCS user can either
                 * read /tmp/vayu_uart2_pty or look at stderr. */
                FILE *adv = fopen(VAYU_UART2_PTY_PATH, "w");
                if (adv) { fprintf(adv, "%s\n", uart2_slave_path); fclose(adv); }
                fprintf(stderr, "host_navhal: UART2 pty open, slave=%s\n",
                        uart2_slave_path);
                fprintf(stderr, "host_navhal: connect the GCS to %s "
                                "(or read %s)\n",
                        uart2_slave_path, VAYU_UART2_PTY_PATH);
            } else {
                close(fd);
            }
        } else {
            if (fd >= 0) close(fd);
            fprintf(stderr, "host_navhal: UART2 pty open failed: %s\n",
                    strerror(errno));
        }
    }
    if (uart2_log_fd < 0) {
        uart2_log_fd = open(VAYU_UART2_LOG_PATH,
                            O_WRONLY | O_CREAT | O_TRUNC, 0666);
        if (uart2_log_fd >= 0) {
            fprintf(stderr, "host_navhal: UART2 raw log -> %s\n",
                    VAYU_UART2_LOG_PATH);
        } else {
            fprintf(stderr, "host_navhal: open %s failed: %s\n",
                    VAYU_UART2_LOG_PATH, strerror(errno));
        }
    }
    pthread_mutex_unlock(&uart2_mu);
}

static void uart2_write_bytes(const void *buf, size_t n) {
    if (n == 0) return;
    uart2_ensure_open();
    if (uart2_pty_master_fd >= 0) {
        ssize_t w = write(uart2_pty_master_fd, buf, n);
        (void)w;  /* drop if the consumer hasn't connected */
    }
    if (uart2_log_fd >= 0) {
        ssize_t w = write(uart2_log_fd, buf, n);
        (void)w;
    }
}

hal_status_t hal_uart_init(hal_uart_t uart, const hal_uart_config_t *cfg) {
    (void)cfg;
    if (uart == HAL_UART_2) uart2_ensure_open();
    return HAL_OK;
}

hal_status_t hal_uart_init_dma_rx(hal_uart_t uart, uint8_t *buf, uint16_t len) {
    (void)uart; (void)buf; (void)len;
    return HAL_OK;
}

hal_status_t hal_uart_write_char(hal_uart_t uart, char c) {
    if (uart == HAL_UART_2) uart2_write_bytes(&c, 1);
    return HAL_OK;
}

hal_status_t hal_uart_write_dma(hal_uart_t uart, const uint8_t *buf,
                                 uint16_t len) {
    if (uart == HAL_UART_2) uart2_write_bytes(buf, len);
    /* The firmware's channel.c uses a busy flag that only clears on
     * the DMA-TX-complete IRQ. Our hal_uart_write_dma is synchronous,
     * so we manually invoke the registered TX-complete handler to
     * unstick the next flush. dma_tx_complete_callback is the
     * symbol defined in extern/vaios/kernel/utils.c that channel.c
     * registers; calling it directly bypasses the IRQ plumbing. */
    extern void dma_tx_complete_callback(void);
    dma_tx_complete_callback();
    return HAL_OK;
}

char hal_uart_read_char(hal_uart_t uart) {
    (void)uart;
    return 0;
}

hal_status_t hal_uart_enable_interrupt(hal_uart_t uart, uint8_t rx, uint8_t tx) {
    (void)uart; (void)rx; (void)tx; return HAL_OK;
}

/* ---- Interrupt HAL: all stubs --------------------------------------- */
hal_status_t hal_interrupt_attach_callback(hal_irq_t irq, void (*cb)(void)) {
    (void)irq; (void)cb; return HAL_OK;
}
hal_status_t hal_interrupt_detach_callback(hal_irq_t irq) {
    (void)irq; return HAL_OK;
}
hal_status_t hal_interrupt_enable(hal_irq_t irq)  { (void)irq; return HAL_OK; }
hal_status_t hal_interrupt_disable(hal_irq_t irq) { (void)irq; return HAL_OK; }

/* Compat shims used by both vaios and vayu's channel.c. Both pairs
 * exist on real NavHAL because the API has been renamed over time;
 * we provide both so neither set of callers needs adjustment. */
uint32_t hal_disable_global_interrupts(void) { return 0; }
void     hal_enable_global_interrupts(uint32_t state) { (void)state; }
uint32_t hal_interrupt_disable_global(void) { return 0; }
void     hal_interrupt_enable_global(uint32_t state) { (void)state; }

/* ---- CRC HAL: STM32-compatible CRC32 -----------------------------------
 * The firmware computes packet checksums via utils_try_compute_crc32,
 * which calls hal_crc_init + hal_crc_compute. On real hardware that
 * lands in the STM32's CRC peripheral, which uses polynomial 0x04C11DB7
 * MSB-first, init 0xFFFFFFFF, no input/output reflection, no final XOR.
 * The GCS's software_crc.cpp implements the same algorithm. To make the
 * SITL's packets pass the GCS's checksum check (it gates ALL packet
 * dispatch on it), the SITL has to use the same variant - NOT the
 * reflected IEEE 802.3 / zlib CRC32 we shipped first. */

hal_status_t hal_crc_init(const hal_crc_config_t *cfg) {
    (void)cfg;
    return HAL_OK;
}

uint32_t hal_crc_compute(const uint8_t *data, uint32_t length) {
    uint32_t crc = 0xFFFFFFFFu;
    for (uint32_t i = 0; i < length; ++i) {
        crc ^= ((uint32_t)data[i]) << 24;
        for (int b = 0; b < 8; ++b) {
            if (crc & 0x80000000u) crc = (crc << 1) ^ 0x04C11DB7u;
            else                   crc = (crc << 1);
        }
    }
    return crc;
}

/* ---- calibration_task stub ---------------------------------------------
 * On hardware, calibration_task lives in src/sensor/bmx160.c (which we
 * don't compile - it's I2C-driver heavy). The host SITL doesn't expose
 * a calibration flow, so provide a do-nothing task body so comm_processor.c
 * can reference it. */
void calibration_task(void *args) { (void)args; }

/* ---- Timer HAL: stubs (heartbeat uses it via task delays, not real timers) */
hal_status_t hal_timer_init_freq(hal_timer_t t, uint32_t freq_hz) {
    (void)t; (void)freq_hz; return HAL_OK;
}
hal_status_t hal_timer_attach_callback(hal_timer_t t, void (*cb)(void)) {
    (void)t; (void)cb; return HAL_OK;
}
hal_status_t hal_timer_enable_interrupt(hal_timer_t t) {
    (void)t; return HAL_OK;
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
