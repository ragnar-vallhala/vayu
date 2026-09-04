#include "comm/comm.h"
#include "control/control.h"
#include "sensor/sensor.h"
#include "storage/fs_owner.h"
#include "navhal.h"
#include "sys/state.h"
#include "task.h"
#include "utils.h"
#include "utils/test_file.h"
#include "sys/timer_callbacks.h"
#include "utils/util.h"
#include "sys/sys_utils.h"
#include "utils/v_fs.h"
#include "vaios.h"
#include "vaios_config_default.h"
#include "variables.h"
#include "vayu_assert.h"
#include "vayu_status.h"
#include "vayu_tasks.h"

#ifdef EKF_SELFTEST
#include "est/ekf_selftest.h"
#endif

// Global state values
uint32_t bmx160_task_id = 0;

#ifdef EKF_SELFTEST
/* On-target EKF self-test: run the shared branch-coverage scenarios and report
 * each check + the total over the telemetry UART (via vayu_log). Built only
 * with -DEKF_SELFTEST; see CMake option of the same name.
 *
 * @noreq build-gated diagnostic: self-test result log callback. */
static void ekf_selftest_log_report(void *ctx, bool pass, const char *name) {
  (void)ctx;
  vayu_log("[EKF-SELFTEST] %s %s", pass ? "ok  " : "FAIL", name);
}
/* @noreq build-gated diagnostic: runs the EKF self-test scenarios at boot. */
static void run_ekf_selftest(void) {
  int fails = ekf_selftest_run(ekf_selftest_log_report, NULL);
  vayu_log("[EKF-SELFTEST] total failures: %d", fails);
}
#endif

/**
 * Configure the system clock: HSE 8 MHz crystal → PLL (M=8 N=336 P=4 Q=7)
 * → 84 MHz SYSCLK on the STM32F401RE.
 *
 * @implements SYS-TIM-004
 */
void clock_setup(void) {
  hal_pll_config_t pll_cfg_hse = {
      .input_src = HAL_CLOCK_SOURCE_HSE, /**< External 8 MHz crystal */
      .pll_m = 8,                        /**< PLLM divider */
      .pll_n = 336,                      /**< PLLN multiplier */
      .pll_p = 4,                        /**< PLLP division factor */
      .pll_q = 7                         /**< PLLQ division factor */
  };
  hal_clock_config_t cfg = {
      .source = HAL_CLOCK_SOURCE_PLL /**< Use PLL as system clock */
  };
  hal_clock_init(&cfg, &pll_cfg_hse);
}

/* @noreq boot plumbing: brings up sensor buffers + drivers and opens the
 * telemetry UART channel. No standalone behavioural requirement. */
void init_sensors(void) {
  imu_buffer_init();
  control_telemetry_buffer_init();
  bmx160_init();
  bme280_init(); /* baro/humidity on the shared I2C1 bus; logs + degrades if absent */
  vl53l0x_init(); /* ToF rangefinder, same shared bus; logs + degrades if absent */
  rc_buffer_init();

  // Initialize global telemetry — USART6 (PC6 TX / PC7 RX) per Vayu PCB wiring.
  serial_args_t uart_args = {
      .baud_rate = UART_BAUDRATE, .uart = HAL_UART_6, .timeout = 100};

  if (get_handler(CHANNEL_TYPE_SERIAL, &g_telemetry_channel, &uart_args,
                  uart2_packet_recv_callback) != NONE) {
    return;
  }
}

/* Stack sizes right-sized to each task's on-target high-water (peak bytes used,
 * from the Kernel Perf view). Each size is peak + the 256 B kernel guard band
 * (TASK_STACK_OVERFLOW_THRESHOLD — the scheduler panics if SP comes within this
 * of the base) + ~150 B buffer for un-observed depth / an FPU exception frame,
 * rounded up to a multiple of 64 (so >=8-byte aligned for AAPCS / v_malloc).
 * Net: every task keeps >=400 B below its peak. Re-check the Kernel Perf view
 * after exercising worst-case paths (arm, calibrate, failsafe) before tightening
 * further. Peak at last measurement noted per line.
 *
 * @noreq boot plumbing: creates the steady-state RTOS task set. */
void init_tasks(void) {
  // Deepest dispatch on the system: the NavLink router runs the xfer + fs_query
  // handlers here, building large aligned message structs on-stack (XFER_DATA
  // ~254 B for data[247]) atop navlink_router_poll's buf[256], and runs the
  // mixer geometry solve (m_mat4_inv, ~0.5 KiB frame) on a SET_GEOMETRY command.
  // 4096: the "peak 1884" was measured WITHOUT an active bulk transfer. An xfer
  // UPLOAD dispatches XFER_DATA on this task — a ~254 B aligned struct + the
  // write-at hop on top of the router's buf[256] — which overflowed the
  // right-sized 2496 and wedged the FC. Restored to the known-good size from
  // feat/centralised-fs-owner (byte-perfect 64 KB round-trips).
  task_create_named(comm_processor_task, NULL, 4096, 0,
                    "comm_processor"); // peak 1884 idle; xfer-upload path deeper
  bmx160_task_id =
      task_create_named(bmx160_initiate_read, NULL, 768, 2,
                        "imu_read"); // peak 332
  // Attitude estimation (fusion), split out of the IMU driver.
  task_create_named(attitude_task, NULL, 1152, 1, "attitude"); // peak 700
  // Vertical estimator (VERT): fuses baro + accel into altitude/climb_rate.
  /* 1024 not 832: the ToF ride-along (range read + quaternion tilt projection)
   * pushed this task's measured peak 428 -> 484 B, leaving 348 B free against a
   * 256 B guard band — under one FP exception frame (132 B) of true headroom.
   * See the rate_ctl note above for what that costs when it runs out. */
  task_create_named(vertical_estimator_task, NULL, 1024, 1,
                    "vertical"); // peak 484 measured (was 428 pre-ToF)
  task_create_named(rc_ibus_task, NULL, 576, 0, "rc_ibus"); // peak 132
  task_create_named(angle_controller_task, NULL, 832, 1,
                    "angle_ctl"); // peak 404, control
  /* 1088 was marginal: a live SWD dump caught rate_ctl 956 B deep with a full
   * FP exception context (EXC_RETURN 0xFFFFFFED, S0-S31 = +132 B) on its stack,
   * inside the 256 B TASK_STACK_OVERFLOW_THRESHOLD guard band -> kernel panic
   * (task.c:304). It only shows when preemption rises (adding the tof_read task
   * was enough to expose it); the depth itself is pre-existing. Measured peak
   * on hardware 2026-09-04: 956 B — at 1088 that left 132 B free, inside the
   * guard band; 1536 leaves 580 B. */
  task_create_named(angle_rate_controller_task, NULL, 1536, 1,
                    "rate_ctl"); // peak 956 measured, control
  task_create_named(motor_task, NULL, 704, 1, "motor"); // peak 284, actuator
  task_create_named(imu_telemetry_task, NULL, 1344, 0,
                    "imu_telemetry"); // peak 908
  task_create_named(bme280_read_task, NULL, 768, 0,
                    "baro_read"); // peak 316, ~20 Hz baro/humidity sampler
  task_create_named(vl53l0x_read_task, NULL, 1024, 0,
                    "tof_read"); // peak 468 measured on hardware; 640 would
                                 // leave 172 B free, inside the 256 B
                                 // TASK_STACK_OVERFLOW_THRESHOLD guard band.
  task_create_named(flush_task, NULL, 640, 0, "flush"); // peak 188
  task_create_named(perf_telemetry_task, NULL, 1216, 0,
                    "perf_telemetry"); // peak 804
  // Centralised FS owner: sole runtime SD/VFS writer (blackbox logger + PID/calib
  // saves). Lowest band (prio 0); blocks on its queue so it only runs when there
  // is work and never preempts control. Queues are created lazily on first run.
  // 2048 (was right-sized to 1152, peak 732 idle): the write-at lane during an
  // xfer upload + the blocking FatFS read during a download run deeper than the
  // idle peak. Restored to the known-good feat/centralised-fs-owner size.
  task_create_named(fs_owner_task, NULL, 2048, 0, "fs_owner"); // peak 732 idle
  // Bulk-transfer (FTP) substrate: runs the xfer SM off the comm + control
  // tasks (prio 0). Blocking SD reads + paced emission live here; the comm-task
  // handlers only touch session state (the C1->C3 invariant). fs_query_tick /
  // xfer_tick do the blocking FatFS dir-walk (FILINFO on-stack) plus a 247 B
  // chunk buffer (RAM budget: firmware/docs/plans/xfer-memory-budget.md).
  // 3072: the download EMIT path runs here — blocking FatFS read + 247 B chunk
  // buffer + XFER_DATA encode per chunk. "peak 404" was measured idle; a real
  // multi-chunk download overflowed the right-sized 832 and froze the FC
  // (heartbeat LED stopped). Restored to the known-good feat/centralised-fs-owner
  // size that did byte-perfect 64 KB downloads.
  task_create_named(xfer_service_task, NULL, 3072, 0, "xfer"); // peak 404 idle
}
/**
 * Bring up the 10 kHz high-frequency timer (TIM5) and register its periodic
 * callbacks (HF tick counter + IMU fast-sample tick).
 *
 * @implements SYS-TIM-005
 */
void init_timer_callbacks(void) {
  timer_callback_init(HIGH_FREQ_TIMER_FREQ);
  if (timer_callback_register(increment_high_freq_timer, 1) != 0) {
    PANIC("Failed to register increment_high_freq_timer");
    return;
  };
  // Pace the IMU accel/gyro reads to IMU_SAMPLE_FREQ_HZ (decouples the sensor
  // rate from the I2C free-run speed; frees CPU above the control need).
  if (timer_callback_register(bmx160_fast_tick_isr, IMU_FAST_PERIOD_US) != 0) {
    PANIC("Failed to register bmx160_fast_tick_isr");
    return;
  };
}
/* @noreq boot plumbing: spawns the heartbeat task + one-shot boot task. */
void system_init_tasks(void) {
  task_create_named(heartbeat_task, NULL, 576, 0, "heartbeat"); // peak 124
  // boot_task runs the one-shot boot sequence then exits (stack freed); left at
  // 1 KiB since it is not in the steady-state perf view (no measured high-water).
  task_create_named(boot_task, NULL, 1024, 0, "boot");
}
hal_i2c_config_t i2c_config = {
    .clock_speed = HAL_I2C_SPEED_FAST,
    .own_address = I2C_MASTER,
    .acknowledge = true};

/* @noreq top-level boot orchestration: runs the init sequence and starts the
 * scheduler. Cold-boot timing (SYS-TIM-001) is a system-level property
 * verified at bench, not implemented by this entry point. */
int main() {
  /* CONV-01 / CONV-02 canary: forces vayu_status.h and vayu_assert.h
   * into the link; doubles as a real check that the state machine
   * static-init landed before main(). */
  vayu_status_t boot_state_ok =
      (system_state_get() == SYSTEM_STATE_UNINITIALIZED) ? VAYU_OK
                                                         : VAYU_ERR_INVALID;
  VAYU_ASSERT(boot_state_ok == VAYU_OK);

  clock_setup();
  hal_cycle_counter_init();
  vaios_init_config_t cfg = {.internal_clock_setup = 0,
                             .internal_sd_card_setup = 1};

  v_system_init(&cfg);

  init_i2c_manager(&i2c_config);
  fs_owner_boot_init(); /* prealloc/open the blackbox log files */
#ifdef EKF_SELFTEST
  run_ekf_selftest(); /* report over UART before the scheduler starts */
#endif
  pid_config_init(); /* COMM-CMD-003: restore persisted PID tune from SD */
  system_state_init();
  init_sensors();
  system_init_tasks();
  init_tasks();
  init_timer_callbacks();

  scheduler_start();
  while (1)
    ;
}
