/* hwtest firmware entry — the on-hardware bench image (-DVAYU_HW_TEST=ON).
 *
 * Brings up the same clock/RTOS/I2C/sensors/telemetry as production
 * (firmware/src/main.c) but, instead of the flight tasks, runs ONE bench task
 * that executes the check registry and reports over the NavLink test dialect.
 * The vehicle never leaves STANDBY and never arms. This file replaces main.c in
 * the hwtest build (main.c is excluded), so it provides its own boot helpers. */
#include "comm/comm.h"
#include "control/control.h"
#include "navhal.h"
#include "sensor/sensor.h"
#include "storage/fs_owner.h"
#include "sys/state.h"
#include "sys/sys_utils.h"
#include "sys/timer_callbacks.h"
#include "task.h"
#include "utils.h"
#include "utils/util.h"
#include "vaios.h"
#include "variables.h"
#include "vayu_tasks.h"

#include "hwtest_runner.h"

/* ---- check registry (functions live in checks/) -------------------------- */
hw_result_t check_system_clock(void);
hw_result_t check_state_machine(void);
hw_result_t check_imu_whoami(void);
hw_result_t check_baro_whoami(void);
hw_result_t check_ekf_selftest(void);

const hw_check_t hwtest_registry[] = {
    {"system_clock", check_system_clock},
    {"state_machine", check_state_machine},
    {"imu_whoami", check_imu_whoami},
    {"baro_whoami", check_baro_whoami},
    {"ekf_selftest", check_ekf_selftest},
    {0, 0},
};

/* ---- boot helpers (mirrors src/main.c; kept local to the test image) ------ */
static void clock_setup(void) {
  hal_pll_config_t pll_cfg_hse = {.input_src = HAL_CLOCK_SOURCE_HSE,
                                  .pll_m = 8,
                                  .pll_n = 336,
                                  .pll_p = 4,
                                  .pll_q = 7};
  hal_clock_config_t cfg = {.source = HAL_CLOCK_SOURCE_PLL};
  hal_clock_init(&cfg, &pll_cfg_hse);
}

static void init_sensors(void) {
  imu_buffer_init();
  control_telemetry_buffer_init();
  bmx160_init();
  bme280_init();
  rc_buffer_init();
  serial_args_t uart_args = {
      .baud_rate = UART_BAUDRATE, .uart = HAL_UART_6, .timeout = 100};
  get_handler(CHANNEL_TYPE_SERIAL, &g_telemetry_channel, &uart_args,
              uart2_packet_recv_callback);
}

static void init_timer_callbacks(void) {
  timer_callback_init(HIGH_FREQ_TIMER_FREQ);
  timer_callback_register(increment_high_freq_timer, 1);
  timer_callback_register(bmx160_fast_tick_isr, IMU_FAST_PERIOD_US);
}

/* Non-static: bmx160.c references this global i2c_config by name (same contract
 * as production src/main.c). */
hal_i2c_config_t i2c_config = {.clock_speed = HAL_I2C_SPEED_FAST,
                               .own_address = I2C_MASTER,
                               .acknowledge = true};

/* ---- bench task ----------------------------------------------------------- */
static void hwtest_task(void *arg) {
  (void)arg;
  vayu_log("[HWTEST] on-hardware bench firmware up; running checks");
  hwtest_run_all();
  /* Done — idle forever in STANDBY. Never arm; never drive motors. */
  for (;;) {
    v_delay(1000);
  }
}

int main(void) {
  clock_setup();
  hal_cycle_counter_init();
  vaios_init_config_t cfg = {.internal_clock_setup = 0,
                             .internal_sd_card_setup = 1};
  v_system_init(&cfg);

  init_i2c_manager(&i2c_config);
  fs_owner_boot_init();
  system_state_init();
  init_sensors();

  task_create_named(heartbeat_task, NULL, 576, 0, "heartbeat");
  task_create_named(hwtest_task, NULL, 2048, 1, "hwtest");
  init_timer_callbacks();

  scheduler_start();
  while (1) {
  }
}
