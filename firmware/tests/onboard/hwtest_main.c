/*
 * Copyright (C) 2026 NAVRobotec Pvt Ltd
 * Author: Ragnar Vallhala
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
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

#include "coverage_dump.h"
#include "hwtest_runner.h"

/* ---- check registry (functions live in checks/) -------------------------- */
hw_result_t check_system_clock(void);
hw_result_t check_state_machine(void);
hw_result_t check_heap_free(void);
hw_result_t check_imu_whoami(void);
hw_result_t check_baro_whoami(void);
hw_result_t check_baro_present(void);
hw_result_t check_imu_gyro_still(void);
hw_result_t check_imu_accel(void);
hw_result_t check_imu_driver_api(void);
hw_result_t check_baro_driver_api(void);
hw_result_t check_sd_readback(void);
hw_result_t check_ekf_selftest(void);

/* Sensor reads are direct + bus-timeout-bounded (no IMU read task here, so the
 * bus is free); ekf_selftest is last as it is the most stack-heavy. */
const hw_check_t hwtest_registry[] = {
    {"system_clock", check_system_clock},
    {"state_machine", check_state_machine},
    {"heap_free", check_heap_free},
    {"imu_whoami", check_imu_whoami},
    {"baro_whoami", check_baro_whoami},
    {"baro_present", check_baro_present},
    {"imu_gyro_still", check_imu_gyro_still},
    {"imu_accel", check_imu_accel},
    {"imu_driver_api", check_imu_driver_api},
    {"baro_driver_api", check_baro_driver_api},
    {"sd_readback", check_sd_readback},
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
  /* Let comm/fs_owner come up, then run the suite ONCE: results are written to
   * 0:hwtest.txt on SD (download over xfer) and a single DONE status goes out
   * over the link. No streaming. Then idle in STANDBY; never arm. */
  v_delay(800);
  hwtest_run_all();
  /* NOTE: spinning up the DMA-driven IMU read task here (to cover the read/
   * process/dma-callback path) was tried and WEDGES the dump (0-byte cov.gcd) —
   * even after the direct-read checks finish, the async I2C DMA chain interferes
   * with the coverage write-out. That ~340-line path stays uncovered on the
   * static bench by design (i2c-bus-sharing); cover it in SITL/HIL instead. */
  coverage_dump(); /* C3: dump on-target gcov to 0:cov.gcd (no-op unless -DVAYU_HW_TEST_COV) */
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

  /* boot_task drives INIT -> STANDBY (clock + SD checks) then exits — the proper
   * boot the state machine expects. */
  task_create_named(boot_task, NULL, 1024, 0, "boot");
  task_create_named(heartbeat_task, NULL, 576, 0, "heartbeat");
  /* flush_task DMAs buffered telemetry out of UART6; without it write_channel()
   * only fills the TX buffer and nothing is transmitted. */
  task_create_named(flush_task, NULL, 640, 0, "flush");
  /* NOTE: the IMU read task is intentionally NOT run here. The sensor checks do
   * direct I2C reads, which conflict with the read task's single-owner DMA loop
   * (i2c-bus-sharing) and wedge the bench. Accel/gyro therefore read 0 (a known
   * finding) — but the driver code still executes, so its coverage is captured.
   * Reading live samples from imu_buffer instead of direct I2C is a follow-up. */
  /* fs_owner persists 0:hwtest.txt to SD. The bench does NOT run comm/xfer
   * itself; the results file is recovered by reflashing production firmware
   * (proven comm+xfer) and downloading it — the SD file survives the reflash. */
  task_create_named(fs_owner_task, NULL, 1152, 0, "fs_owner");
  /* Priority 0 (co-equal), 8 KiB stack (ekf_selftest builds matrices on-stack). */
  task_create_named(hwtest_task, NULL, 8192, 0, "hwtest");
  init_timer_callbacks();

  scheduler_start();
  while (1) {
  }
}
