#include "comm/comm.h"
#include "control/control.h"
#include "sensor/sensor.h"
#include "logger/logger.h"
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

// Global state values
uint32_t bmx160_task_id = 0;

void clock_setup(void) {
  hal_pll_config_t pll_cfg_hse = {
      .input_src = HAL_CLOCK_SOURCE_HSE, /**< External 8 MHz crystal */
      .pll_m = 8,                        /**< PLLM divider */
      .pll_n = 336,                      /**< PLLN multiplier */
      .pll_p = 4,                        /**< PLLP division factor */
      .pll_q = 7                         /**< PLLQ division factor */
  };
  hal_pll_config_t pll_cfg_hsi = {
      .input_src = HAL_CLOCK_SOURCE_HSI, /**< Internal 16 MHz crystal */
      .pll_m = 16,                       /**< PLLM divider */
      .pll_n = 336,                      /**< PLLN multiplier */
      .pll_p = 4,                        /**< PLLP division factor */
      .pll_q = 7                         /**< PLLQ division factor */
  };
  hal_clock_config_t cfg = {
      .source = HAL_CLOCK_SOURCE_PLL /**< Use PLL as system clock */
  };
  hal_clock_init(&cfg, &pll_cfg_hse);
}

void init_sensors(void) {
  imu_buffer_init();
  control_telemetry_buffer_init();
  bmx160_init();
  rc_buffer_init();

  // Initialize global telemetry — USART6 (PC6 TX / PC7 RX) per Vayu PCB wiring.
  serial_args_t uart_args = {
      .baud_rate = UART_BAUDRATE, .uart = HAL_UART_6, .timeout = 100};

  if (get_handler(CHANNEL_TYPE_SERIAL, &g_telemetry_channel, &uart_args,
                  uart2_packet_recv_callback) != NONE) {
    return;
  }
}

void init_tasks(void) {
  task_create(comm_processor_task, NULL, 4096, 0);
  bmx160_task_id = task_create(bmx160_initiate_read, NULL, 4096, 2);
  task_create(rc_ibus_task, NULL, 4096, 0);
  task_create(angle_controller_task, NULL, 1024 * 8,
              1); // Higher priority for control
  task_create(angle_rate_controller_task, NULL, 1024 * 8,
              1);                             // Higher priority for control
  task_create(motor_task, NULL, 1024 * 4, 1); // Higher priority for control
  task_create(imu_telemetry_task, NULL, 1024 * 3, 0);
  task_create(flush_task, NULL, 4096, 0);
  // task_create(test_task, NULL, 4096, 0);
}
void init_timer_callbacks(void) {
  timer_callback_init(HIGH_FREQ_TIMER_FREQ);
  if (timer_callback_register(increment_high_freq_timer, 1) != 0) {
    PANIC("Failed to register increment_high_freq_timer");
    return;
  };
}
void system_init_tasks(void) {
  task_create(heartbeat_task, NULL, 2048, 0);
  task_create(boot_task, NULL, 1024, 0);
}
hal_i2c_config_t i2c_config = {
    .clock_speed = HAL_I2C_SPEED_FAST,
    .own_address = I2C_MASTER,
    .acknowledge = true};

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
  logger_init();
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
