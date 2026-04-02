#include "comm/channel.h"
#include "comm/serializer.h"
#include "core/cortex-m4/clock.h"
#include "core/cortex-m4/uart.h"
#include "drivers/i2c_manager.h"
#include "logger/logger.h"
#include "maths/control.h"
#include "navhal.h"
#include "sensor/bmx160.h"
#include "sensor/imu_buffer.h"
#include "sys/state.h"
#include "task.h"
#include "utils.h"
#include "utils/test_file.h"
#include "utils/timer_callbacks.h"
#include "utils/util.h"
#include "utils/utils.h"
#include "utils/v_fs.h"
#include "vaios.h"
#include "vaios_config_default.h"
#include "variables.h"
#include "vayu_tasks.h"

channel_t g_telemetry_channel;
MutexHandle_t g_comm_mutex;
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
      .input_src = HAL_CLOCK_SOURCE_HSI, /**< External 8 MHz crystal */
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
  bmx160_init();

  // Initialize global telemetry
  serial_args_t uart_args = {
      .baud_rate = UART_BAUDRATE, .uart = UART2, .timeout = 100};

  g_comm_mutex = v_mutex_create();

  if (get_handler(CHANNEL_TYPE_SERIAL, &g_telemetry_channel, &uart_args,
                  uart2_packet_recv_callback) != NONE) {
    return;
  }
}

void init_tasks(void) {
  task_create(i2c_manager_task, NULL, 4096, 0);
  task_create(comm_processor_task, NULL, 4096, 0);
  bmx160_task_id = task_create(bmx160_initiate_read, NULL, 4096, 2);
  task_create(rc_ibus_task, NULL, 4096, 0);
  task_create(control_task, NULL, 1024 * 5, 1); // Higher priority for control
  task_create(imu_telemetry_task, NULL, 4096, 0);
  task_create(flush_task, NULL, 4096, 0);
  // task_create(test_task, NULL, 4096, 0);
  // task_create(test_task, NULL, 512, 0);
}
void init_timer_callbacks(void) {
  timer_callback_init(HIGH_FREQ_TIMER_FREQ);
  if (timer_callback_register(increment_high_freq_timer, 1) != 0) {
    PANIC("Failed to register increment_high_freq_timer");
    return;
  };
  if (timer_callback_register(wake_imu_read_task, 1000) != 0) {
    PANIC("Failed to register wake_imu_read_task");
    return;
  }
}
void system_init_tasks(void) {
  task_create(heartbeat_task, NULL, 2048, 0);
  task_create(boot_task, NULL, 1024, 0);
}
hal_i2c_config_t i2c_config = {
    .clock_speed = FAST_MODE, .own_address = I2C_MASTER, .acknowledge = true};

int main() {
  clock_setup();
  vaios_init_config_t cfg = {.internal_clock_setup = 0,
                             .internal_sd_card_setup = 1};

  v_system_init(&cfg);

  init_i2c_manager(&i2c_config);
  logger_init();
  system_state_init();
  init_sensors();
  system_init_tasks();
  init_tasks();
  init_timer_callbacks();

  scheduler_start();
  while (1)
    ;
}
