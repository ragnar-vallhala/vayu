#include "comm/channel.h"
#include "memory.h"
#include "sensor/bmx160.h"
#include "sensor/imu_buffer.h"
#include "sys/state.h"
#include "task.h"
#include "utils.h"
#include "utils/test_file.h"
#include "utils/timer_callbacks.h"
#include "utils/utils.h"
#include "vaios.h"
#include "vaios_config_default.h"
#include "variables.h"
#include "vayu_tasks.h"

// Global state values
uint32_t bmx160_task_id = 0;

void clock_setup(void) {
  hal_pll_config_t pll_cfg = {
      .input_src = HAL_CLOCK_SOURCE_HSE, /**< External 8 MHz crystal */
      .pll_m = 8,                        /**< PLLM divider */
      .pll_n = 168,                      /**< PLLN multiplier */
      .pll_p = 2,                        /**< PLLP division factor */
      .pll_q = 7                         /**< PLLQ division factor */
  };
  hal_clock_config_t cfg = {
      .source = HAL_CLOCK_SOURCE_PLL /**< Use PLL as system clock */
  };
  hal_clock_init(&cfg, &pll_cfg);
}

void init_sensors(void) {
  imu_buffer_init();
  bmx160_init();
}

void init_tasks(void) {

  task_create(comm_processor_task, NULL, 1024, 0);
  bmx160_task_id = task_create(bmx160_initiate_read, NULL, 2048, 0);
  task_create(rc_ibus_task, NULL, 2048, 0);
  task_create(motor_task, NULL, 2048, 0);
  task_create(imu_telemetry_task, NULL, 2048, 0);
  task_create(flush_task, NULL, 1024, 0);
  // task_create(test_task, NULL, 1024, 0);
}
void init_timer_callbacks(void) {
  timer_callback_init(HIGH_FREQ_TIMER_FREQ);
  if (timer_callback_register(increment_high_freq_timer, 1) != 0) {
    PANIC("Failed to register increment_high_freq_timer");
    return;
  };
  if (timer_callback_register(wake_imu_read_task, 10) != 0) {
    PANIC("Failed to register wake_imu_read_task");
    return;
  }
}
void system_init_tasks(void) {
  task_create(heartbeat_task, NULL, 1024, 0);
  task_create(boot_task, NULL, 1024, 0);
}
int main() {
  clock_setup();
  v_init();
  v_heap_memory_init();
  system_state_init();
  scheduler_init();
  system_init_tasks();
  init_sensors();
  init_timer_callbacks();
  init_tasks();
  scheduler_start();
  while (1)
    ;
}
