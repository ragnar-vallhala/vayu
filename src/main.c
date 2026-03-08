#include "comm/channel.h"
#include "memory.h"
#include "sensor/bmx160.h"
#include "sensor/imu_buffer.h"
#include "task.h"
#include "utils/test_file.h"
#include "utils/timer_callbacks.h"
#include "utils/utils.h"
#include "vaios.h"
#include "variables.h"
#include "vayu_tasks.h"

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
int main() {
  clock_setup();
  v_init();
  v_heap_memory_init();

  timer_callback_init(HIGH_FREQ_TIMER_FREQ);
  if (timer_callback_register(increment_high_freq_timer, 1) != 0) {
    return 1;
  };

  imu_buffer_init();
  bmx160_init();

  if (timer_callback_register(bmx160_initiate_read, 1000) != 0) {
    return 1;
  }

  scheduler_init();
  task_create(physical_heartbeat, NULL, 512, 0);
  task_create(comm_processor_task, NULL, 1024, 0);
  task_create(imu_telemetry_task, NULL, 2048, 0);
  task_create(flush_task, NULL, 1024, 0);
  task_create(test_task, NULL, 1024, 0);
  scheduler_start();
  while (1)
    ;
}
