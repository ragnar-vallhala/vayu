#include "comm/channel.h"
#include "core/cortex-m4/uart.h"
#include "logger/sd.h"
#include "memory.h"
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

// Global state values
uint32_t bmx160_task_id = 0;

void clock_setup(void) {
  hal_pll_config_t pll_cfg_hse = {
      .input_src = HAL_CLOCK_SOURCE_HSE, /**< External 8 MHz crystal */
      .pll_m = 8,                        /**< PLLM divider */
      .pll_n = 168,                      /**< PLLN multiplier */
      .pll_p = 2,                        /**< PLLP division factor */
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
}

void init_sd_card(void) {
  uart2_write_string("\n\r--- NavHAL FatFS/POSIX Test ---\n\r");

  hal_sdio_config_t sd_config = {.clock_div = 118, .bus_width = 1};
  if (sdio_init(&sd_config) != HAL_SDIO_OK) {
    uart2_write_string("SDIO Peripheral Init Failed!\n\r");
    while (1)
      ;
  }

  /* 3. Perform SD Card Handshake */
  if (sdio_card_init() != HAL_SDIO_OK) {
    uart2_write_string("SD Card Handshake Failed!\n\r");
    while (1)
      ;
  }
  uart2_write_string("SD Card Ready.\n\r");

  /* 4. Initialize Filesystem */
  if (v_fs_init() != 0) {
    uart2_write_string("Filesystem Mount Failed!\n\r");

    while (1)
      ;
  }

  uart2_write_string("Filesystem Mounted.\n\r");

  /* 5. Create and write to a file */
  uart2_write_string("Creating test.txt...\n\r");

  v_fd_t fd = v_open("test.txt", V_O_CREAT | V_O_RDWR | V_O_TRUNC);
  if (fd < 0) {
    uart2_write_string("Failed to open test.txt for writing.\n\r");
    while (1)
      ;
  }

  const char *msg = "Hello from NavHAL FatFS!";
  int written = v_write(fd, msg, 24);
  if (written > 0) {
    uart2_write_string("Write Success (");
    uart2_write(written);
    uart2_write_string(" bytes).\n\r");
  } else {
    uart2_write_string("Write Error (Code: ");
    uart2_write(-written);
    uart2_write_string(").\n\r");
  }
  v_close(fd);

  /* 6. Read back from the file */
  uart2_write_string("Reading back test.txt...\n\r");
  fd = v_open("test.txt", V_O_RDONLY);
  if (fd < 0) {
    uart2_write_string("Failed to open test.txt for reading.\n\r");
    while (1)
      ;
  }

  uint8_t read_buf[32] __attribute__((aligned(4)));
  hal_memset(read_buf, 0, sizeof(read_buf));
  int read_bytes = v_read(fd, read_buf, 25);
  if (read_bytes > 0) {
    uart2_write_string("Read Success (");
    uart2_write(read_bytes);
    uart2_write_string(" bytes).\n\r");
    uart2_write_string("Data: ");
    uart2_write_string((char *)read_buf);
    uart2_write_string("\n\r");
  } else {
    uart2_write_string("Read Error (Code: ");
    uart2_write(-read_bytes);
    uart2_write_string(").\n\r");
  }
  v_close(fd);

  uart2_write_string("Test Finished Success.\n\r");
}

void init_tasks(void) {

  task_create(comm_processor_task, NULL, 1024, 3);
  bmx160_task_id = task_create(bmx160_initiate_read, NULL, 2048, 5);
  task_create(rc_ibus_task, NULL, 2048, 4);
  task_create(motor_task, NULL, 2048, 0);
  task_create(imu_telemetry_task, NULL, 2048, 0);
  task_create(flush_task, NULL, 1024, 0);
  task_create(test_task, NULL, 1024, 0);
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
int main() {
  clock_setup();
  vaios_init_config_t cfg = {.internal_clock_setup = 0,
                             .internal_sd_card_setup = 1};
  v_system_init(&cfg);
  uart2_write_string("System Started\n\r");
  system_state_init();
  init_sensors();
  system_init_tasks();
  init_timer_callbacks();
  init_tasks();
  scheduler_start();
  while (1)
    ;
}
