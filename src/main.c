#include "core/cortex-m4/gpio.h"
#include "memory.h"
#include "navhal.h"
#include "sensor/bmx160.h"
#include "task.h"
#include "utils.h"
#include "utils/gpio_types.h"
#include "vaios.h"
#include "variables.h"
#include <stdbool.h>
#include <stdint.h>

uint64_t g_system_time_ms = 0;
uint8_t g_device_id = 0;
bool g_time_synced = false;

// Manual string parsing helpers (No stdlib)
uint64_t v_atou64(const char *s) {
  uint64_t res = 0;
  while (*s >= '0' && *s <= '9') {
    res = res * 10 + (*s - '0');
    s++;
  }
  return res;
}

void v_u64toa(uint64_t val, char *buf) {
  char temp[21];
  int i = 0;
  if (val == 0) {
    buf[0] = '0';
    buf[1] = '\0';
    return;
  }
  while (val > 0) {
    temp[i++] = (val % 10) + '0';
    val /= 10;
  }
  int j;
  for (j = 0; j < i; j++) {
    buf[j] = temp[i - 1 - j];
  }
  buf[j] = '\0';
}

void timer5_callback(void) { g_system_time_ms++; }

void sync_task(void *arg) {
  char buf[64];
  while (!g_time_synced) {
    // Flush any pending characters
    while (uart2_available()) {
      uart2_read_char();
    }

    uart2_write_string("$TIME_REQ\n");

    // Wait up to 200ms for response
    uint32_t start = g_system_time_ms;
    while (!g_time_synced && (g_system_time_ms - start < 200)) {
      if (uart2_available()) {
        uint32_t len = uart2_read_until(buf, sizeof(buf), '\n');
        if (len > 10) {
          if (v_strncmp(buf, "$TIME_SET,", 10) == 0) {
            char *p = &buf[10];
            char *comma = NULL;
            for (int i = 0; p[i] != '\0'; i++) {
              if (p[i] == ',') {
                p[i] = '\0';
                comma = &p[i + 1];
                break;
              }
            }
            if (comma) {
              g_system_time_ms = v_atou64(p);
              g_device_id = (uint8_t)v_atou64(comma);
              g_time_synced = true;
              break;
            }
          }
        }
      }
    }

    if (!g_time_synced) {
      v_delay(800);
    }
  }
  task_exit();
}

void heartbeat_task(void *arg) {
  char time_buf[24];
  char id_buf[8];
  while (1) {
    if (g_time_synced) {
      uart2_write_string("$HB,");
      v_u64toa(g_system_time_ms, time_buf);
      uart2_write_string(time_buf);
      uart2_write_string(",");
      v_u64toa(g_device_id, id_buf);
      uart2_write_string(id_buf);
      uart2_write_string("\n");
    }
    v_delay(1000);
  }
}

void physical_heartbeat(void* args){
  while(1){
   hal_gpio_setmode(GPIO_PB10, GPIO_OUTPUT, GPIO_PUPD_NONE);
   hal_gpio_digitalwrite(GPIO_PB10,GPIO_HIGH);
   v_delay(1000); 
   hal_gpio_digitalwrite(GPIO_PB10,GPIO_LOW);
   v_delay(1000); 
  }
}
void run_bmx();

int main() {
  v_init();
  v_heap_memory_init();

  uint32_t apb1_clk = hal_clock_get_apb1clk();
  uint32_t total_div = apb1_clk / 1000;

  timer_init(TIM5, 0, total_div - 1);
  timer_attach_callback(TIM5, timer5_callback);
  timer_enable_interrupt(TIM5);
  timer_start(TIM5);

  scheduler_init();

  task_create(sync_task, NULL, 2048, 1);
  task_create(heartbeat_task, NULL, 2048, 0);
  task_create(physical_heartbeat, NULL, 512, 0);

  scheduler_start();
  while (1)
    ;
}
