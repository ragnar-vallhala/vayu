#include "utils/timer_callbacks.h"
#include "core/cortex-m4/clock.h"
#include "core/cortex-m4/rcc_reg.h"
#include "core/cortex-m4/timer.h"
#include "utils.h"
#include "utils/utils.h"
#include "variables.h"
#include <stddef.h>
#include <stdint.h>

typedef struct {
  void (*callback)(void);
  uint32_t us_delay;
  uint32_t last_call;
} _callback_t;

static _callback_t _callbacks[MAX_TIMER_CALLBACKS];

static uint8_t _callback_count = 0;
static hal_timer_t _timer_inst = TIM5;
static uint32_t _timer_interrupt_freq = 0;
static volatile uint64_t _interrupt_count = 0;

static void _timer_isr_handler(void) {
  _interrupt_count++;
  uint32_t us_per_interrupt = 1000000 / _timer_interrupt_freq;
  if (us_per_interrupt == 0)
    us_per_interrupt = 1;

  for (int i = 0; i < _callback_count; i++) {
    uint32_t delay_interrupts = _callbacks[i].us_delay / us_per_interrupt;
    if (delay_interrupts == 0)
      delay_interrupts = 1;

    if (_interrupt_count >= _callbacks[i].last_call + delay_interrupts) {
      _callbacks[i].callback();
      _callbacks[i].last_call = _interrupt_count;
    }
  }
}

void timer_callback_init(uint32_t freq_hz) {
  v_memset(_callbacks, 0, sizeof(_callback_t) * MAX_TIMER_CALLBACKS);
  // Calculate PSC and ARR for desired frequency
  // Freq = TimerClock / ((PSC + 1) * (ARR + 1))
  // We'll use hal_clock_get_apb1clk() as the base.
  // Note: On STM32F4, if APB prescaler is 1, Timer Clock = APB Clock.
  // Otherwise, Timer Clock = 2 * APB Clock.
  uint32_t ppre1 = (RCC->CFGR >> RCC_CFGR_PPRE1_BIT) & 0x7;

  uint32_t apb_clk;
  uint32_t timer_clk;

  apb_clk = hal_clock_get_apb1clk();
  timer_clk = (ppre1 == 0) ? apb_clk : (apb_clk * 2);

  // To minimize error and avoid overflow:
  // Try PSC = 0 first.
  uint32_t total_div = timer_clk / freq_hz;
  uint32_t psc = 0;
  uint32_t arr = total_div - 1;
  _timer_interrupt_freq = freq_hz;
  timer_init(_timer_inst, psc, arr);
  timer_attach_callback(_timer_inst, _timer_isr_handler);
  timer_enable_interrupt(_timer_inst);
}
uint32_t timer_get_callback_frequency(void) { return _timer_interrupt_freq; }

int timer_callback_register(void (*fn)(void), uint32_t us_delay) {
  if (_callback_count >= MAX_TIMER_CALLBACKS) {
    return -1;
  }
  _callbacks[_callback_count++] =
      (_callback_t){.callback = fn, .us_delay = us_delay, .last_call = 0};
  return 0;
}
