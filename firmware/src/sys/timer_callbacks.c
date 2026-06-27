#include "sys/timer_callbacks.h"
#include "navhal.h"
#include "utils.h"
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

/* TIM5 update ISR: dispatches each registered callback at its requested
 * sub-millisecond period.
 *
 * @implements SYS-TIM-005 */
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
      _callbacks[i].last_call = (uint32_t)_interrupt_count;
    }
  }
}

void timer_callback_init(uint32_t freq_hz) {
  v_memset(_callbacks, 0, sizeof(_callback_t) * MAX_TIMER_CALLBACKS);
  // hal_timer_init_freq() handles the PSC/ARR derivation from the timer's
  // base clock for the requested update frequency.
  _timer_interrupt_freq = freq_hz;
  hal_timer_init_freq(_timer_inst, freq_hz);
  hal_timer_attach_callback(_timer_inst, _timer_isr_handler);
  hal_timer_enable_interrupt(_timer_inst);
}
/* @noreq trivial accessor: returns the configured HF timer frequency. */
uint32_t timer_get_callback_frequency(void) { return _timer_interrupt_freq; }

int timer_callback_register(void (*fn)(void), uint32_t us_delay) {
  if (_callback_count >= MAX_TIMER_CALLBACKS) {
    return -1;
  }
  _callbacks[_callback_count++] =
      (_callback_t){.callback = fn, .us_delay = us_delay, .last_call = 0};
  return 0;
}
