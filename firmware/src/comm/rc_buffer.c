#include "comm/rc_buffer.h"
#include "comm/perf_telemetry.h" /* perf_fifo_fill_row + FIFO ids */
#include "structure.h"

#define RC_BUFFER_INTERNAL_CAPACITY (RC_BUFFER_SIZE + 1)

static ibus_data_t _rc_telemetry_buffer[RC_BUFFER_INTERNAL_CAPACITY];
static ibus_data_t _rc_control_buffer[RC_BUFFER_INTERNAL_CAPACITY];

static spsc_fifo_t _rc_telemetry_queue;
static spsc_fifo_t _rc_control_queue;

void rc_buffer_init(void) {
  spsc_init(&_rc_telemetry_queue, _rc_telemetry_buffer,
            RC_BUFFER_INTERNAL_CAPACITY, sizeof(ibus_data_t));
  spsc_set_policy(&_rc_telemetry_queue, SPSC_POLICY_OVERWRITE);

  spsc_init(&_rc_control_queue, _rc_control_buffer, RC_BUFFER_INTERNAL_CAPACITY,
            sizeof(ibus_data_t));
  spsc_set_policy(&_rc_control_queue, SPSC_POLICY_OVERWRITE);
}

int rc_buffer_perf_fifos(perf_fifo_row_t *rows, int max) {
  int n = 0;
  if (n < max)
    perf_fifo_fill_row(&rows[n++], PERF_FIFO_RC_TELEMETRY, &_rc_telemetry_queue);
  if (n < max)
    perf_fifo_fill_row(&rows[n++], PERF_FIFO_RC_CONTROL, &_rc_control_queue);
  return n;
}

bool rc_queue_telemetry_push(const ibus_data_t *data) {
  return spsc_write(&_rc_telemetry_queue, data, 1) == 1;
}

bool rc_queue_telemetry_pop(ibus_data_t *out_data) {
  return spsc_read(&_rc_telemetry_queue, out_data, 1) == 1;
}

bool rc_queue_control_push(const ibus_data_t *data) {
  return spsc_write(&_rc_control_queue, data, 1) == 1;
}

bool rc_queue_control_pop(ibus_data_t *out_data) {
  return spsc_read(&_rc_control_queue, out_data, 1) == 1;
}
