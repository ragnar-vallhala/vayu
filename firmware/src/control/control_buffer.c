#include "control/control_buffer.h"
#include "comm/perf_telemetry.h" /* perf_fifo_fill_row + FIFO ids */
#include "structure.h"

static spsc_fifo_t _telemetry_queue;
static control_telemetry_t _telemetry_buffer[CONTROL_TELEMETRY_BUFFER_SIZE];

/** @noreq control-telemetry SPSC buffer init (infrastructure). */
void control_telemetry_buffer_init(void) {
  spsc_init(&_telemetry_queue, _telemetry_buffer, CONTROL_TELEMETRY_BUFFER_SIZE,
            sizeof(control_telemetry_t));
  spsc_set_policy(&_telemetry_queue, SPSC_POLICY_OVERWRITE);
}

/** @noreq perf-telemetry FIFO-row plumbing (glue). */
int control_buffer_perf_fifos(perf_fifo_row_t *rows, int max) {
  if (max < 1)
    return 0;
  perf_fifo_fill_row(&rows[0], PERF_FIFO_TELEMETRY, &_telemetry_queue);
  return 1;
}

/** @noreq SPSC telemetry-queue push (infrastructure). */
bool control_telemetry_queue_push(const control_telemetry_t *data) {
  return spsc_write(&_telemetry_queue, data, 1) == 1;
}

/** @noreq SPSC telemetry-queue pop (infrastructure). */
bool control_telemetry_queue_pop(control_telemetry_t *out_data) {
  return spsc_read(&_telemetry_queue, out_data, 1) == 1;
}
