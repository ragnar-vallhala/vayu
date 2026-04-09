#include "maths/control_buffer.h"
#include "structure.h"

static spsc_fifo_t _telemetry_queue;
static control_telemetry_t _telemetry_buffer[CONTROL_TELEMETRY_BUFFER_SIZE];

void control_telemetry_buffer_init(void) {
  spsc_init(&_telemetry_queue, _telemetry_buffer, CONTROL_TELEMETRY_BUFFER_SIZE,
            sizeof(control_telemetry_t));
  spsc_set_policy(&_telemetry_queue, SPSC_POLICY_OVERWRITE);
}

bool control_telemetry_queue_push(const control_telemetry_t *data) {
  return spsc_write(&_telemetry_queue, data, 1) == 1;
}

bool control_telemetry_queue_pop(control_telemetry_t *out_data) {
  return spsc_read(&_telemetry_queue, out_data, 1) == 1;
}
