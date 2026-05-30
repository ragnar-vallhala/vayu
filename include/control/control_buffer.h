#ifndef VAYU_CONTROL_BUFFER_H
#define VAYU_CONTROL_BUFFER_H

#include "variables.h"
#include <stdbool.h>

#define CONTROL_TELEMETRY_BUFFER_SIZE 10

void control_telemetry_buffer_init(void);
bool control_telemetry_queue_push(const control_telemetry_t *data);
bool control_telemetry_queue_pop(control_telemetry_t *out_data);

#endif // VAYU_CONTROL_BUFFER_H
