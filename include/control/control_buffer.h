#ifndef VAYU_CONTROL_BUFFER_H
#define VAYU_CONTROL_BUFFER_H

#include "comm/perf_packet.h"
#include "variables.h"
#include <stdbool.h>

#define CONTROL_TELEMETRY_BUFFER_SIZE 10

void control_telemetry_buffer_init(void);
bool control_telemetry_queue_push(const control_telemetry_t *data);
bool control_telemetry_queue_pop(control_telemetry_t *out_data);

/* Fill perf wire rows for this module's SPSC fifos. Returns rows written. */
int control_buffer_perf_fifos(perf_fifo_row_t *rows, int max);

#endif // VAYU_CONTROL_BUFFER_H
