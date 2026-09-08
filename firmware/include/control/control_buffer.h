#ifndef VAYU_CONTROL_BUFFER_H
#define VAYU_CONTROL_BUFFER_H

#include "comm/perf_packet.h"
#include "variables.h"
#include <stdbool.h>

/* Mailbox, not a queue: the control loop pushes at loop rate and the telemetry
 * task pops at most one per gate tick. 4 => 2 usable slots (see imu_buffer.h).
 * Was 10, which sat permanently full and only served stale samples. */
#define CONTROL_TELEMETRY_BUFFER_SIZE 4

void control_telemetry_buffer_init(void);
bool control_telemetry_queue_push(const control_telemetry_t *data);
bool control_telemetry_queue_pop(control_telemetry_t *out_data);

/* Fill perf wire rows for this module's SPSC fifos. Returns rows written. */
int control_buffer_perf_fifos(perf_fifo_row_t *rows, int max);

#endif // VAYU_CONTROL_BUFFER_H
