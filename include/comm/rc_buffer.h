#ifndef VAYU_RC_BUFFER_H
#define VAYU_RC_BUFFER_H

#include "comm/ibus.h"
#include <stdbool.h>

#define RC_BUFFER_SIZE 5

void rc_buffer_init(void);

bool rc_queue_telemetry_push(const ibus_data_t *data);
bool rc_queue_telemetry_pop(ibus_data_t *out_data);

bool rc_queue_control_push(const ibus_data_t *data);
bool rc_queue_control_pop(ibus_data_t *out_data);

#endif // VAYU_RC_BUFFER_H
