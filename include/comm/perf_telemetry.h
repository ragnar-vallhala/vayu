#ifndef VAYU_COMM_PERF_TELEMETRY_H
#define VAYU_COMM_PERF_TELEMETRY_H

/**
 * @file perf_telemetry.h
 * @brief Periodic task that streams the vaios perf/observability snapshot to
 *        the GCS as fragmented PACKET_TYPE_PERF_STATS packets.
 */

#include "comm/perf_packet.h"
#include "structure.h"

/* Periodic perf reporter (FC -> GCS). One report = GLOBAL + TASKS + FIFOS. */
void perf_telemetry_task(void *args);

/* Fill one FIFO wire row from a live SPSC fifo. Used by the buffer modules to
 * expose their otherwise-static fifos without leaking the pointers. */
void perf_fifo_fill_row(perf_fifo_row_t *row, uint8_t fifo_id,
                        const spsc_fifo_t *f);

#endif /* VAYU_COMM_PERF_TELEMETRY_H */
