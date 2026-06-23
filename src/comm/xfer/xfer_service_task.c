/**
 * @file src/comm/xfer/xfer_service_task.c
 * @brief Dedicated service task for the bulk-transfer (FTP) substrate.
 *
 * Runs the xfer state machine off the comm + control tasks (prio 0). All the
 * blocking SD reads (provider->read / provider->open) and all paced emission
 * live here — the comm-task handlers only touch session state, preserving the
 * C1->C3 "no blocking SD on the comm task" invariant (see the centralised FS
 * owner). Body: bind the emitter, register providers, then loop xfer_tick().
 *
 * Phase C wires the transport + SM + task; real providers (file/log/stream) are
 * registered here in Phase E. Until then the substrate is live but has no
 * provider, so any XFER_OPEN is answered UNSUPPORTED — the wiring + RAM budget
 * are exercised on hardware without yet exposing the SD.
 */
#include "vayu_tasks.h"

#include "comm/channel.h"            /* channel_tx_overflow_count */
#include "comm/xfer/navlink_xfer.h"
#include "comm/xfer/navlink_xfer_tx.h" /* g_xfer_tx_ops */
#include "utils.h"                   /* v_get_ticks, v_delay */
#include "vaios.h"

/* Per-tick emission cap shared round-robin across sessions. Keeps a single big
 * download from monopolising the channel; the loop runs often enough that this
 * still sustains the link's chunk rate. */
#define XFER_CHUNK_BUDGET 4

void xfer_service_task(void *args) {
  (void)args;
  xfer_init(&g_xfer_tx_ops);
  /* Phase E: xfer_register_provider(file/log/stream) goes here. */

  for (;;) {
    int emitted = xfer_tick((uint32_t)v_get_ticks(),
                            channel_tx_overflow_count(), XFER_CHUNK_BUDGET);
    /* Busy -> short delay to keep chunks flowing; idle -> back off. */
    v_delay(emitted > 0 ? 4 : 10);
  }
}
