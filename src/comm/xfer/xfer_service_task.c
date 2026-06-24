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
 * Providers (Phase E): FILE (any SD path, up+down), LOG (blackbox download),
 * STREAM (named live source). All SD I/O goes through fs_owner; the STREAM
 * provider is registered but carries no source until one is wired via
 * xfer_stream_register_source().
 */
#include "vayu_tasks.h"

#include "comm/channel.h"            /* channel_tx_overflow_count */
#include "comm/xfer/fs_query.h"        /* filesystem navigation */
#include "comm/xfer/navlink_xfer.h"
#include "comm/xfer/navlink_xfer_tx.h" /* g_xfer_tx_ops, g_fs_query_tx_ops */
#include "comm/xfer/xfer_providers.h"  /* xfer_providers_register_all */
#include "utils.h"                   /* v_get_ticks, v_delay */
#include "vaios.h"

/* Per-tick emission cap shared round-robin across sessions. Keeps a single big
 * download from monopolising the channel; the loop runs often enough that this
 * still sustains the link's chunk rate. */
#define XFER_CHUNK_BUDGET 4

void xfer_service_task(void *args) {
  (void)args;
  xfer_init(&g_xfer_tx_ops);
  fs_query_init(&g_fs_query_tx_ops);
  /* FILE (up+down any SD path), LOG (download the blackbox files), STREAM
   * (named live source). The STREAM provider is registered but sourceless until
   * a source is wired via xfer_stream_register_source(). */
  xfer_providers_register_all();

  for (;;) {
    int emitted = xfer_tick((uint32_t)v_get_ticks(),
                            channel_tx_overflow_count(), XFER_CHUNK_BUDGET);
    /* Directory browse / stat replies (blocking VFS walk lives here too). */
    emitted += fs_query_tick(XFER_CHUNK_BUDGET);
    /* Busy -> short delay to keep chunks flowing; idle -> back off. */
    v_delay(emitted > 0 ? 4 : 10);
  }
}
