/*
 * Copyright (C) 2026 NAVRobotec Pvt Ltd
 * Author: Ragnar Vallhala
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
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
 * Providers: FILE (any SD path, up+down), LOG (blackbox download),
 * STREAM (named live source). All SD I/O goes through fs_owner; the STREAM
 * provider is registered but carries no source until one is wired via
 * xfer_stream_register_source().
 */
#include "vayu_tasks.h"

#include "comm/channel.h"       /* channel_tx_overflow_count */
#include "comm/xfer/fs_query.h" /* filesystem navigation */
#include "comm/xfer/navlink_xfer.h"
#include "comm/xfer/navlink_xfer_tx.h" /* g_xfer_tx_ops, g_fs_query_tx_ops */
#include "comm/xfer/xfer_providers.h"  /* xfer_providers_register_all */
#include "storage/fs_owner.h"          /* fs_owner_suppress_logs */
#include "utils.h"                     /* v_get_ticks, v_delay */
#include "vaios.h"

/* Per-tick emission cap shared round-robin across sessions. Keeps a single big
 * download from monopolising the channel; the loop runs often enough that this
 * still sustains the link's chunk rate. */
#define XFER_CHUNK_BUDGET 4

/** @implements COMM-XFER-001 */
void xfer_service_task(void *args) {
  (void)args;
  xfer_init(&g_xfer_tx_ops);
  fs_query_init(&g_fs_query_tx_ops);
  /* FILE (up+down any SD path), LOG (download the blackbox files), STREAM
   * (named live source). The STREAM provider is registered but sourceless until
   * a source is wired via xfer_stream_register_source(). */
  xfer_providers_register_all();

  for (;;) {
    /* Quiesce best-effort blackbox logging for the whole transfer window so the
     * SD sees only the transfer's file (sequential = the one corruption-free
     * multi-file pattern). Released the moment all sessions go idle. */
    fs_owner_suppress_logs(xfer_active());
    int emitted = xfer_tick((uint32_t)v_get_ticks(),
                            channel_tx_overflow_count(), XFER_CHUNK_BUDGET);
    /* Directory browse / stat replies (blocking VFS walk lives here too). */
    emitted += fs_query_tick(XFER_CHUNK_BUDGET);
    /* Busy -> short delay to keep chunks flowing; idle -> back off. */
    v_delay(emitted > 0 ? 4 : 10);
  }
}
