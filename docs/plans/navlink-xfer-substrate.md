# Generic NavLink Bulk-Transfer + Streaming Substrate ("xfer")

## Context

We just centralised all SD writes behind the FS owner (`docs/plans/centralised-fs-owner.md`).
The natural next capability is moving **files and data streams to/from the FC over the UART
link** — but instead of a one-off file-transfer feature, the goal is a **generic, reusable
"FTP-like" substrate**: a reliable chunked bulk-transfer + streaming sublayer over NavLink v2
that future apps (file transfer, blackbox-log download, config/tune export, live data streams)
all plug into via a **provider registry**, without rebuilding the transport each time.

The FC already has a consistent house style for bulk transfer (sysid dump, PARAM enumeration,
telemetry streaming): *near-stateless FC, self-describing chunks, GCS-driven idempotent
re-request, FC-side pacing against `channel_tx_overflow_count`*. This substrate **formalises
that proven model** rather than inventing a TCP-like windowed transport.

**Locked decisions (with the user):**
- **Scope:** full end-to-end — transport + provider(s) + GCS/Python client + conformance tests.
- **Base provider:** a generic **read/write any SD path** provider; log-download and
  config/tune blob are thin specialisations (open a specific path). STREAM mode is built into
  the protocol so live-stream providers plug in later.
- **Reliability:** offset-addressed **cumulative-ack + GCS-driven idempotent re-request**
  (resume by rewinding the FC cursor). No sliding window.

**Resolved design choices (consistent with the above):** N=2 concurrent sessions (configurable
to 4); fold stream rate into `XFER_OPEN` (supersede the proposed `CMD_SET_STREAM_RATE`); LOG
provider = raw circular-file dump; fs_owner gets a **dedicated third write-at lane** so uploads
can't starve PID/calib saves.

**The one invariant that governs everything (the C1→C3 fix must hold):** no blocking SD I/O on
the comm task. Uploads enqueue to fs_owner (async); downloads read on a dedicated xfer task; the
comm-task handlers only touch session state.

## Architecture

```
            GCS (Python sim / real GCS)  ── NavLink v2, USART6 ~230400, ~150 pkt/s shared
  ──────────────────┼──────────────────────────────────────────────────────────
  comm_processor_task (250Hz)            xfer_service_task (NEW, prio 0)
    navlink_router_poll                    xfer_tick(now, tx_overflow, budget):
      on_xfer_open  (deferred ack)           - download: provider->read + emit XFER_DATA, paced
      on_xfer_data  (upload ingest)          - upload:   periodic XFER_ACK flow control
      on_xfer_ack   (download flow ctl)      - per-session timeouts; deferred COMMAND_ACK + XFER_INFO
      on_xfer_close                          - blocking SD reads live HERE, off the comm task
            │ (state only, no SD)   ▲
            ▼                       │ vtable
                       navlink_xfer.c  (generic SM, session table N=2, registry)
                                  │
              ┌───────────────────┼────────────────────┐
         file_provider       log_provider         stream_provider
         (up+down, any path)  (download, blackbox) (subscribe @rate, best-effort)
              │                    │
        fs_owner write-at lane (async) / fs_owner_read_at (sync, mutex-serialised)
```

The generated `navlink_msgs.h` is included **only** in the router glue and a small xfer TX seam —
the core SM and providers are codec-blind (same separation as `navlink_tx.c`).

## Dialect additions (`navlink/dialect.json`) — Phase A

Free msgids confirmed: commands **8201/8202** (in 0x2000–0x2FFF → auto-ack + time-sync gate),
telemetry **1042/1043/1044** (no ack; SM owns reliability). Each wire size ≤255 (validator
enforces). One enum `xfer_flags {NONE=0,DONE=1,NAK=2,ABORT=4,EOF=8}`. Reuse `command_result` for
open results.

| Msg | id | dir | key fields (wire size) |
|---|---|---|---|
| `XFER_OPEN` | 8201 cmd | GCS→FC | target_sys/comp, req_seq, session, dir(0=down/1=up), mode(0=file/1=stream), service_id:u16, offset_start:u32, rate_hz:u16, arg:char[32] (46B) |
| `XFER_INFO` | 1042 tlm | FC→GCS | session, result(enum), chunk_size, _pad, total_size:u32 (0xFFFFFFFF=stream), mtime:u32 **extension** (12B) |
| `XFER_DATA` | 1043 tlm | both | session, flags, len, offset:u32, **data:u8[247]** (254B ≤255 ✓) |
| `XFER_ACK` | 1044 tlm | both | session, flags, result(enum), _pad, next_offset:u32 (8B) |
| `XFER_CLOSE` | 8202 cmd | GCS→FC | target_sys/comp, req_seq, session, result(enum) (5B) |

Notes: `data[247]` leaves a 1-byte margin under the 255 cap; `chunk_size` is advertised in
XFER_INFO so neither side hard-codes it. The SM **trusts `len`, never the unpacked array tail**
(trailing-zero truncation compresses the final short chunk, like SYSID_SAMPLE). `mtime` is an
extension field → ABI-safe, excluded from CRC_EXTRA. The sim auto-acks 8192–12319 blindly, so
`endpoint.py` must reroute XFER_OPEN/CLOSE to the real two-phase reply (ACK + INFO).

## New module + key signatures

- `include/comm/xfer/navlink_xfer.h`, `src/comm/xfer/navlink_xfer.c` — session table, registry,
  the dual-direction state machine. Provider vtable:
  ```c
  typedef struct {
    uint16_t service_id; const char *name;
    int  (*open)(xfer_session_t*, const xfer_open_args_t*, uint32_t *total_out);
    int  (*read)(xfer_session_t*, uint32_t off, uint8_t *buf, uint16_t max);   /* download */
    int  (*write)(xfer_session_t*, uint32_t off, const uint8_t *buf, uint16_t len); /* upload */
    int  (*poll)(xfer_session_t*, uint8_t *buf, uint16_t max);  /* stream only */
    void (*close)(xfer_session_t*, int result);
  } xfer_provider_t;
  void xfer_init(void); int xfer_register_provider(const xfer_provider_t*);
  int  xfer_on_open(const xfer_open_args_t*, uint8_t req_seq, ...);  /* -> COMMAND_ACK result */
  void xfer_on_data(uint8_t s, uint32_t off, const uint8_t*, uint8_t len, uint8_t flags);
  void xfer_on_ack (uint8_t s, uint32_t next_off, uint8_t flags);
  int  xfer_on_close(uint8_t s, uint8_t result);
  int  xfer_tick(uint32_t now_ms, uint32_t tx_overflow, int chunk_budget);
  ```
  `#define XFER_MAX_SESSIONS 2`, `XFER_CHUNK_MAX 247`, `XFER_ARG_MAX 32`.
- `src/comm/xfer/navlink_xfer_tx.c` — the only xfer file including `navlink_msgs.h`:
  `xfer_tx_data/_ack/_info(...)` build via `navlink_<msg>_encode` + `write_channel(g_telemetry_channel,...)`
  (same shape as `navlink_tx_sysid_sample`).
- `src/comm/xfer/providers/{file,log,stream}_provider.c` — each `*_provider_register()` at boot.
- Router glue in `src/comm/navlink_router.c`: add `on_xfer_open` (returns `navlink_ack_deferred()`),
  `on_xfer_data`, `on_xfer_ack`, `on_xfer_close`; register by field-assignment in
  `navlink_router_init()`. `on_xfer_open` maps the codec struct → `xfer_open_args_t` (stamping
  `hdr->sysid/compid` as reply target) and defers; `xfer_tick` later emits COMMAND_ACK + XFER_INFO
  via `navlink_command_ack_send` + `xfer_tx_info` — exactly the `arm_ack_service()` pattern.

## Session state machine (cumulative-ack, resume-by-rewind)

Per-session: state, dir/mode/service_id, gcs_sys/comp, open_req_seq, total_size, chunk_size,
**cursor** (download: next offset to emit; upload: next contiguous offset expected), acked_offset,
last_rx/tx_ms, deadline_ms, provider, user scratch, rate_hz/next_due_ms.

- **Download (FC→GCS):** OPEN → provider.open sets total_size, cursor=offset_start → (tick)
  COMMAND_ACK(ACCEPTED)+XFER_INFO → ACTIVE: emit `XFER_DATA(cursor, n, EOF?)`, cursor+=n, paced by
  budget + `tx_overflow`. On `XFER_ACK{next_offset}`: if `next_offset<cursor` **rewind** cursor
  (idempotent re-read = resume/refill); `DONE`→close; `ABORT`→close.
- **Upload (GCS→FC):** OPEN → provider.open (truncate/create) → ACTIVE: `on_xfer_data` (comm task):
  if `offset==cursor` → `provider.write` (**enqueues fs_owner write-at, never inline**); on accept
  cursor+=len; `offset<cursor` dup-ignore; `offset>cursor` gap-ignore. (tick) emit periodic
  `XFER_ACK{next_offset=cursor}` (flow control); GCS retransmits ≥cursor. On `EOF` + fs_owner drained
  → DONE. If the write-at lane is full, `provider.write` returns "not accepted", cursor stalls, and
  the next XFER_ACK tells the GCS to pause/retransmit.
- **Stream (open-ended):** total_size=0xFFFFFFFF; `provider.poll` at rate_hz; never rewinds
  (best-effort); `tx_overflow` rising → skip stream emission first.

Timeouts: INFO retry 500ms, ACK period 200ms, idle 5s → auto-close(FAILED)+provider.close, DONE
linger 1s for late acks. Error→`COMMAND_ACK` mapping: table-full→TEMPORARILY_REJECTED, unknown
service→UNSUPPORTED, open error→FAILED, bad dir/mode→DENIED; sub-code in `result_param2`. Dedup
deferred-open by (session, req_seq) to survive XFER_OPEN retransmit.

## fs_owner extensions (`src/storage/fs_owner.{c,h}`)

- **`bool fs_owner_enqueue_write_at(const char *path, uint32_t offset, const void *data, uint32_t len)`**
  — a **dedicated third lane** (drained after saves, before logs) so uploads can't starve PID/calib
  saves. Handler: `vfs_open(path, O_RDWR|O_CREAT)` → `vfs_lseek(offset)` → `vfs_write` → `vfs_sync` →
  `vfs_close`. Drop-and-count on full (mirrors the log/save lanes).
- **`int fs_owner_read_at(const char *path, uint32_t offset, void *buf, uint32_t len)`** — synchronous,
  **called only from `xfer_service_task`**. Safe because the kernel `vfs_*` ops already take the global
  `vfs_mutex` (`extern/vaios/kernel/vfs.c`), so the read serialises against `fs_owner_task`'s writes —
  SD stays single-transaction-at-a-time with no new locks. Update the fs_owner doc comment: "sole
  *writer*; reads are mutex-serialised via fs_owner_read_at (xfer-task only)."

## Where it runs

Dedicated **`xfer_service_task` (prio 0, stack 2048)**, added to `init_tasks()` in `src/main.c` after
`fs_owner_task`. NOT piggybacked on telemetry_task (unlike sysid) because file/log providers do
**blocking `vfs_read`** — those belong off both the comm task and the 166 Hz telemetry tick. Body:
register providers, then loop `xfer_tick(v_get_ticks(), channel_tx_overflow_count(), budget)`;
`v_delay(busy ? 4 : 10)`. Optional (Phase F): `xfer_download_active()` lets `imu_telemetry_task`
suppress heavy streams during a big file download (mirrors `sysid_dump_active()`).

## Providers

- **FILE** (`service_id=0`, up+down, `arg`=path): `read`→`fs_owner_read_at`; `write`→
  `fs_owner_enqueue_write_at`; `open(down)` sizes via `vfs_open/size/close` on the xfer task.
- **LOG** (`service_id=1`, download only): FILE pointed at the 3 circular blackbox paths
  (`*_LOGGING_FILENAME`), write disabled (→DENIED). Raw circular dump; GCS de-wraps.
- **STREAM** (`service_id=2`, download stream, `arg`=name e.g. "imu"): `open` subscribes to an existing
  telemetry ring (e.g. `imu_queue_telemetry_pop`), total=0xFFFFFFFF; `poll` returns the next due sample
  or 0. Proves the substrate generalises beyond storage.

## GCS / Python side (`navlink/sim`)

`navlink/sim/xfer_client.py` — `XferClient`: `open(service_id,dir,mode,arg,...)`; download driver
(reassemble sparse-by-offset, periodic `XFER_ACK{next_offset=lowest_missing}`, `NAK` on gap, `DONE`
at total); upload driver (read INFO chunk_size, send from offset, advance/retransmit on XFER_ACK,
final `EOF/DONE`). Reroute `endpoint.py` FC role to a Python `XferServer` for loopback tests instead
of the blanket auto-ack.

## Phasing

- **A — Dialect + conformance:** add messages+enum, regenerate, `navlink/tests/run_tests.py`
  (round-trip + C/Python parity), assert XFER_DATA wire==254. Add `XferClient`. *Lock the wire first.*
- **B — Core SM (host only):** `navlink_xfer.c` + fake in-memory provider + `test_xfer_sm.c`.
- **C — Router + xfer task (firmware):** register handlers, add task, deferred ack + paced emit;
  RAM provider only → SITL end-to-end RAM transfer.
- **D — fs_owner extensions:** write-at lane + read-at; extend `test_fs_owner.c`.
- **E — Real providers:** file/log/stream, registered at boot → SITL FILE round-trip ≤4KB, LOG, stream.
- **F — Hardening + docs:** timeouts, budget sharing across sessions, stream backpressure, optional
  telemetry suppression; update `navlink/docs/reference/messages/` and mark `stream_rates.md` superseded.

## Verification

- **Conformance:** `python3 navlink/tests/run_tests.py` (every new message round-trips + parity; wire≤255).
- **SM unit (`tools/sim_host/tests/test_xfer_sm.c`, link `vayu_sitl_core`, fake provider):** up→down
  roundtrip `memcmp`; resume after dropped chunk (NAK rewinds cursor); out-of-order upload only advances
  on contiguous offset; session-table-full → TEMPORARILY_REJECTED; unknown service → UNSUPPORTED; offset
  past EOF → immediate DONE; provider open error → FAILED.
- **fs_owner (`test_fs_owner.c`):** write-at@offset then read-at compare; saves still drain first; full
  write-at lane drops-and-counts.
- **FILE provider SITL (≤4KB — host VFS `HOST_VFS_FILE_CAP=4096`):** 2KB file up then down, byte-compare.
  Large-file/wrap is on-target only — documented SITL gap (same limit as the fs_owner test).
- **Stream SITL:** open "imu"@10Hz for 1s → ~10 XFER_DATA, monotonic offsets, clean close.

## Risks / edge cases

- **255B cap:** XFER_DATA=254B, 1B margin; drop to data[240] if a field is added (validator backstops).
- **Comm task must not block:** `on_xfer_data` only enqueues; reads only on xfer task; both SD tasks
  prio 0 + vfs_mutex-serialised. **This is the #1 review property.**
- **fs_owner semantics shift:** now sole *writer* + a mutex-serialised reader; guard `fs_owner_read_at`
  as xfer-task-only (comment/assert).
- **Concurrent sessions share one channel budget:** `xfer_tick` round-robins a global `chunk_budget`.
- **Sim auto-ack collision** (8192–12319): reroute XFER_OPEN/CLOSE in Phase A or tests see spurious acks.
- **SITL 4KB cap:** multi-KB / circular-wrap paths are on-target only.

## Critical files

- `navlink/dialect.json` — 5 messages + 1 enum (everything downstream is generated)
- `src/comm/xfer/navlink_xfer.{c,h}`, `src/comm/xfer/navlink_xfer_tx.c`,
  `src/comm/xfer/providers/{file,log,stream}_provider.c` *(new)*
- `src/comm/navlink_router.c` — register `on_xfer_*`, deferred-ack glue (mirror `on_cmd_arm`/`arm_ack_service`)
- `src/storage/fs_owner.{c,h}` — `fs_owner_enqueue_write_at` (new lane) + `fs_owner_read_at`
- `src/main.c` — `xfer_service_task` in `init_tasks()` (prio 0, after fs_owner)
- `navlink/sim/xfer_client.py`, `navlink/sim/endpoint.py` — Python client + reroute auto-ack
- `tools/sim_host/tests/test_xfer_sm.c`, `tools/sim_host/tests/test_fs_owner.c` — tests; register in `tools/sim_host/CMakeLists.txt`
