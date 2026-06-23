# Bulk Transfer & Streaming Substrate (`XFER_*`)

> Authoritative wire spec: `navlink/dialect.json` + `../navlink-v2-spec.md`.
> Design + rationale: `../../../../docs/plans/navlink-xfer-substrate.md`;
> RAM budget: `../../../../docs/plans/xfer-memory-budget.md`.

A generic, reusable "FTP-like" sublayer over NavLink v2: reliable, offset-addressed
**chunked transfer** (download FC→GCS, upload GCS→FC) plus open-ended **streams**.
Future apps — file transfer, blackbox-log download, config/tune export, live data
streams — plug in as **providers** without rebuilding the transport.

## Messages

| Message | msgid | dir | wire | purpose |
| ------- | ----- | --- | ---- | ------- |
| `XFER_OPEN`  | 8201 (cmd) | GCS→FC | 46 B | open a session: dir, mode, `service_id`, `offset_start`, `rate_hz`, `arg` |
| `XFER_INFO`  | 1042 (tlm) | FC→GCS | 12 B | open reply: `result`, negotiated `chunk_size`, `total_size` (0xFFFFFFFF = stream) |
| `XFER_DATA`  | 1043 (tlm) | both   | 254 B| one chunk: `offset`, `len`, `flags` (EOF), `data[247]` |
| `XFER_ACK`   | 1044 (tlm) | both   | 7 B  | cumulative flow-control: `next_offset` (lowest missing byte), `flags` (NAK/DONE/ABORT) |
| `XFER_CLOSE` | 8202 (cmd) | GCS→FC | 5 B  | close/abort a session |

`XFER_OPEN`/`XFER_CLOSE` are in the command range `0x2000–0x2FFF`, so they are
**§10.5 time-sync-gated** and auto-acknowledged with `COMMAND_ACK`. The data-plane
messages carry no `COMMAND_ACK`; the session state machine owns reliability.

`enum xfer_flags { NONE=0, DONE=1, NAK=2, ABORT=4, EOF=8 }`. `XFER_DATA` is 254 B —
one byte under the 255 payload cap (`data[247]`). The receiver **trusts `len`, never
the unpacked array tail**: the final short chunk trailing-zero-truncates on the wire
(like `SYSID_SAMPLE`).

## Reliability model — cumulative-ack + resume-by-rewind

Deliberately **not** a sliding window. It formalises the FC's proven bulk pattern
(near-stateless FC, self-describing chunks, GCS-driven idempotent re-request,
FC-side pacing against channel backpressure):

- **Download (FC→GCS):** the FC streams `XFER_DATA` from a cursor, paced by a shared
  per-tick chunk budget and `channel_tx_overflow_count()`. An `XFER_ACK{next_offset
  < cursor}` (or `flags=NAK`) **rewinds** the cursor — an idempotent re-read that
  resumes/refills dropped chunks. `EOF` marks the last chunk; the GCS replies
  `XFER_ACK{DONE}`.
- **Upload (GCS→FC):** the FC ingests **contiguous-only** chunks (`offset == cursor`);
  a dup (`<cursor`) or gap (`>cursor`) is ignored. It periodically emits
  `XFER_ACK{next_offset = cursor}`, and the GCS retransmits from there. The final
  chunk carries `EOF`.
- **Stream:** `total_size = 0xFFFFFFFF`; the provider is polled at `rate_hz`; never
  rewinds (best-effort), and rising `tx_overflow` skips stream emission first.

## Two-phase open

1. GCS sends `XFER_OPEN`. The FC validates on the comm task (session slot,
   provider, dir/mode capability) — an immediate reject returns the result in
   `COMMAND_ACK` (`TEMPORARILY_REJECTED` slot busy/out-of-range, `UNSUPPORTED`
   unknown `service_id`, `DENIED` bad dir/mode).
2. A valid open is **deferred**: the `xfer_service_task` runs the (possibly
   blocking) `provider->open` off the comm task, then emits `COMMAND_ACK`
   (`ACCEPTED`/`FAILED`) **+** `XFER_INFO`. This upholds the "no blocking SD on the
   comm task" invariant (the centralised FS owner / C1→C3 fix).

## Providers

| `service_id` | name | dir | `arg` | backing |
| ------------ | ---- | --- | ----- | ------- |
| 0 | `file`   | up + down      | SD path        | `fs_owner_read_at` / `fs_owner_enqueue_write_at` |
| 1 | `log`    | download only  | `navlink`/`system`/`general` | blackbox circular file (raw dump; GCS de-wraps). Upload → `DENIED` |
| 2 | `stream` | download stream| source name (e.g. `imu`) | a named live telemetry source @ `rate_hz` |

All storage I/O is mediated by the **fs_owner** gateway: uploads ride a dedicated
async write-at lane (drained after PID/calib saves, before logs); downloads use the
one sanctioned synchronous reader (`fs_owner_read_at`, `vfs_mutex`-serialised,
xfer-task-only).

## Notes

- `chunk_size` is advertised in `XFER_INFO` so neither side hard-codes it.
- Concurrent sessions (default 2) share one channel budget, round-robined per tick.
- A big file download sets `xfer_download_active()`, which lets the telemetry task
  suppress the heaviest tuning streams to hand the link to the transfer (mirrors the
  system-ID dump).
- `XFER_OPEN` folds the stream rate into `rate_hz`, **superseding** the never-shipped
  `CMD_SET_STREAM_RATE` proposal (see `stream_rates.md`).
